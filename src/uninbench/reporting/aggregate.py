"""Aggregation of run records into the tables of the reporting protocol."""

from __future__ import annotations

import csv
import math
from collections import defaultdict
from pathlib import Path

import numpy as np

from .stats import describe, friedman, nemenyi_cd, pairwise_wilcoxon, pareto_front, scaling_exponent

PRIMARY = "normalized_makespan"
QUALITY = ["normalized_makespan", "makespan", "sum_of_costs", "coverage_ratio", "overlap_ratio", "path_length",
           "transit_share", "turns", "heading_change", "workload_imbalance", "uncovered_representation",
           "uncovered_planning", "conflict_instants"]
COMPUTE = ["planning_time", "peak_rss_mb", "adapter_time", "eval_time"]
FAIL = ("incomplete", "invalid", "timeout", "crash")


def variant(rec: dict) -> str:
    """Display name of a planner variant.  ``*`` marks results that are not those of an
    original published algorithm (staged compositions, benchmark baselines, planners
    composed with a framework stage), so they are never reported as originals."""
    name = rec.get("planner", "?")
    fac = (rec.get("factors") or {}).get("planning_resolution")
    if fac is not None and float(fac) != 1.0:
        name += f"@dp{float(fac):g}"
    for st in rec.get("framework_stages") or []:
        name += "+pw" if st == "prioritized_waiting" else f"+{st}"
    if rec.get("composition_of") or rec.get("provenance", "original") != "original":
        name += "*"
    return name


def adapter_attributed(rec: dict) -> bool:
    """A failure caused by the representation (flagged adapter), not by the planner (R3)."""
    return rec.get("failure_attribution") == "adapter"


def group_key(rec: dict):
    return (rec.get("family", "?"), int(rec.get("team_size", 0)), rec.get("config_label", "default"))


def unit_key(rec: dict):
    return rec.get("instance_id") or rec.get("base_instance")


def value(rec: dict, metric: str):
    v = rec.get(metric) if metric in COMPUTE else (rec.get("metrics") or {}).get(metric)
    if isinstance(v, str):  # JSON encodes infinities as strings
        try:
            v = float(v)
        except ValueError:
            return None
    return v


def flatten(rec: dict) -> dict:
    m = rec.get("metrics") or {}
    fid = ((rec.get("adapter") or {}).get("fidelity") or {})
    row = {
        "run_id": rec.get("run_id"), "family": rec.get("family"), "instance_id": rec.get("instance_id"),
        "base_instance": rec.get("base_instance"), "instance_hash": rec.get("instance_hash"),
        "split": rec.get("split"), "team_size": rec.get("team_size"), "planner": rec.get("planner"),
        "variant": variant(rec), "planner_version": rec.get("planner_version"), "mode": rec.get("planner_mode"),
        "provenance": rec.get("provenance"), "composition_of": "+".join(rec.get("composition_of") or []),
        "stochastic": rec.get("stochastic"),
        "config_label": rec.get("config_label"), "config_hash": rec.get("config_hash"), "seed": rec.get("seed"),
        "outcome": rec.get("outcome"), "failure_attribution": rec.get("failure_attribution"),
        "reason": rec.get("reason"), "source_resolution": rec.get("source_resolution"), "delta_p": rec.get("delta_p"),
        "delta_e": rec.get("delta_e"), "adapter": (rec.get("adapter") or {}).get("adapter"),
        "fidelity_rel_error": fid.get("rel_error"), "adapter_flagged": rec.get("adapter_flagged"),
        "sensitivity_stable": (rec.get("sensitivity") or {}).get("stable"),
        "memory_isolated": (rec.get("memory") or {}).get("isolated"),
    }
    for k in COMPUTE:
        row[k] = rec.get(k)
    for k in QUALITY + ["T_LB", "coverable_area", "static_collision_time", "conflict_time"]:
        row[k] = m.get(k)
    for k, v in (rec.get("factors") or {}).items():
        row[f"factor_{k}"] = v
    return row


def write_csv(rows: list[dict], path: Path):
    keys = []
    for r in rows:
        for k in r:
            if k not in keys:
                keys.append(k)
    with open(path, "w", newline="", encoding="utf8") as fh:
        w = csv.DictWriter(fh, fieldnames=keys)
        w.writeheader()
        for r in rows:
            w.writerow(r)


def per_instance(records, metric: str, fail_inf: bool = True):
    """{(group, variant): {unit: median over seeds}} with failures as +inf (or dropped).

    Not-applicable runs and failures attributed to a flagged adapter are left out:
    they say nothing about the planner, so the instance is not compared for it."""
    acc = defaultdict(lambda: defaultdict(list))
    for r in records:
        if r.get("outcome") == "not_applicable" or adapter_attributed(r):
            continue
        if r.get("outcome") == "success":
            v = value(r, metric)
            if v is None:  # not measured (inactive metric, inline memory): no information
                continue
        else:
            v = None
        if v is None or not np.isfinite(v):
            if not fail_inf:
                continue
            v = math.inf
        acc[(group_key(r), variant(r))][unit_key(r)].append(float(v))
    return {k: {u: float(np.median(vs)) for u, vs in d.items()} for k, d in acc.items()}


def success_table(records) -> list[dict]:
    """Outcomes per planner.  ``success_rate`` excludes not-applicable runs and failures
    attributed to a flagged adapter; ``raw_success_rate`` only excludes not-applicable runs."""
    acc = defaultdict(lambda: defaultdict(int))
    for r in records:
        c = acc[(group_key(r), variant(r))]
        c[r.get("outcome", "?")] += 1
        if adapter_attributed(r):
            c["adapter_failures"] += 1
    rows = []
    for ((fam, m, cfg), var), c in sorted(acc.items()):
        n = sum(c.get(o, 0) for o in ("success",) + FAIL)
        n_planner = n - c.get("adapter_failures", 0)
        rows.append({"family": fam, "team_size": m, "config": cfg, "planner": var, "runs": n,
                     "success_rate": c.get("success", 0) / n_planner if n_planner else None,
                     "raw_success_rate": c.get("success", 0) / n if n else None,
                     "adapter_failures": c.get("adapter_failures", 0),
                     **{o: c.get(o, 0) for o in ("success",) + FAIL + ("not_applicable",)}})
    return rows


def descriptive_table(records, metrics=None) -> list[dict]:
    metrics = metrics or QUALITY + COMPUTE
    rows = []
    succ = [r for r in records if r.get("outcome") == "success"]
    for metric in metrics:
        pi = per_instance(succ, metric, fail_inf=False)
        for ((fam, m, cfg), var), d in sorted(pi.items()):
            vals = [v for v in d.values() if np.isfinite(v)]
            if not vals:
                continue
            rows.append({"family": fam, "team_size": m, "config": cfg, "planner": var, "metric": metric,
                         **describe(vals)})
    return rows


def seed_variability(records, metric: str = PRIMARY) -> list[dict]:
    acc = defaultdict(lambda: defaultdict(list))
    for r in records:
        if r.get("stochastic") and r.get("outcome") == "success":
            v = value(r, metric)
            if v is not None and np.isfinite(v):
                acc[(group_key(r), variant(r))][unit_key(r)].append(v)
    rows = []
    for ((fam, m, cfg), var), d in sorted(acc.items()):
        iqr = [float(np.subtract(*np.quantile(v, [0.75, 0.25]))) for v in d.values() if len(v) > 1]
        cv = [float(np.std(v) / np.mean(v)) for v in d.values() if len(v) > 1 and np.mean(v) != 0]
        rows.append({"family": fam, "team_size": m, "config": cfg, "planner": var, "metric": metric,
                     "instances": len(d), "median_seed_iqr": float(np.median(iqr)) if iqr else None,
                     "median_seed_cv": float(np.median(cv)) if cv else None})
    return rows


def comparison_matrices(records, metric: str = PRIMARY):
    """Per comparison group: (units, planners, matrix) with failures = +inf."""
    pi = per_instance(records, metric)
    groups = defaultdict(dict)
    for (g, var), d in pi.items():
        groups[g][var] = d
    out = {}
    for g, planners in groups.items():
        names = sorted(planners)
        units = sorted(set().union(*[set(d) for d in planners.values()]))
        units = [u for u in units if all(u in planners[p] for p in names)]  # only units every planner ran on
        M = np.array([[planners[p][u] for p in names] for u in units]) if units else np.zeros((0, len(names)))
        out[g] = (units, names, M)
    return out


def hypothesis_tests(records, metric: str = PRIMARY, alpha: float = 0.05) -> list[dict]:
    rows = []
    for (fam, m, cfg), (units, names, M) in sorted(comparison_matrices(records, metric).items()):
        if len(names) < 2 or len(units) < 2:
            continue
        fr = friedman(M)
        res = {"family": fam, "team_size": m, "config": cfg, "metric": metric, "planners": names,
               "instances": len(units), "friedman": fr, "cd": nemenyi_cd(len(names), len(units), alpha),
               "pairwise": pairwise_wilcoxon(M, names)}
        rows.append(res)
    return rows


def scaling_table(records, factor: str, metric: str) -> list[dict]:
    """Median and bootstrap CI of a metric against a factor (successful runs)."""
    acc = defaultdict(lambda: defaultdict(list))
    for r in records:
        if r.get("outcome") != "success":
            continue
        if factor == "team_size":
            x = r.get("team_size")
        elif factor == "area":
            f = r.get("factors") or {}
            x = (f.get("width") or 0) * (f.get("height") or 0) or None
        else:
            x = (r.get("factors") or {}).get(factor)
        v = value(r, metric)
        if x is None or v is None or not np.isfinite(v):
            continue
        acc[(r.get("family"), r.get("config_label"), variant(r))][float(x)].append(float(v))
    rows = []
    for (fam, cfg, var), d in sorted(acc.items()):
        xs = sorted(d)
        pts = []
        for x in xs:
            ds = describe(d[x])
            pts.append({"x": x, **ds})
        allx = [x for x in xs for _ in d[x]]
        ally = [v for x in xs for v in d[x]]
        rows.append({"family": fam, "config": cfg, "planner": var, "factor": factor, "metric": metric,
                     "points": pts, "loglog": scaling_exponent(allx, ally)})
    return rows


def pareto_table(records, quality: str = PRIMARY, cost: str = "planning_time") -> list[dict]:
    acc = defaultdict(lambda: defaultdict(lambda: ([], [])))
    for r in records:
        if r.get("outcome") != "success":
            continue
        q, c = value(r, quality), value(r, cost)
        if q is None or c is None:
            continue
        a = acc[(r.get("family"), r.get("config_label"))][variant(r)]
        a[0].append(float(c))
        a[1].append(float(q))
    rows = []
    for (fam, cfg), d in sorted(acc.items()):
        names = sorted(d)
        pts = [(float(np.median(d[n][0])), float(np.median(d[n][1]))) for n in names]
        front = set(pareto_front(pts))
        for k, n in enumerate(names):
            rows.append({"family": fam, "config": cfg, "planner": n, cost: pts[k][0], quality: pts[k][1],
                         "pareto": k in front})
    return rows


def stage_table(records) -> list[dict]:
    """Separate evaluation of intermediate outputs of staged planners (R6): median and IQR of
    every numeric stage metric (e.g. partition imbalance before any route is built)."""
    acc = defaultdict(list)
    for r in records:
        if r.get("outcome") in ("not_applicable", None):
            continue
        for stage, ms in (r.get("stage_metrics") or {}).items():
            for k, v in (ms or {}).items():
                if isinstance(v, bool):
                    v = float(v)
                if isinstance(v, (int, float)) and np.isfinite(v):
                    acc[(r.get("family"), int(r.get("team_size", 0)), variant(r), stage, k)].append(float(v))
    rows = []
    for (fam, m, var, stage, k), vals in sorted(acc.items()):
        d = describe(vals)
        rows.append({"family": fam, "team_size": m, "planner": var, "stage": stage, "metric": k, "n": d["n"],
                     "median": d["median"], "q1": d["q1"], "q3": d["q3"]})
    return rows


def fidelity_table(records) -> list[dict]:
    acc = defaultdict(list)
    for r in records:
        a = r.get("adapter")
        if not a:
            continue
        acc[(r.get("family"), variant(r), a.get("adapter"))].append(r)
    rows = []
    for (fam, var, ad), rs in sorted(acc.items()):
        rel = [((x.get("adapter") or {}).get("fidelity") or {}).get("rel_error") for x in rs]
        rel = [v for v in rel if v is not None]
        rows.append({"family": fam, "planner": var, "adapter": ad,
                     "median_rel_error": float(np.median(rel)) if rel else None,
                     "max_rel_error": float(np.max(rel)) if rel else None,
                     "flagged_runs": int(sum(bool(x.get("adapter_flagged")) for x in rs)), "runs": len(rs),
                     "adapter_failures": int(sum(adapter_attributed(x) for x in rs)),
                     "source_resolution": sorted({x.get("source_resolution") for x in rs
                                                  if x.get("source_resolution") is not None}),
                     "delta_p": sorted({x.get("delta_p") for x in rs if x.get("delta_p") is not None}),
                     "delta_e": sorted({x.get("delta_e") for x in rs if x.get("delta_e") is not None})})
    return rows

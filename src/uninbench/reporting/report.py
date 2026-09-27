"""Statistical report of an experiment: CSV/JSON tables and a self-contained HTML page."""

from __future__ import annotations

import base64
import html
import math
from collections import defaultdict
from pathlib import Path

import numpy as np

from ..runner.experiment import load_runs
from ..util import dump_json, load_json
from . import aggregate as A
from . import plots as P
from .complexity import complexity
from .stats import performance_profile

SCALING_FACTORS = [("team_size", "team size m"), ("area", "environment area [m²]"),
                   ("obstacle_density", "obstacle density"), ("graph_density", "graph density")]


def _fmt(v, nd=3):
    if v is None:
        return "–"
    if isinstance(v, bool):
        return "yes" if v else "no"
    if isinstance(v, (int, np.integer)):
        return str(int(v))
    if isinstance(v, float):
        if math.isinf(v):
            return "∞"
        if math.isnan(v):
            return "–"
        if v != 0 and (abs(v) < 1e-3 or abs(v) >= 1e5):
            return f"{v:.2e}"
        return f"{v:.{nd}f}"
    if isinstance(v, (list, tuple)):
        return ", ".join(_fmt(x, nd) for x in v)
    return html.escape(str(v))


def _table(rows, cols, caption=None) -> str:
    if not rows:
        return "<p class='muted'>No data.</p>"
    head = "".join(f"<th>{html.escape(label)}</th>" for _, label in cols)
    body = "".join("<tr>" + "".join(f"<td>{_fmt(r.get(k))}</td>" for k, _ in cols) + "</tr>" for r in rows)
    cap = f"<caption>{html.escape(caption)}</caption>" if caption else ""
    return f"<div class='tw'><table>{cap}<thead><tr>{head}</tr></thead><tbody>{body}</tbody></table></div>"


def _img(png: bytes, alt: str) -> str:
    return (f"<figure><img alt='{html.escape(alt)}' src='data:image/png;base64,"
            f"{base64.b64encode(png).decode('ascii')}'><figcaption>{html.escape(alt)}</figcaption></figure>")


CSS = """
:root{--surface:#fcfcfb;--page:#f9f9f7;--ink:#0b0b0b;--ink2:#52514e;--muted:#898781;--grid:#e1e0d9;
--border:rgba(11,11,11,.10);--accent:#2a78d6}
@media (prefers-color-scheme:dark){:root:not([data-theme="light"]){--surface:#1a1a19;--page:#0d0d0d;--ink:#fff;
--ink2:#c3c2b7;--muted:#898781;--grid:#2c2c2a;--border:rgba(255,255,255,.10);--accent:#3987e5}}
:root[data-theme="dark"]{--surface:#1a1a19;--page:#0d0d0d;--ink:#fff;--ink2:#c3c2b7;--muted:#898781;
--grid:#2c2c2a;--border:rgba(255,255,255,.10);--accent:#3987e5}
*{box-sizing:border-box}body{margin:0;background:var(--page);color:var(--ink);
font:14px/1.5 system-ui,-apple-system,"Segoe UI",sans-serif}
main{max-width:1100px;margin:0 auto;padding:24px 16px 64px}h1{font-size:24px;margin:0 0 4px}
h2{font-size:18px;margin:36px 0 8px;padding-top:8px;border-top:1px solid var(--grid)}h3{font-size:15px;margin:20px 0 6px}
p,li{color:var(--ink2)}.muted{color:var(--muted)}nav a{color:var(--accent);margin-right:12px;font-size:13px}
.tw{overflow-x:auto;margin:8px 0 16px;border:1px solid var(--border);border-radius:8px;background:var(--surface)}
table{border-collapse:collapse;width:100%;font-variant-numeric:tabular-nums;font-size:12.5px}
caption{text-align:left;padding:8px 10px;color:var(--ink2);font-size:12px}
th,td{padding:5px 10px;text-align:left;border-bottom:1px solid var(--grid);white-space:nowrap}
th{color:var(--ink2);font-weight:600;background:var(--surface);position:sticky;top:0}
figure{margin:12px 0;background:#fcfcfb;border:1px solid var(--border);border-radius:8px;padding:8px}
figure img{max-width:100%;height:auto;display:block}figcaption{font-size:12px;color:#52514e;padding:4px 2px 0}
.kpis{display:flex;flex-wrap:wrap;gap:12px;margin:12px 0}.kpi{background:var(--surface);border:1px solid var(--border);
border-radius:8px;padding:10px 14px;min-width:120px}.kpi b{display:block;font-size:22px}.kpi span{color:var(--ink2);font-size:12px}
code{font-size:12px}
"""


def build_report(results_dir, out_dir=None, metric: str = A.PRIMARY) -> Path:
    results_dir = Path(results_dir)
    out = Path(out_dir) if out_dir else results_dir / "report"
    out.mkdir(parents=True, exist_ok=True)
    recs = load_runs(results_dir)
    exp = load_json(results_dir / "experiment.json") if (results_dir / "experiment.json").exists() else {}
    excluded = load_json(results_dir / "excluded.json").get("excluded", []) if (
        results_dir / "excluded.json").exists() else []
    colors = P.palette([A.variant(r) for r in recs])

    flat = [A.flatten(r) for r in recs]
    if flat:
        A.write_csv(flat, out / "runs.csv")
    success = A.success_table(recs)
    desc = A.descriptive_table(recs)
    seedvar = A.seed_variability(recs, metric)
    tests = A.hypothesis_tests(recs, metric)
    fid = A.fidelity_table(recs)
    pareto = A.pareto_table(recs, metric)
    comp = [complexity(n) for n in sorted({r.get("planner_name") for r in recs if r.get("planner_name")})]
    scaling = {f: A.scaling_table(recs, f, metric) for f, _ in SCALING_FACTORS}
    scaling_time = {f: A.scaling_table(recs, f, "planning_time") for f, _ in SCALING_FACTORS}
    for name, rows in (("success", success), ("descriptives", desc), ("seed_variability", seedvar),
                       ("fidelity", fid), ("pareto", pareto)):
        if rows:
            A.write_csv(rows, out / f"{name}.csv")
    dump_json({"tests": tests, "complexity": comp, "scaling": scaling, "scaling_planning_time": scaling_time},
              out / "stats.json")

    parts = []
    name = exp.get("config", {}).get("name", results_dir.name)
    outcomes = defaultdict(int)
    for r in recs:
        outcomes[r.get("outcome", "?")] += 1
    parts.append(f"<h1>{html.escape(name)} – benchmark report</h1>")
    envd = exp.get("environment", {})
    sw = envd.get("software", {})
    hw = envd.get("hardware", {})
    parts.append(f"<p class='muted'>uninbench {html.escape(str(sw.get('uninbench')))} · git "
                 f"{html.escape(str(sw.get('git')))} · Python {html.escape(str(sw.get('python')))} · "
                 f"{html.escape(str(hw.get('platform')))} · {hw.get('cpu_count')} CPUs</p>")
    parts.append("<nav>" + "".join(f"<a href='#{a}'>{t}</a>" for a, t in (
        ("outcomes", "Outcomes"), ("descriptives", "Descriptives"), ("tests", "Tests"), ("profiles", "Profiles"),
        ("scaling", "Scaling"), ("tradeoffs", "Trade-offs"), ("fidelity", "Fidelity"), ("raw", "Raw data"))) + "</nav>")
    parts.append("<div class='kpis'>" + "".join(
        f"<div class='kpi'><b>{outcomes.get(k, 0)}</b><span>{k.replace('_', ' ')}</span></div>"
        for k in ("success", "incomplete", "invalid", "timeout", "crash", "not_applicable")) +
        f"<div class='kpi'><b>{len(recs)}</b><span>runs recorded</span></div></div>")
    parts.append("<p>Quality metrics are aggregated over successful runs only and are always reported together with "
                 "the success rate. Failed runs are never dropped: in rank tests and performance profiles they "
                 "count as +∞. Stochastic planners enter the tests with their per-instance median over seeds. "
                 "Planner names marked * are staged compositions, not original algorithms.</p>")

    # ---------------------------------------------------------------- outcomes
    parts.append("<h2 id='outcomes'>Outcomes and success rates</h2>")
    parts.append(_img(P.outcome_bars([r for r in success if r["runs"]], "Run outcomes by planner and team size"),
                      "Share of run outcomes; not-applicable combinations are excluded from the denominator."))
    parts.append(_table(success, [("family", "family"), ("team_size", "m"), ("config", "config"),
                                  ("planner", "planner"), ("runs", "runs"), ("success_rate", "success rate"),
                                  ("success", "success"), ("incomplete", "incompl."), ("invalid", "invalid"),
                                  ("timeout", "timeout"), ("crash", "crash"), ("not_applicable", "n/a")]))
    if excluded:
        parts.append("<h3>Not admitted</h3>")
        parts.append(_table(excluded[:200], [("planner", "planner"), ("instance", "instance"), ("reason", "reason")]))

    # ---------------------------------------------------------------- descriptives
    parts.append("<h2 id='descriptives'>Descriptive statistics</h2>")
    parts.append("<p>Medians and interquartile ranges over instances; 95 % confidence intervals by bootstrap "
                 "resampling over instances.</p>")
    for mt in (metric, "coverage_ratio", "overlap_ratio", "workload_imbalance", "planning_time"):
        rows = [r for r in desc if r["metric"] == mt]
        parts.append(f"<h3>{html.escape(mt)}</h3>")
        parts.append(_table(rows, [("family", "family"), ("team_size", "m"), ("config", "config"),
                                   ("planner", "planner"), ("n", "instances"), ("median", "median"), ("q1", "Q1"),
                                   ("q3", "Q3"), ("ci_lo", "CI low"), ("ci_hi", "CI high")]))
    groups = defaultdict(lambda: defaultdict(list))
    for r in recs:
        if r.get("outcome") == "success" and (r.get("metrics") or {}).get(metric) is not None:
            groups[(r.get("family"), r.get("team_size"))][A.variant(r)].append(r["metrics"][metric])
    for (fam, m), d in sorted(groups.items()):
        parts.append(_img(P.box_by_planner(d, f"{metric}: {fam}, m = {m}", metric, colors),
                          f"Distribution of {metric} over successful runs ({fam}, m = {m})."))
    if seedvar:
        parts.append("<h3>Variability across seeds (stochastic planners)</h3>")
        parts.append(_table(seedvar, [("family", "family"), ("team_size", "m"), ("planner", "planner"),
                                      ("instances", "instances"), ("median_seed_iqr", "median seed IQR"),
                                      ("median_seed_cv", "median seed CV")]))

    # ---------------------------------------------------------------- tests
    parts.append("<h2 id='tests'>Hypothesis tests</h2>")
    parts.append("<p>Paired comparisons on the instances every planner ran on. Friedman test on per-instance ranks, "
                 "Nemenyi critical difference (α = 0.05), pairwise Wilcoxon signed-rank tests with Holm "
                 "adjustment; effect size = median paired difference and matched-pairs rank-biserial r.</p>")
    for t in tests:
        fr = t["friedman"]
        parts.append(f"<h3>{html.escape(str(t['family']))}, m = {t['team_size']}, {html.escape(t['config'])} "
                     f"({t['instances']} instances)</h3>")
        parts.append(f"<p>Friedman χ² = {_fmt(fr.get('statistic'))}, p = {_fmt(fr.get('p'), 4)}; "
                     f"Iman–Davenport p = {_fmt(fr.get('p_iman_davenport'), 4)}; CD = {_fmt(t['cd'])}</p>")
        if len(t["planners"]) >= 2 and t["instances"] >= 2:
            parts.append(_img(P.cd_diagram(t["planners"], fr["avg_ranks"], t["cd"], "Average ranks (lower is better)",
                                           colors), "Critical-difference diagram; bars join planners whose average "
                                                    "ranks differ by less than the CD."))
        parts.append(_table(t["pairwise"], [("a", "A"), ("b", "B"), ("n", "pairs"), ("median_diff", "median A−B"),
                                            ("rank_biserial", "r"), ("p", "p"), ("p_holm", "p (Holm)")]))

    # ---------------------------------------------------------------- profiles
    parts.append("<h2 id='profiles'>Performance profiles</h2>")
    parts.append("<p>Fraction of instances on which a planner is within a factor τ of the best planner "
                 "(Dolan &amp; Moré, 2002); failures have an infinite ratio.</p>")
    taus = np.logspace(0, 3, 200, base=2)
    for mt, label in (("makespan", "makespan"), ("planning_time", "planning time")):
        for (fam, m, cfg), (units, names, M) in sorted(A.comparison_matrices(recs, mt).items()):
            if len(units) < 1 or len(names) < 2:
                continue
            rho = performance_profile(M, taus)
            parts.append(_img(P.performance_profile_plot(names, taus, rho, f"{label}: {fam}, m = {m}, {cfg}",
                                                         colors=colors),
                              f"Performance profile of {label} ({len(units)} instances)."))

    # ---------------------------------------------------------------- scaling
    parts.append("<h2 id='scaling'>Scaling</h2>")
    for f, label in SCALING_FACTORS:
        for mt, rows_by, logy in ((metric, scaling, False), ("planning_time", scaling_time, True)):
            rows = rows_by[f]
            fams = sorted({(r["family"], r["config"]) for r in rows})
            for fam, cfg in fams:
                sub = [r for r in rows if r["family"] == fam and r["config"] == cfg and len(r["points"]) > 1]
                if not sub:
                    continue
                parts.append(_img(P.scaling_plot(sub, f"{mt} vs {label}: {fam}, {cfg}", label, mt, logy, colors),
                                  "Median with 95 % bootstrap band; for planning time the labels give the "
                                  "empirical scaling exponent (log–log slope)."))
                if logy:
                    parts.append(_table([{"planner": r["planner"], **(r["loglog"] or {})} for r in sub],
                                        [("planner", "planner"), ("slope", "exponent"), ("r2", "R²"), ("n", "runs")]))

    # ---------------------------------------------------------------- trade-offs
    parts.append("<h2 id='tradeoffs'>Quality–time trade-off and implementation complexity</h2>")
    for fam, cfg in sorted({(r["family"], r["config"]) for r in pareto}):
        sub = [r for r in pareto if r["family"] == fam and r["config"] == cfg]
        parts.append(_img(P.pareto_plot(sub, metric, "planning_time", f"Pareto front: {fam}, {cfg}", colors),
                          "Median quality against median planning time; dashed line = Pareto front."))
    parts.append(_table(comp, [("planner", "planner"), ("mode", "mode"), ("sloc_core", "SLOC (core)"),
                               ("hyperparameters", "hyperparameters"), ("stages", "stages"),
                               ("external_dependencies", "external deps"), ("composition_of", "composition of")],
                        "Coarse indicators; shared modules are counted for every planner that uses them."))

    # ---------------------------------------------------------------- fidelity
    parts.append("<h2 id='fidelity'>Representation fidelity and resolutions</h2>")
    parts.append("<p>Relative reconstruction error ε̄ = μ(F̂ △ F_c)/μ(F_c) at the evaluation resolution. "
                 "Flagged runs exceed the tolerance ε_max; their planning failures are attributed to the adapter. "
                 "δ_p = planning resolution, δ_e = evaluation resolution.</p>")
    parts.append(_table(fid, [("family", "family"), ("planner", "planner"), ("adapter", "adapter"),
                              ("median_rel_error", "median ε̄"), ("max_rel_error", "max ε̄"),
                              ("flagged_runs", "flagged"), ("runs", "runs"), ("delta_p", "δ_p"),
                              ("delta_e", "δ_e")]))
    ul = [r for r in desc if r["metric"] in ("uncovered_representation", "uncovered_planning")]
    parts.append(_table(ul, [("family", "family"), ("team_size", "m"), ("planner", "planner"), ("metric", "loss"),
                             ("median", "median [m²]"), ("q1", "Q1"), ("q3", "Q3")],
                        "Uncovered area split into representation loss and planning loss."))

    parts.append("<h2 id='raw'>Raw data</h2>")
    parts.append("<p>All per-run records, including failed runs: <code>runs.csv</code> (flat), "
                 "<code>../runs.jsonl</code> (full records with adapter records, configurations, seeds, limits, "
                 "hardware and software). Tables: <code>success.csv</code>, <code>descriptives.csv</code>, "
                 "<code>fidelity.csv</code>, <code>pareto.csv</code>, <code>stats.json</code>.</p>")
    page = (f"<!doctype html><html lang='en'><head><meta charset='utf-8'>"
            f"<meta name='viewport' content='width=device-width,initial-scale=1'><title>{html.escape(name)} report"
            f"</title><style>{CSS}</style></head><body><main>{''.join(parts)}</main></body></html>")
    path = out / "index.html"
    path.write_text(page, encoding="utf8")
    return path

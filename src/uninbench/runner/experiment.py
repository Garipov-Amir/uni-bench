"""Experiment runner: which instances, team sizes, seeds and configurations are executed."""

from __future__ import annotations

import json
import logging
import os
import sys
import time
from pathlib import Path

from ..datasets.synthetic import generate_family
from ..datasets.safelog import synthetic_safelog
from ..instance.model import EvalSettings, Instance, TaskSpec
from ..instance.splits import assign_splits, load_instances
from ..instance.team import with_team
from ..planners import admission, get_planner
from ..planners.execution import framework_stages
from ..profiles import apply_profile, profile_adapter_params
from ..util import content_hash, dump_json, load_json, seed_list, to_jsonable
from .config import ExperimentConfig, load_config
from .environment import describe_environment
from .pool import WorkerPool, run_inline
from .worker import RunTask, not_applicable_record

log = logging.getLogger("uninbench.runner")


# ------------------------------------------------------------------ instances
def resolve_instances(cfg: ExperimentConfig, split: str | None = None) -> list[Instance]:
    split = split or cfg.split
    out: list[Instance] = []
    for spec in cfg.instances:
        if isinstance(spec, str):
            spec = {"path": spec}
        if "generate" in spec:
            gen = dict(spec["generate"])
            kind = gen.pop("kind", "synthetic")
            if kind == "synthetic":
                insts = generate_family(**gen)
            elif kind == "safelog":
                insts = synthetic_safelog(**gen)
            else:
                raise ValueError(f"unknown generator {kind!r}")
        else:
            insts = load_instances(spec["path"])
        if spec.get("family"):
            for i in insts:
                i.family = spec["family"]
        if any("split" not in i.metadata for i in insts):
            splits = assign_splits(insts, spec.get("tuning_fraction", 0.2), spec.get("stratify", "auto"))
            for i in insts:
                i.metadata.setdefault("split", splits[i.id])
        want = spec.get("split", split)
        if want and want != "all":
            insts = [i for i in insts if i.metadata.get("split", "test") == want]
        if spec.get("limit"):
            insts = insts[: int(spec["limit"])]
        out.extend(insts)
    return out


def derive_instance(base: Instance, m: int, start_config: str, cfg: ExperimentConfig) -> Instance:
    inst = with_team(base, m, start_config)
    inst = inst.copy()
    inst.task = TaskSpec.from_dict({**inst.task.to_dict(), **cfg.task})
    inst.evaluation = EvalSettings.from_dict({**inst.evaluation.to_dict(), **cfg.evaluation})
    inst.task.profiles = ["core"]
    for p in cfg.profiles:
        if p != "core":
            inst = apply_profile(inst, p, cfg.raw.get("profile_params", {}).get(p))
    inst.__dict__.pop("hash", None)
    return inst


# ------------------------------------------------------------------ configurations
def tuned_config_path(tuning_dir, planner: str, family: str) -> Path:
    return Path(tuning_dir) / f"{planner}__{family}.json"


def resolve_config(label: str, man, spec, cfg: ExperimentConfig, family: str):
    theta0 = man.defaults()
    if label == "default":
        return {**theta0, **spec.params}, ("custom" if spec.params else "default")
    if label == "tuned":
        if not cfg.tuning_dir:
            return None, "tuned"
        p = tuned_config_path(cfg.tuning_dir, man.name, family)
        if not p.exists():
            return None, "tuned"
        rec = load_json(p)
        if not rec.get("frozen", False):
            raise ValueError(f"tuned configuration {p} is not frozen")
        return {**theta0, **rec["theta_star"]}, "tuned"
    raise ValueError(f"unknown configuration label {label!r}")


def validation_dir(cfg: ExperimentConfig) -> Path:
    return Path(cfg.validation_dir or (Path(cfg.output_dir).parent / "validation"))


_VALIDATION: dict = {}


def validation_status(cfg: ExperimentConfig, planner: str):
    """(passed, summary) of the planner's validation report (R9).

    A missing report, or one for another planner version, is produced on the
    spot by running the planner's validation cases; planners that fail are
    excluded from the experiment and the report documents the deviation.
    """
    if not cfg.require_validation:
        return True, None
    man = get_planner(planner).manifest
    p = validation_dir(cfg) / f"{man.name}.json"
    key = (str(p), man.version)
    if key not in _VALIDATION:
        from ..validation import validate_planner, validation_passed

        rep = load_json(p) if p.exists() else None
        if rep is None or rep.get("version") != man.version or "internal_passed" not in rep:
            log.info("validating %s %s (report: %s)", man.name, man.version, p)
            rep = validate_planner(man.name, p.parent)
        ok = validation_passed(rep, bool(cfg.raw.get("require_published_validation", True)))
        _VALIDATION[key] = (ok, rep.get("summary"))
    return _VALIDATION[key]


def adapter_for(man, inst: Instance, cfg: ExperimentConfig) -> tuple[str, dict]:
    """Adapter name and parameters for a planner on an instance (fixed per experiment and family)."""
    if cfg.raw.get("native_graph", True) and man.adapter == "grid" and inst.metadata.get("graph"):
        return "graph", {"coarse": bool(man.adapter_params.get("coarse", False))}
    ap = {**man.adapter_params, **profile_adapter_params(inst, man.adapter), **cfg.adapters.get(man.adapter, {})}
    return man.adapter, ap


def check_resolutions(inst: Instance, adapter: str, ap: dict):
    """R10: the evaluation resolution must be finer than the planning resolution."""
    de = inst.eval_resolution()
    if adapter == "grid":
        r = ap.get("resolution", "auto")
        dp = float(ap.get("resolution_factor", 1.0)) * inst.min_width if r in ("auto", None) else float(r)
    elif adapter == "graph":
        g = inst.metadata["graph"]
        dp = float(g["cell_size"]) / (2 if ap.get("coarse") else 1)  # sub-cells of the graph's vertices
    else:
        return
    if not de < dp:
        raise ValueError(f"{inst.id}: evaluation resolution {de:.4g} m is not finer than the planning resolution "
                         f"{dp:.4g} m; lower evaluation.resolution_factor or raise the planning resolution")


def build_tasks(cfg: ExperimentConfig, instances: list[Instance], out_dir: Path):
    tasks: list[RunTask] = []
    na_records: list[dict] = []
    excluded: list[dict] = []
    warned = set()
    res_factors = cfg.factors.get("planning_resolution", [1.0]) or [1.0]
    time_limit = float(cfg.task.get("time_limit", TaskSpec().time_limit))
    mem = cfg.task.get("memory_limit_mb", TaskSpec().memory_limit_mb)
    for base in instances:
        for m in cfg.team_sizes:
            for sc in cfg.start_configs:
                try:
                    inst = derive_instance(base, int(m), sc, cfg)
                except ValueError as e:
                    excluded.append({"instance": base.id, "team_size": m, "start_config": sc, "reason": str(e)})
                    continue
                inst_dict = inst.to_dict()
                for spec in cfg.planner_specs:
                    cls = get_planner(spec.name)
                    man = cls.manifest
                    ok, why = validation_status(cfg, man.name)
                    if not ok:
                        if (spec.display, "validation") not in warned:
                            excluded.append({"planner": spec.display, "reason": f"failed validation: {why}"})
                            warned.add((spec.display, "validation"))
                        continue
                    dstage = cfg.raw.get("deconfliction_stage")
                    status, why = admission(man, inst, dstage)
                    if status == "excluded":
                        excluded.append({"instance": inst.id, "planner": spec.display, "reason": why})
                        continue
                    adapter, ap0 = adapter_for(man, inst, cfg)
                    factors = [None] if adapter != "grid" else res_factors
                    for fac in factors:
                        ap = dict(ap0)
                        extra = {"start_config": sc}
                        if fac is not None:
                            ap["resolution_factor"] = float(ap.get("resolution_factor", 1.0)) * float(fac)
                            extra["planning_resolution"] = float(fac)
                        check_resolutions(inst, adapter, ap)
                        for label in spec.configs:
                            params, clabel = resolve_config(label, man, spec, cfg, inst.family)
                            if params is None:
                                key = (man.name, inst.family)
                                if key not in warned:
                                    log.warning("no frozen tuned configuration for %s on %s; skipped", *key)
                                    warned.add(key)
                                continue
                            seeds = seed_list(int(cfg.seeds)) if man.stochastic else [0]
                            reps = 1 if man.stochastic else int(cfg.repetitions)
                            if status == "not_applicable":
                                seeds = seeds[:1]
                            for seed in seeds:
                                stages = framework_stages(man, inst, dstage)
                                rid = content_hash({"i": inst.hash, "p": man.name, "v": man.version, "c": params,
                                                    "l": clabel, "s": seed, "a": ap, "ad": adapter,
                                                    "pl": spec.display, "t": time_limit, "f": extra, "st": stages})
                                task = RunTask(run_id=rid, experiment=cfg.name, instance=inst_dict, planner=spec.name,
                                               planner_label=spec.display, params=params, config_label=clabel,
                                               seed=int(seed), repetitions=reps, adapter=adapter,
                                               adapter_params=ap, eps_max=float(cfg.eps_max), time_limit=time_limit,
                                               memory_limit_mb=mem, profiles=list(inst.task.profiles), factors=extra,
                                               output_dir=str(out_dir), store=dict(cfg.store),
                                               deconfliction_stage=dstage)
                                if status == "not_applicable":
                                    na_records.append(not_applicable_record(task, why))
                                else:
                                    tasks.append(task)
    return tasks, na_records, excluded


# ------------------------------------------------------------------ records
def load_runs(results_dir) -> list[dict]:
    p = Path(results_dir) / "runs.jsonl"
    if not p.exists():
        return []
    recs = {}
    with open(p, encoding="utf8") as fh:
        for line in fh:
            line = line.strip()
            if line:
                r = json.loads(line)
                recs[r["run_id"]] = r
    return list(recs.values())


def run_experiment(config, workers: int | None = None, inline: bool | None = None, resume: bool = True,
                   progress: bool = True, output: str | None = None) -> dict:
    cfg = config if isinstance(config, ExperimentConfig) else load_config(config)
    if output:
        cfg.raw["output"] = output
    out = cfg.output_dir
    out.mkdir(parents=True, exist_ok=True)
    manifests = {s.display: get_planner(s.name).manifest.to_dict() for s in cfg.planner_specs}
    dump_json({"config": cfg.to_dict(), "environment": describe_environment(), "planners": manifests,
               "started": time.strftime("%Y-%m-%dT%H:%M:%S")}, out / "experiment.json")
    instances = resolve_instances(cfg)
    tasks, na, excluded = build_tasks(cfg, instances, out)
    done = {r["run_id"] for r in load_runs(out)} if resume else set()
    if not resume and (out / "runs.jsonl").exists():
        (out / "runs.jsonl").unlink()
    tasks = [t for t in tasks if t.run_id not in done]
    na = [r for r in na if r["run_id"] not in done]
    dump_json({"excluded": excluded}, out / "excluded.json")
    counts: dict[str, int] = {}
    fh = open(out / "runs.jsonl", "a", encoding="utf8")

    def on_result(rec):
        fh.write(json.dumps(to_jsonable(rec)) + "\n")
        fh.flush()
        counts[rec.get("outcome", "?")] = counts.get(rec.get("outcome", "?"), 0) + 1

    t0 = time.time()

    tty = sys.stderr.isatty()
    every = os.environ.get("UNINBENCH_PROGRESS_EVERY")  # set to 1 by the UI job runner
    step = max(1, int(every)) if every and every.isdigit() else max(1, len(tasks) // 20)

    def on_progress(k, n, rec):
        if not progress:
            return
        el = time.time() - t0
        eta = el / k * (n - k) if k else 0
        line = (f"[{k}/{n}] {rec.get('planner', '?'):>12s} {rec.get('instance_id', '?')[:40]:40s} "
                f"{rec.get('outcome', '?'):>12s}  elapsed {el:6.0f}s  eta {eta:6.0f}s")
        if tty:
            print("\r" + line, end="", file=sys.stderr, flush=True)
        elif k % step == 0 or k == n:  # plain, line-based progress for logs and captured output
            print(line, file=sys.stderr, flush=True)

    for r in na:
        on_result(r)
    use_inline = cfg.inline if inline is None else inline
    n_workers = workers or int(cfg.workers)
    try:
        if use_inline:
            run_inline(tasks, on_result, on_progress)
        else:
            mem = cfg.task.get("memory_limit_mb", TaskSpec().memory_limit_mb)
            pool = WorkerPool(n_workers, mem, float(cfg.grace), cfg.raw.get("max_tasks_per_worker", 1))
            try:
                pool.run(tasks, on_result, on_progress)
            finally:
                pool.close()
    finally:
        fh.close()
        if progress and tasks and tty:
            print(file=sys.stderr)
    summary = {"runs": len(tasks) + len(na), "skipped_existing": len(done), "outcomes": counts,
               "excluded": len(excluded), "output": str(out), "wall_time": time.time() - t0}
    dump_json(summary, out / "summary.json")
    return summary

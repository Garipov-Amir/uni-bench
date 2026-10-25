"""Equal, systematic tuning of every planner on the tuning split (R8).

For each (planner, instance family) the hyperparameters are tuned by Bayesian
optimisation with the same budget: ``budget`` configuration evaluations, each
over all tuning instances, team sizes and start configurations (and a fixed set
of ``tuning_seeds`` seeds for stochastic planners, whose results are averaged).
The objective is the mean normalised makespan; incomplete, invalid, timed-out
and crashed runs receive a fixed penalty.  Only planners that passed validation
are tuned (R9 before R8).  The best configuration theta* is frozen before the
test split is used; the record states the number of planner calls spent.
"""

from __future__ import annotations

import json
import math
import time
from collections import defaultdict
from pathlib import Path

import numpy as np

from ..planners import admission, get_planner
from ..planners.execution import framework_stages
from ..util import content_hash, dump_json, seed_list, to_jsonable
from ..runner.config import ExperimentConfig, load_config
from ..runner.experiment import (adapter_for, check_resolutions, derive_instance, resolve_instances,
                                 tuned_config_path, validation_status)
from ..runner.pool import WorkerPool, run_inline
from ..runner.worker import RunTask
from .bo import Space, bayes_opt

DEFAULT_PENALTY = 10.0


def _tasks(cfg, planner_name, params, instances, seeds, out_dir):
    cls = get_planner(planner_name)
    man = cls.manifest
    time_limit = float(cfg.task.get("time_limit", 60.0))
    tasks = []
    for base in instances:
        for m in cfg.team_sizes:
            for sc in cfg.start_configs:
                try:
                    inst = derive_instance(base, int(m), sc, cfg)
                except ValueError:
                    continue
                dstage = cfg.raw.get("deconfliction_stage")
                status, _ = admission(man, inst, dstage)
                if status != "admitted":
                    continue
                adapter, ap = adapter_for(man, inst, cfg)
                check_resolutions(inst, adapter, ap)
                for seed in seeds:
                    rid = content_hash({"tune": True, "i": inst.hash, "p": man.name, "c": params, "s": seed, "a": ap,
                                        "st": framework_stages(man, inst, dstage)})
                    tasks.append(RunTask(run_id=rid, experiment=f"tune:{cfg.name}", instance=inst.to_dict(),
                                         planner=planner_name, planner_label=planner_name, params=params,
                                         config_label="tuning", seed=int(seed), repetitions=1, adapter=adapter,
                                         adapter_params=ap, eps_max=float(cfg.eps_max), time_limit=time_limit,
                                         memory_limit_mb=cfg.task.get("memory_limit_mb"),
                                         profiles=list(inst.task.profiles), factors={"start_config": sc},
                                         output_dir=str(out_dir), store={"trajectories": False, "coverage": False,
                                                                         "events": False},
                                         deconfliction_stage=dstage))
    return tasks


def score(rec: dict, penalty: float) -> float:
    if rec.get("outcome") == "success":
        v = rec.get("metrics", {}).get("normalized_makespan")
        if v is not None and math.isfinite(v):
            return float(v)
    return penalty


def tune_planner(cfg: ExperimentConfig, planner_name: str, family: str, instances, budget: int = 20,
                 seed: int = 0, penalty: float = DEFAULT_PENALTY, tuning_seeds: int = 3, workers: int = 1,
                 inline: bool = False, out_dir: Path | None = None, verbose: bool = True) -> dict:
    cls = get_planner(planner_name)
    man = cls.manifest
    space = Space(man.hyperparameters)
    seeds = seed_list(tuning_seeds) if man.stochastic else [0]
    out_dir = Path(out_dir or cfg.tuning_dir or (cfg.output_dir / "tuning"))
    out_dir.mkdir(parents=True, exist_ok=True)
    log_path = out_dir / f"{planner_name}__{family}.runs.jsonl"
    pool = None if inline else WorkerPool(workers, cfg.task.get("memory_limit_mb"), float(cfg.grace),
                                          cfg.raw.get("max_tasks_per_worker", 1))
    history = []

    def objective(theta):
        params = {**man.defaults(), **theta}
        tasks = _tasks(cfg, planner_name, params, instances, seeds, out_dir)
        recs = []
        (run_inline if inline else pool.run)(tasks, recs.append)
        with open(log_path, "a", encoding="utf8") as fh:
            for r in recs:
                fh.write(json.dumps(to_jsonable({k: r.get(k) for k in ("run_id", "instance_id", "team_size", "seed",
                                                                        "outcome", "config")}
                                                | {"normalized_makespan": r.get("metrics", {}).get(
                                                    "normalized_makespan")})) + "\n")
        vals = [score(r, penalty) for r in recs]
        val = float(np.mean(vals)) if vals else penalty
        history.append({"theta": params, "objective": val, "runs": len(recs),
                        "success_rate": float(np.mean([r.get("outcome") == "success" for r in recs])) if recs else 0})
        return val

    def log(k, th, y):
        if verbose:
            print(f"  [{planner_name}/{family}] eval {k}/{budget}: objective {y:.4f}  {th}", flush=True)

    t0 = time.time()
    try:
        bayes_opt(objective, space, budget if space.dim else 1, np.random.default_rng(seed), theta0=man.defaults(),
                  log=log)
    finally:
        if pool is not None:
            pool.close()
    best = min(history, key=lambda h: h["objective"]) if history else {"theta": man.defaults(), "objective": None}
    rec = {
        "planner": planner_name,
        "planner_version": man.version,
        "family": family,
        "budget": budget,
        "optimizer": "gp-ei (Matern 5/2)",
        "objective": "mean normalized makespan, penalty for unsuccessful runs",
        "penalty": penalty,
        "seed": seed,
        "tuning_seeds": seeds,
        "planner_calls": int(sum(h["runs"] for h in history)),
        "instances": [i.id for i in instances],
        "split": "tuning",
        "theta0": man.defaults(),
        "objective_theta0": history[0]["objective"] if history else None,
        "theta_star": best["theta"],
        "objective_star": best["objective"],
        "history": history,
        "wall_time": time.time() - t0,
        "frozen": True,
        "frozen_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
    }
    rec["hash"] = content_hash(rec["theta_star"])
    dump_json(rec, tuned_config_path(out_dir, planner_name, family))
    return rec


def tune_experiment(config, budget: int = 20, planners=None, workers: int | None = None, inline: bool | None = None,
                    seed: int = 0, penalty: float = DEFAULT_PENALTY, tuning_seeds: int = 3, verbose: bool = True):
    cfg = config if isinstance(config, ExperimentConfig) else load_config(config)
    instances = resolve_instances(cfg, split="tuning")
    by_family = defaultdict(list)
    for i in instances:
        by_family[i.family].append(i)
    out = {}
    for spec in cfg.planner_specs:
        if planners and spec.name not in planners:
            continue
        ok, why = validation_status(cfg, spec.name)
        if not ok:
            if verbose:
                print(f"skipping {spec.name}: failed validation ({why})", flush=True)
            continue
        for fam, insts in sorted(by_family.items()):
            if verbose:
                print(f"tuning {spec.name} on {fam}: {len(insts)} tuning instances, budget {budget}", flush=True)
            out[(spec.name, fam)] = tune_planner(cfg, spec.name, fam, insts, budget=budget, seed=seed,
                                                 penalty=penalty, tuning_seeds=tuning_seeds,
                                                 workers=workers or int(cfg.workers),
                                                 inline=cfg.inline if inline is None else inline, verbose=verbose)
    return out

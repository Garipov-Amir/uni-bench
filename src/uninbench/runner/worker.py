"""Execution of a single run (adapter -> planner -> evaluator) inside a worker.

Messages sent to the parent through ``send``:

* ``("planning", time_limit)`` -- a planner call starts (the parent arms the timeout),
* ``("planned", seconds)``     -- the planner call returned (timeout disarmed),
* ``("result", record)``       -- the final run record.
"""

from __future__ import annotations

import gc
import os
import resource
import sys
import time
import traceback
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from ..adapters import make_adapter
from ..evaluator import evaluate
from ..instance.model import Instance
from ..planners import PlanContext, get_planner
from ..util import content_hash, dump_json, pack_array
from .environment import describe_environment

OUTCOMES = ("success", "incomplete", "invalid", "timeout", "crash", "not_applicable")


@dataclass
class RunTask:
    run_id: str
    experiment: str
    instance: dict
    planner: str
    planner_label: str
    params: dict
    config_label: str
    seed: int
    repetitions: int
    adapter: str
    adapter_params: dict
    eps_max: float
    time_limit: float
    memory_limit_mb: int | None
    profiles: list
    factors: dict
    output_dir: str
    store: dict = field(default_factory=dict)


def _rss_mb() -> float:
    r = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    return r / (1024 * 1024) if sys.platform == "darwin" else r / 1024


_ADAPTER_CACHE: dict = {}
_INSTANCE_CACHE: dict = {}


def set_memory_limit(mb: int | None):
    if not mb or sys.platform == "darwin":  # RLIMIT_AS is not enforced on macOS
        return
    try:
        lim = int(mb) * 1024 * 1024
        resource.setrlimit(resource.RLIMIT_AS, (lim, lim))
    except (ValueError, OSError):
        pass


def _instance(d: dict) -> Instance:
    key = d.get("hash") or content_hash(d)
    if key not in _INSTANCE_CACHE:
        if len(_INSTANCE_CACHE) > 16:
            _INSTANCE_CACHE.clear()
        _INSTANCE_CACHE[key] = Instance.from_dict(d)
    return _INSTANCE_CACHE[key]


def _adapter(inst: Instance, name: str, params: dict, eps_max: float):
    key = (inst.hash, name, content_hash(params), eps_max)
    if key not in _ADAPTER_CACHE:
        if len(_ADAPTER_CACHE) > 8:
            _ADAPTER_CACHE.clear()
        _ADAPTER_CACHE[key] = make_adapter(name, **params)(inst, eps_max=eps_max)
    return _ADAPTER_CACHE[key]


def base_record(task: RunTask, inst: Instance, planner_cls) -> dict:
    man = planner_cls.manifest
    env = describe_environment()
    return {
        "run_id": task.run_id,
        "experiment": task.experiment,
        "instance_id": inst.id,
        "base_instance": inst.metadata.get("base_id", inst.id),
        "instance_hash": inst.hash,
        "family": inst.family,
        "split": inst.metadata.get("split"),
        "factors": {**inst.metadata.get("factors", {}), **task.factors},
        "team_size": inst.m,
        "planner": task.planner_label,
        "planner_name": man.name,
        "planner_version": man.version,
        "planner_mode": man.mode,
        "composition_of": man.composition_of,
        "stochastic": man.stochastic,
        "config_label": task.config_label,
        "config": task.params,
        "config_hash": content_hash(task.params, 10),
        "seed": task.seed,
        "profiles": task.profiles,
        "limits": {"time_s": task.time_limit, "memory_mb": task.memory_limit_mb},
        "hardware": env["hardware"],
        "software": env["software"],
        "started": time.strftime("%Y-%m-%dT%H:%M:%S"),
    }


def execute(task: RunTask, send) -> dict:
    inst = _instance(task.instance)
    planner_cls = get_planner(task.planner)
    rec = base_record(task, inst, planner_cls)
    try:
        rep, arec = _adapter(inst, task.adapter, task.adapter_params, task.eps_max)
    except Exception as e:  # adapter failure is attributed to the adapter, not the planner
        rec.update(outcome="crash", reason=f"adapter error: {e!r}", traceback=traceback.format_exc())
        send(("result", rec))
        return rec
    rec["adapter"] = arec.to_dict()
    rec["adapter_flagged"] = arec.flagged
    rec["delta_p"] = arec.params.get("resolution")
    rec["adapter_time"] = arec.runtime
    times, digests = [], []
    result = None
    rss0 = _rss_mb()
    try:
        planner = planner_cls(task.params)
        for k in range(max(1, task.repetitions)):
            gc.collect()
            ctx = PlanContext(seed=task.seed, time_limit=task.time_limit, profiles=set(task.profiles),
                              return_to_depot=bool(inst.task.return_to_depot))
            send(("planning", task.time_limit))
            t0 = time.perf_counter()
            res = planner.plan(rep, ctx)
            dt = time.perf_counter() - t0
            send(("planned", dt))
            times.append(dt)
            digests.append(content_hash(res.trajectories.to_dict()))
            if result is None:
                result = res
            if dt > task.time_limit:
                break
    except MemoryError:
        rec.update(outcome="crash", reason="memory limit exceeded", traceback=traceback.format_exc())
        send(("result", rec))
        return rec
    except Exception as e:
        rec.update(outcome="crash", reason=repr(e), traceback=traceback.format_exc())
        send(("result", rec))
        return rec
    rec["planning_time"] = float(np.median(times))
    rec["planning_times"] = times
    rec["peak_rss_mb"] = _rss_mb()
    rec["rss_growth_mb"] = max(0.0, _rss_mb() - rss0)
    rec["deterministic_check"] = len(set(digests)) <= 1
    if max(times) > task.time_limit:
        rec.update(outcome="timeout", reason=f"planning took {max(times):.3g} s > {task.time_limit} s")
        send(("result", rec))
        return rec
    rec["stage_metrics"] = result.stage_metrics
    rec["stage_outputs"] = result.stage_outputs
    rec["planner_info"] = result.info
    try:
        ev = evaluate(inst, result.trajectories, rep.reconstruction, record_events=bool(task.store.get("events")),
                      profiles=task.profiles)
    except Exception as e:
        rec.update(outcome="crash", reason=f"evaluation error: {e!r}", traceback=traceback.format_exc())
        send(("result", rec))
        return rec
    rec["outcome"] = ev.status
    rec["reason"] = "; ".join(ev.validity.reasons) or None
    rec["warnings"] = ev.validity.warnings
    rec["metrics"] = {k: v for k, v in ev.metrics.items() if k != "per_robot"}
    rec["per_robot"] = ev.metrics.get("per_robot")
    rec["delta_e"] = ev.resolution
    rec["sensitivity"] = ev.sensitivity
    rec["eval_time"] = ev.eval_time
    rec["artifacts"] = _store(task, inst, result, ev, rep)
    rec["finished"] = time.strftime("%Y-%m-%dT%H:%M:%S")
    send(("result", rec))
    return rec


def _store(task: RunTask, inst: Instance, result, ev, rep) -> dict:
    out = Path(task.output_dir)
    arts = {}
    inst_path = out / "instances" / f"{inst.id}__{inst.hash}.json"
    if not inst_path.exists():
        inst.save(inst_path)
    arts["instance"] = str(inst_path.relative_to(out))
    if task.store.get("trajectories", True):
        p = out / "runs" / f"{task.run_id}.json.gz"
        payload = {
            "run_id": task.run_id,
            "trajectories": result.trajectories.to_dict(),
            "retimed": ev.retimed.to_dict() if ev.retimed is not None else None,
            "layers": result.layers,
            "adapter_layers": rep.layers(),
            "reconstruction": rep.reconstruction.to_dict() if rep.reconstruction is not None else None,
        }
        dump_json(payload, p, indent=None)
        arts["trajectories"] = str(p.relative_to(out))
    if task.store.get("coverage", True) and ev.coverage is not None:
        cm = ev.coverage
        p = out / "runs" / f"{task.run_id}.coverage.json.gz"
        payload = {"grid": cm.grid.to_dict(), "counts": pack_array(np.minimum(cm.counts, 65535).astype(np.uint16)),
                   "owner": pack_array(cm.owner), "robots": cm.robots}
        if cm.events is not None:
            payload["events"] = {"cell": pack_array(cm.events["cell"].astype(np.int32)),
                                 "t": pack_array(cm.events["t"].astype(np.float32)),
                                 "robot": pack_array(cm.events["robot"].astype(np.int16))}
        dump_json(payload, p, indent=None)
        arts["coverage"] = str(p.relative_to(out))
    return arts


def worker_main(conn, memory_limit_mb):
    """Long-lived worker process loop."""
    set_memory_limit(memory_limit_mb)
    os.environ.setdefault("OMP_NUM_THREADS", "1")

    def send(msg):
        conn.send(msg)

    while True:
        try:
            task = conn.recv()
        except (EOFError, KeyboardInterrupt):
            break
        if task is None:
            break
        try:
            execute(task, send)
        except Exception as e:  # last resort: never let the worker die silently
            try:
                rec = base_record(task, _instance(task.instance), get_planner(task.planner))
            except Exception:
                rec = {"run_id": task.run_id, "planner": task.planner_label, "seed": task.seed}
            rec.update(outcome="crash", reason=repr(e), traceback=traceback.format_exc())
            conn.send(("result", rec))


def not_applicable_record(task: RunTask, reason: str) -> dict:
    inst = _instance(task.instance)
    rec = base_record(task, inst, get_planner(task.planner))
    rec.update(outcome="not_applicable", reason=reason)
    return rec


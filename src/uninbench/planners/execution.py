"""Running a planner the same way everywhere (runner, tuning, validation).

* :func:`framework_stages` decides which framework-provided stages are
  composed with a planner.  The only one is ``prioritized_waiting``
  deconfliction: a planner that does not model inter-robot conflicts itself can
  take part in the deconfliction profile only as the *composition* "planner +
  prioritized waiting", which is recorded and reported as such (R6, R12).
* :func:`run_planner` executes one planner call followed by the composed
  stages; both are part of the measured planning time.
* :func:`conservative_labels` implements the output convention for planners
  whose manifest declares that they do not distinguish service from transit:
  every motion segment is labelled service with the tool active, so that the
  evaluator counts overlap conservatively.
"""

from __future__ import annotations

import numpy as np

from ..trajectory import SERVICE, TRANSIT, Trajectory, TrajectorySet
from .base import Manifest, PlanContext, PlanResult, required_constraints

DECONFLICTION_STAGES = ("prioritized_waiting",)


def framework_stages(manifest: Manifest, inst, deconfliction_stage: str | None = None) -> list[str]:
    """Framework stages composed with ``manifest``'s planner on ``inst``."""
    if deconfliction_stage is None or inst.m < 2:
        return []
    if deconfliction_stage not in DECONFLICTION_STAGES:
        raise ValueError(f"unknown deconfliction stage {deconfliction_stage!r}; available: {DECONFLICTION_STAGES}")
    needs = "deconfliction" in required_constraints(inst)
    if needs and "deconfliction" not in manifest.constraints:
        return [deconfliction_stage]
    return []


def run_planner(planner, rep, ctx: PlanContext, stages=()) -> PlanResult:
    res = planner.plan(rep, ctx)
    for st in stages:
        if st == "prioritized_waiting":
            from .deconflict import deconflict

            ts, info = deconflict(res.trajectories, rep.team)
            ts.metadata.update(info)
            res.trajectories = ts
            res.info.update(info)
        else:  # pragma: no cover - guarded by framework_stages
            raise ValueError(st)
    return res


def conservative_labels(ts: TrajectorySet) -> TrajectorySet:
    """Relabel every transit segment as service with the tool active."""
    out = TrajectorySet(metadata=dict(ts.metadata))
    for tr in ts:
        labels = tr.labels.copy()
        active = tr.active.copy()
        tra = labels == TRANSIT
        labels[tra] = SERVICE
        active[tra] = True
        out.add(Trajectory(tr.robot_id, tr.xy.copy(), tr.psi.copy(), labels, active, tr.wait.copy(),
                           None if tr.t is None else np.asarray(tr.t).copy()))
    return out


def finalize_output(res: PlanResult, manifest: Manifest) -> PlanResult:
    """Apply the manifest's output conventions to a plan (not part of planning time)."""
    if not manifest.distinguishes_service_transit:
        res.trajectories = conservative_labels(res.trajectories)
    return res

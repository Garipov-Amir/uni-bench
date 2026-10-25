"""Planner-independent evaluator  M(E_c, Pi_i)  (R5, R10).

Inputs are only the canonical instance, the trajectory set and -- for the
decomposition of the uncovered area -- the adapter reconstruction \\hat F_i.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field

import numpy as np

from ..geometry.raster import RasterGrid
from ..trajectory import (SERVICE, TRANSIT, TURN, TrajectorySet, Validity, canonicalize_set, retime_set,
                          validate_set, validate_turning_radius)
from .collisions import capacity_violations, inter_robot_conflicts, static_collisions
from .coverage import CoverageMap, coverage_map

# metrics that depend on the evaluation raster (all others are computed from the
# trajectories alone and do not change with delta_e); the value is the floor of the
# denominator of the relative change: ratios use 1 (coverage) or 0.05 (overlap),
# areas the coverable area mu(F*)
RASTER_METRICS = {"coverage_ratio": 1.0, "overlap_ratio": 0.05, "uncovered_area": "area",
                  "uncovered_representation": "area", "uncovered_planning": "area"}
COVERAGE_KEYS = tuple(RASTER_METRICS)
ALWAYS_REPORTED = ("coverage_ratio", "coverable_area", "T_LB")  # needed for the outcome and normalisation
MAX_SENSITIVITY_CELLS = 2.5e7


def _max_rel_change(coarse: dict, fine: dict, area: float) -> tuple[float, str]:
    worst, which = 0.0, ""
    for k, floor in RASTER_METRICS.items():
        a, b = coarse.get(k), fine.get(k)
        if a is None or b is None:
            continue
        den = max(abs(b), area if floor == "area" else floor, 1e-12)
        rel = abs(a - b) / den
        if rel > worst:
            worst, which = rel, k
    return worst, which


@dataclass
class Evaluation:
    status: str  # success | incomplete | invalid
    metrics: dict
    validity: Validity
    resolution: float
    sensitivity: dict
    coverage: CoverageMap | None = None
    retimed: TrajectorySet | None = None
    eval_time: float = 0.0
    notes: list = field(default_factory=list)

    def summary(self) -> dict:
        return {"status": self.status, "reasons": self.validity.reasons, "warnings": self.validity.warnings,
                "delta_e": self.resolution, "sensitivity": self.sensitivity, "eval_time": self.eval_time,
                **self.metrics}


def eval_grid(inst, res: float) -> RasterGrid:
    pad = max(r.tool.radius for r in inst.team) + res
    return RasterGrid.covering(inst.environment.bounds, res, pad=pad)


def coverage_metrics(inst, retimed: TrajectorySet, res: float, reconstruction=None, record_events=False):
    grid = eval_grid(inst, res)
    cmap = coverage_map(grid, list(retimed), inst.team, record_events=record_events)
    fstar = grid.rasterize(inst.derived.coverable)
    A = grid.cell_area
    n = cmap.counts
    covered = (n >= 1) & fstar
    mu = max(float(fstar.sum()) * A, 1e-12)
    uncovered = fstar & ~covered
    m = {
        "coverage_ratio": float(covered.sum()) * A / mu,
        "uncovered_area": float(uncovered.sum()) * A,
        "overlap_ratio": float(np.maximum(n[fstar] - 1, 0).sum()) * A / mu,
        "coverable_area_raster": mu,
        "swept_outside_coverable": float(((n >= 1) & ~fstar).sum()) * A,
    }
    if reconstruction is not None:
        rec = reconstruction.mask(grid)
        m["uncovered_representation"] = float((uncovered & ~rec).sum()) * A
        m["uncovered_planning"] = float((uncovered & rec).sum()) * A
    else:
        m["uncovered_representation"] = None
        m["uncovered_planning"] = m["uncovered_area"]
    return m, cmap


def _turns(tr, merge_dist: float):
    L = tr.seg_lengths()
    d = tr.heading_changes()
    total = float(d.sum())
    count = 0
    since = math.inf  # travelled length since last heading change
    for k in range(tr.K):
        if d[k] > 1e-6:
            if since > merge_dist:
                count += 1
            since = 0.0
        if tr.labels[k] != TURN:
            since += L[k]
    return count, total


def evaluate(inst, trajectories: TrajectorySet, reconstruction=None, record_events: bool = False,
             sensitivity: bool | None = None, profiles=None) -> Evaluation:
    t0 = time.perf_counter()
    es = inst.evaluation
    profiles = set(profiles if profiles is not None else inst.task.profiles)
    res = inst.eval_resolution()
    pos_tol = max(1e-6, 0.5 * res)
    agri = "agricultural" in profiles
    validity = validate_set(trajectories, inst, pos_tol)
    if not validity.valid:
        return Evaluation("invalid", {}, validity, res, {}, eval_time=time.perf_counter() - t0)
    canonical, inserted = canonicalize_set(trajectories, inst.team)
    if inserted:
        validity.warnings.append("heading changes without turn segments were turned into explicit turns: " +
                                 ", ".join(f"{rid} {n}" for rid, n in sorted(inserted.items())))
    if agri:
        validate_turning_radius(canonical, inst, validity)
        if not validity.valid:
            return Evaluation("invalid", {}, validity, res, {}, eval_time=time.perf_counter() - t0)
    retimed = retime_set(canonical, inst.team)
    robots = {r.id: r for r in inst.team}

    # ------------------------------------------------ coverage and resolution
    sens = {"enabled": False}
    do_sens = es.sensitivity if sensitivity is None else sensitivity
    cov, cmap = coverage_metrics(inst, retimed, res, reconstruction, record_events)
    if do_sens:
        sens = {"enabled": True, "tolerance": es.sensitivity_tolerance, "refinements": 0, "checks": []}
        for _ in range(es.max_refinements + 1):
            g = eval_grid(inst, res / 2)
            if g.nx * g.ny > MAX_SENSITIVITY_CELLS:
                sens["skipped"] = "grid too large for the refined check"
                break
            fine, fine_map = coverage_metrics(inst, retimed, res / 2, reconstruction, record_events)
            rel, which = _max_rel_change(cov, fine, inst.derived.coverable_area)
            sens["checks"].append({"delta_e": res, "delta_check": res / 2, "max_rel_change": rel, "metric": which})
            if rel <= es.sensitivity_tolerance:
                break
            if sens["refinements"] >= es.max_refinements:
                sens["unstable"] = True
                break
            res, cov, cmap = res / 2, fine, fine_map
            sens["refinements"] += 1
        sens["stable"] = not sens.get("unstable", False) and "skipped" not in sens

    # ------------------------------------------------ time, length, turns
    merge = es.turn_merge_distance if es.turn_merge_distance is not None else 0.5 * inst.min_width
    T, L, Ls, Lt, turns, dpsi = {}, {}, {}, {}, {}, {}
    for tr in retimed:
        T[tr.robot_id] = tr.end_time
        L[tr.robot_id] = tr.length()
        Ls[tr.robot_id] = tr.length(SERVICE)
        Lt[tr.robot_id] = tr.length(TRANSIT)
        turns[tr.robot_id], dpsi[tr.robot_id] = _turns(tr, merge)
    Tv = np.array(list(T.values()))
    Tmax = float(Tv.max()) if len(Tv) else 0.0
    Lsum = float(sum(L.values()))
    T_LB = inst.derived.T_LB
    metrics = dict(cov)
    metrics.update({
        "path_length": Lsum,
        "service_length": float(sum(Ls.values())),
        "transit_length": float(sum(Lt.values())),
        "transit_share": float(sum(Lt.values())) / Lsum if Lsum > 0 else 0.0,
        "makespan": Tmax,
        "sum_of_costs": float(Tv.sum()),
        "T_LB": T_LB,
        "normalized_makespan": Tmax / T_LB if T_LB > 0 else math.inf,
        "turns": int(sum(turns.values())),
        "heading_change": float(sum(dpsi.values())),
        "workload_imbalance": Tmax / float(Tv.mean()) if len(Tv) and Tv.mean() > 0 else math.nan,
        "per_robot": {rid: {"T": T[rid], "L": L[rid], "service": Ls[rid], "transit": Lt[rid], "turns": turns[rid]}
                      for rid in T},
        "coverable_area": inst.derived.coverable_area,
    })

    # ------------------------------------------------ collisions and conflicts
    # penetration-depth tolerance: bodies may touch obstacles, and numerical grazing
    # below 1e-4 working widths (or 1e-6 of the workspace extent) is not a collision
    minx, miny, maxx, maxy = inst.environment.bounds
    tol = max(1e-4 * inst.min_width, 1e-6 * max(maxx - minx, maxy - miny))
    ct, cl = 0.0, 0.0
    for tr in retimed:
        a, b = static_collisions(tr, robots[tr.robot_id].body_fp, inst.free, tol, res)
        ct += a
        cl += b
    metrics["static_collision_time"] = ct
    metrics["static_collision_length"] = cl
    metrics.update(inter_robot_conflicts(list(retimed), inst.team, res))
    if ct > 0 and es.collision == "invalidate":
        validity.fail(f"static collision for {ct:.4g} s ({cl:.4g} m)")
    enforce_conflicts = es.conflicts == "enforce" or "deconfliction" in profiles
    if enforce_conflicts and metrics["conflict_instants"] > 0:
        validity.fail(f"inter-robot conflicts at {metrics['conflict_instants']} sampled instants")
    if agri or any(r.capacity for r in inst.team):
        worst = max(capacity_violations(tr, robots[tr.robot_id], pos_tol) for tr in retimed)
        metrics["capacity_excess"] = worst if math.isfinite(worst) else None
        if math.isfinite(worst) and worst > 1e-6 * max(1.0, inst.min_width ** 2):
            validity.fail(f"capacity exceeded by {worst:.4g} m^2")

    if not validity.valid:
        status = "invalid"
    elif metrics["coverage_ratio"] < 1.0 - inst.task.tolerance:
        status = "incomplete"
    else:
        status = "success"
    if es.metrics:  # active metric set of the instance's evaluation settings
        keep = set(es.metrics) | set(ALWAYS_REPORTED) | {"per_robot"}
        metrics = {k: v for k, v in metrics.items() if k in keep}
    return Evaluation(status, metrics, validity, res, sens, coverage=cmap, retimed=retimed,
                      eval_time=time.perf_counter() - t0)

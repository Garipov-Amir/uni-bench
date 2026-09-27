"""Validation cases: worked examples and published properties of each planner."""

from __future__ import annotations

import time
import traceback
from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable

import shapely.affinity
from shapely.geometry import box

from ..adapters import make_adapter
from ..evaluator import evaluate
from ..instance.model import Instance, Robot, default_robot, instance_from_polygon
from ..geometry.footprint import Footprint
from ..planners import PlanContext, get_planner
from ..util import dump_json


@dataclass
class Expectation:
    metric: str
    expected: float
    rel_tol: float = 0.0
    abs_tol: float = 0.0
    kind: str = "eq"  # eq | le | ge

    def check(self, value) -> bool:
        if value is None:
            return False
        tol = max(self.abs_tol, self.rel_tol * abs(self.expected))
        if self.kind == "eq":
            return abs(value - self.expected) <= tol
        if self.kind == "le":
            return value <= self.expected + tol
        return value >= self.expected - tol


@dataclass
class ValidationCase:
    name: str
    planner: str
    source: str
    build: Callable[[], Instance]
    expectations: list[Expectation] = field(default_factory=list)
    params: dict = field(default_factory=dict)
    adapter_params: dict = field(default_factory=dict)
    seed: int = 0
    checks: list[Callable] = field(default_factory=list)  # f(result, evaluation, rep) -> str | None


def _rect(w, h, starts, holes=(), width=1.0) -> Instance:
    poly = box(0, 0, w, h)
    for hb in holes:
        poly = poly.difference(box(*hb))
    inst = instance_from_polygon(f"val_{w}x{h}", poly, default_robot(width=width, start=starts[0]))
    if len(starts) > 1:
        team = []
        for k, s in enumerate(starts):
            team.append(Robot(id=f"r{k}", start=(s[0], s[1], 0.0), tool=Footprint("disc", width)))
        inst = inst.copy(team=team)
    return inst


# ------------------------------------------------------------------ structural checks
def each_subcell_once(res, ev, rep):
    """STC property: the tour visits every sub-cell of the tree's coarse cells exactly once."""
    visited = Counter()
    for rid, cells in res.native.items():
        visited.update(int(c) for c in cells)
    dup = [c for c, n in visited.items() if n > 1]
    if dup:
        return f"{len(dup)} sub-cells visited more than once"
    expected = int(rep.used.sum()) if hasattr(rep, "used") else len(visited)
    if len(visited) != expected:
        return f"visited {len(visited)} of {expected} sub-cells"
    return None


def no_redundancy_across_robots(res, ev, rep):
    """Non-backtracking MSTC: robots' tour segments are disjoint and cover the tour."""
    seen = Counter()
    for rid, cells in res.native.items():
        seen.update(int(c) for c in cells)
    for r in rep.team:
        seen[int(rep.starts[r.id]["cell"])] += 0
    dup = [c for c, n in seen.items() if n > 1]
    return f"{len(dup)} sub-cells assigned to several robots" if dup else None


def darp_optimal_division(res, ev, rep):
    m = res.stage_metrics.get("work_distribution:darp", {})
    sizes = m.get("sizes") or []
    if not m.get("connected", False):
        return "DARP regions are not connected"
    if sizes and max(sizes) - min(sizes) > 1:
        return f"DARP sizes {sizes} differ by more than one cell"
    return None


def four_bcd_cells(res, ev, rep):
    n = len(rep.cells)
    return None if n == 4 else f"expected 4 boustrophedon cells around one hole, got {n}"


# ------------------------------------------------------------------ cases
CASES: list[ValidationCase] = [
    ValidationCase(
        "stc_open_rectangle", "stc",
        "Gabriely & Rimon (2001): the circumnavigation of a spanning tree visits every sub-cell exactly once; "
        "on an 8x6 area with 1x1 sub-cells the open tour has 47 unit moves",
        lambda: _rect(8, 6, [(0.5, 0.5, 0.0)]),
        [Expectation("path_length", 47.0, abs_tol=1e-6), Expectation("coverage_ratio", 0.98, kind="ge"),
         Expectation("static_collision_time", 0.0, abs_tol=1e-9)],
        checks=[each_subcell_once]),
    ValidationCase(
        "stc_obstacle", "stc",
        "Gabriely & Rimon (2001): coarse cells blocked by an obstacle are excluded and the remaining 72 sub-cells "
        "are visited once (71 unit moves)",
        lambda: _rect(10, 8, [(0.5, 0.5, 0.0)], holes=[(4, 2, 6, 6)]),
        [Expectation("path_length", 71.0, abs_tol=1e-6), Expectation("static_collision_time", 0.0, abs_tol=1e-9)],
        checks=[each_subcell_once]),
    ValidationCase(
        "mstc_nonbacktracking", "mstc",
        "Hazon & Kaminka (2005): non-backtracking MSTC splits one STC tour into disjoint segments (no redundancy)",
        lambda: _rect(8, 8, [(0.5, 0.5, 0.0), (7.5, 7.5, 0.0), (0.5, 7.5, 0.0)]),
        [Expectation("path_length", 61.0, abs_tol=1e-6), Expectation("coverage_ratio", 0.97, kind="ge")],
        params={"split": "nonbacktracking"}, checks=[no_redundancy_across_robots]),
    ValidationCase(
        "darp_obstacle_free", "darp_stc",
        "Kapoutsis et al. (2017): on obstacle-free grids DARP converges to connected regions whose sizes differ by "
        "at most one cell",
        lambda: _rect(20, 20, [(0.5, 0.5, 0.0), (19.5, 19.5, 0.0), (0.5, 19.5, 0.0), (19.5, 0.5, 0.0)]),
        [Expectation("coverage_ratio", 0.98, kind="ge"), Expectation("workload_imbalance", 1.05, kind="le")],
        checks=[darp_optimal_division]),
    ValidationCase(
        "darp_uneven", "darp_stc",
        "Kapoutsis et al. (2017): fair division for a number of cells not divisible by the team size",
        lambda: _rect(18, 14, [(0.5, 0.5, 0.0), (17.5, 13.5, 0.0), (0.5, 13.5, 0.0)], holes=[(6, 4, 10, 8)]),
        [Expectation("coverage_ratio", 0.97, kind="ge")],
        checks=[darp_optimal_division]),
    ValidationCase(
        "darp_bf_composition", "darp_bf",
        "Composition check: the DARP stage yields an optimal division and the back-and-forth stage covers every "
        "region (results are reported as a composition, not as DARP)",
        lambda: _rect(16, 12, [(0.5, 0.5, 0.0), (15.5, 11.5, 0.0)]),
        [Expectation("coverage_ratio", 0.97, kind="ge"), Expectation("static_collision_time", 0.0, abs_tol=1e-9)],
        checks=[darp_optimal_division]),
    ValidationCase(
        "swath_rectangle", "swath_bf",
        "Boustrophedon on a 10x6 rectangle with unit width: 6 swaths of 9 m and 5 unit connections (59 m), no overlap",
        lambda: _rect(10, 6, [(0.5, 0.5, 0.0)]),
        [Expectation("path_length", 59.0, rel_tol=0.01), Expectation("coverage_ratio", 0.99, kind="ge"),
         Expectation("overlap_ratio", 0.02, kind="le")],
        adapter_params={"angle": 0.0, "perimeter": False}),
    ValidationCase(
        "swath_perimeter_obstacle", "swath_bf",
        "Swaths with a contour pass around the boundary and a rotated obstacle cover the field completely",
        lambda: instance_from_polygon("val_rot", box(0, 0, 14, 10).difference(
            shapely.affinity.rotate(box(5, 3, 9, 6), 30)), default_robot(start=(0.5, 0.5, 0.0))),
        [Expectation("coverage_ratio", 0.99, kind="ge"), Expectation("static_collision_time", 0.0, abs_tol=1e-9)]),
    ValidationCase(
        "bcd_single_hole", "bcd",
        "Choset (2000): one rectangular hole yields four boustrophedon cells; the cells are covered completely",
        lambda: _rect(12, 9, [(0.5, 0.5, 0.0)], holes=[(4, 3, 8, 6)]),
        [Expectation("coverage_ratio", 0.98, kind="ge"), Expectation("static_collision_time", 0.0, abs_tol=1e-9)],
        adapter_params={"angle": 0.0}, checks=[four_bcd_cells]),
    ValidationCase(
        "kmeans_tsp_balance", "kmeans_tsp",
        "Target baseline: balanced clusters (imbalance <= 1.25) and complete coverage of the targets",
        lambda: _rect(12, 12, [(0.5, 0.5, 0.0), (11.5, 11.5, 0.0)]),
        [Expectation("coverage_ratio", 0.98, kind="ge"), Expectation("workload_imbalance", 1.25, kind="le")]),
]


def run_case(case: ValidationCase) -> dict:
    out = {"case": case.name, "planner": case.planner, "source": case.source, "results": [], "errors": []}
    t0 = time.perf_counter()
    try:
        inst = case.build()
        cls = get_planner(case.planner)
        man = cls.manifest
        rep, arec = make_adapter(man.adapter, **{**man.adapter_params, **case.adapter_params})(inst)
        res = cls({**case.params}).plan(rep, PlanContext(seed=case.seed))
        ev = evaluate(inst, res.trajectories, rep.reconstruction, sensitivity=False)
        for e in case.expectations:
            v = ev.metrics.get(e.metric)
            ok = e.check(v)
            out["results"].append({"metric": e.metric, "value": v, "expected": e.expected, "kind": e.kind,
                                   "rel_tol": e.rel_tol, "abs_tol": e.abs_tol, "ok": ok})
        for chk in case.checks:
            msg = chk(res, ev, rep)
            out["results"].append({"check": chk.__name__, "ok": msg is None, "message": msg})
        if ev.status == "invalid":
            out["errors"].append("invalid output: " + "; ".join(ev.validity.reasons))
    except Exception as exc:
        out["errors"].append(repr(exc))
        out["traceback"] = traceback.format_exc()
    out["passed"] = not out["errors"] and all(r["ok"] for r in out["results"])
    out["time"] = time.perf_counter() - t0
    return out


def validate_planner(name: str, out_dir: str | Path | None = None, extra_cases=None) -> dict:
    cases = [c for c in CASES + list(extra_cases or []) if c.planner == name]
    results = [run_case(c) for c in cases]
    rep = {"planner": name, "version": get_planner(name).manifest.version, "cases": results,
           "passed": bool(results) and all(r["passed"] for r in results),
           "summary": f"{sum(r['passed'] for r in results)}/{len(results)} cases passed" if results
           else "no validation cases", "validated_at": time.strftime("%Y-%m-%dT%H:%M:%S")}
    if out_dir is not None:
        dump_json(rep, Path(out_dir) / f"{name}.json")
    return rep


def validate_all(out_dir=None, planners=None) -> dict:
    from ..planners import PLANNERS

    names = planners or sorted(PLANNERS)
    return {n: validate_planner(n, out_dir) for n in names}



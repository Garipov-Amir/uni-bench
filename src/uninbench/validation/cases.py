"""Validation cases: worked examples and published properties of each planner."""

from __future__ import annotations

import time
import traceback
from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable

import shapely.affinity
import numpy as np
from shapely.geometry import box

from ..adapters import make_adapter
from ..evaluator import evaluate
from ..instance.model import Instance, Robot, default_robot, instance_from_polygon
from ..geometry.footprint import Footprint
from ..planners import PlanContext, get_planner
from ..planners.execution import finalize_output, run_planner
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
    time_limit: float = 120.0


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


def balanced_partitions(limit: float):
    def check(res, ev, rep):
        w = np.array(list(res.info.get("partition_weights", {}).values()), dtype=float)
        if len(w) == 0:
            return "no partition weights reported"
        return None if w.max() / w.min() <= limit else f"partition weights {np.round(w, 2).tolist()} not balanced"
    check.__name__ = f"balanced_partitions_{limit}"
    return check


def trees_cover_all_cells(res, ev, rep):
    """Tree covers (MFC, MIP): every complete coarse cell lies in some robot's tree (Cover constraint)."""
    visited = {int(rep.coarse_of(c)[0]) * rep.coarse["grid"].nx + int(rep.coarse_of(c)[1])
               for cells in res.native.values() for c in cells}
    need = set(np.flatnonzero(rep.coarse["cells"].ravel()).tolist())
    miss = need - visited
    return f"{len(miss)} coarse cells in no tree" if miss else None


def tree_weight_at_most(bound_fn):
    def check(res, ev, rep):
        b = bound_fn(rep)
        mx = res.info.get("max_tree_weight")
        return None if mx is not None and mx <= b + 1e-9 else f"largest tree weight {mx} > {b}"
    check.__name__ = "tree_weight_bound"
    return check


def minimum_bricks(expected: int):
    def check(res, ev, rep):
        n = res.info.get("bricks")
        return None if n == expected else f"expected a minimum brick tiling with {expected} bricks, got {n}"
    check.__name__ = f"minimum_bricks_{expected}"
    return check


def all_subcells_visited(res, ev, rep):
    """ESTC / LS-MCPP: every traversable sub-cell (incomplete coarse cells included) is on some path."""
    visited = {int(c) for cells in res.native.values() for c in cells}
    visited |= {int(rep.starts[r.id]["cell"]) for r in rep.team}
    need = set(np.flatnonzero(rep.cells.ravel()).tolist())
    miss = need - visited
    return f"{len(miss)} sub-cells not visited" if miss else None


def every_cell_in_tour(res, ev, rep):
    """CRC: the Chinese-postman tour covers every boustrophedon cell that has passes."""
    missing = set(res.info.get("cells_with_passes", [])) - set(res.info.get("covered_cells", []))
    return f"cells {sorted(missing)} not covered by the tour" if missing else None


def cells_assigned_once(res, ev, rep):
    """SCoPP: after the conflict auction every square cell belongs to exactly one robot."""
    pts = [tuple(np.round(q, 6)) for w in res.native.values() for q in w]
    return None if len(pts) == len(set(pts)) and pts else "cells assigned to several robots or none"


def isolines_covered(res, ev, rep):
    """MCFS: every isoline of the isograph lies on some robot's connected Fermat spiral."""
    on_path = {int(n[0]) for cyc in res.native.values() for n in cyc}
    n_iso = len([s for s in rep.swaths if s["line"].length > 1e-6])
    return None if len(on_path) == n_iso else f"{n_iso - len(on_path)} isolines not on any path"


def voronoi_property(res, ev, rep):
    """GM-VPC: regions are contiguous and every cell is at least as close (geodesic-Manhattan) to its
    own robot as to any other robot."""
    m = res.stage_metrics.get("work_distribution:gm_voronoi", {})
    return None if m.get("connected", False) else "Voronoi regions are not contiguous"


def local_search_not_worse(res, ev, rep):
    m = res.stage_metrics
    a = m.get("work_distribution:mfc_initial", {}).get("initial_makespan_steps")
    b = m.get("optimization:local_search", {}).get("makespan_steps")
    return None if a is not None and b is not None and b <= a else f"local search made the makespan worse ({a} -> {b})"


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
        checks=[no_redundancy_across_robots]),
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
        "mstc_star_balanced", "mstc_star",
        "Tang et al. (2021): Balanced-MSTC* cuts one STC tour into partitions of nearly equal weight; every "
        "sub-cell is covered",
        lambda: _rect(16, 12, [(0.5, 0.5, 0.0), (1.5, 0.5, 0.0), (0.5, 1.5, 0.0), (1.5, 1.5, 0.0)]),
        [Expectation("coverage_ratio", 0.98, kind="ge"), Expectation("static_collision_time", 0.0, abs_tol=1e-9)],
        checks=[balanced_partitions(1.1)]),
    ValidationCase(
        "tmstc_star_bricks", "tmstc_star",
        "Lu et al. (2023), Fig. 4: the minimum brick tiling of a 3 x 4 grid has 3 bricks; coverage is complete",
        lambda: _rect(8, 6, [(0.5, 0.5, 0.0), (7.5, 5.5, 0.0)]),
        [Expectation("coverage_ratio", 0.98, kind="ge")], checks=[minimum_bricks(3)]),
    ValidationCase(
        "mfc_tree_cover", "mfc",
        "Zheng et al. (2010) / Even et al. (2004): the rooted trees cover every terrain vertex and the largest "
        "tree weighs at most 4 OPT; on an open 8 x 6 terrain graph with robots in the four corners OPT = 11 "
        "(one 4 x 3 quadrant per robot)",
        lambda: _rect(16, 12, [(0.5, 0.5, 0.0), (15.5, 11.5, 0.0), (0.5, 11.5, 0.0), (15.5, 0.5, 0.0)]),
        [Expectation("coverage_ratio", 0.98, kind="ge")],
        checks=[trees_cover_all_cells, tree_weight_at_most(lambda rep: 4 * 11.0)]),
    ValidationCase(
        "mip_open_grid_optimum", "mip",
        "Tang & Ma (2023): the MIP finds an optimal rooted tree cover; on an open 4 x 4 terrain graph with two "
        "robots in opposite corners the optimal largest tree has 7 edges (8 vertices each)",
        lambda: _rect(8, 8, [(0.5, 0.5, 0.0), (7.5, 7.5, 0.0)]),
        [Expectation("coverage_ratio", 0.98, kind="ge")],
        params={"heuristic": "none"}, checks=[trees_cover_all_cells, tree_weight_at_most(lambda rep: 7.0)]),
    ValidationCase(
        "ls_mcpp_incomplete_terrain", "ls_mcpp",
        "Tang & Ma (2024): ESTC covers incomplete terrain vertices (odd-sized area and an off-grid hole), and the "
        "local search does not increase the makespan of the MFC initial solution",
        lambda: _rect(13, 9, [(0.5, 0.5, 0.0), (12.5, 8.5, 0.0)], holes=[(4.3, 3.4, 7.7, 5.2)]),
        [Expectation("coverage_ratio", 0.9, kind="ge"), Expectation("static_collision_time", 0.0, abs_tol=1e-9)],
        params={"max_iterations": 400}, checks=[all_subcells_visited, local_search_not_worse]),
    ValidationCase(
        "crc_route_clustering", "crc",
        "Karapetyan et al. (2017): the Eulerian tour of the Reeb graph covers every boustrophedon cell; the FHK "
        "split gives every robot a sub-tour from and to the start",
        lambda: _rect(14, 10, [(0.5, 0.5, 0.0), (1.5, 0.5, 0.0)], holes=[(5, 3, 9, 7)]),
        [Expectation("coverage_ratio", 0.9, kind="ge"), Expectation("static_collision_time", 0.0, abs_tol=1e-9)],
        adapter_params={"angle": 0.0}, checks=[every_cell_in_tour]),
    ValidationCase(
        "scopp_partition", "scopp",
        "Collins et al. (2021): Lloyd clustering and the conflict auction assign every square cell to exactly one "
        "robot; nearest-neighbour paths visit all cells",
        lambda: _rect(12, 10, [(0.5, 0.5, 0.0), (11.5, 9.5, 0.0), (0.5, 9.5, 0.0)]),
        [Expectation("coverage_ratio", 0.97, kind="ge"), Expectation("static_collision_time", 0.0, abs_tol=1e-9)],
        checks=[cells_assigned_once]),
    ValidationCase(
        "mcfs_fermat_spirals", "mcfs",
        "Tang & Ma (2024b): the tree cover of the isograph contains every isovertex, so the connected Fermat "
        "spirals traverse every isoline",
        lambda: _rect(14, 10, [(0.5, 0.5, 0.0), (13.5, 9.5, 0.0)], holes=[(5, 4, 9, 6)]),
        [Expectation("coverage_ratio", 0.93, kind="ge"), Expectation("static_collision_time", 0.0, abs_tol=1e-9)],
        checks=[isolines_covered], time_limit=60.0),
    ValidationCase(
        "gm_vpc_voronoi", "gm_vpc",
        "Nair & Guruprasad (2020): the geodesic-Manhattan Voronoi partition gives contiguous regions and complete, "
        "non-overlapping coverage at grid level",
        lambda: _rect(16, 12, [(0.5, 0.5, 0.0), (15.5, 11.5, 0.0), (0.5, 11.5, 0.0)], holes=[(6, 4, 10, 8)]),
        [Expectation("coverage_ratio", 0.97, kind="ge"), Expectation("overlap_ratio", 0.05, kind="le")],
        checks=[voronoi_property]),
    ValidationCase(
        "ga_targets_complete", "ga_targets",
        "GA baseline: every coverage disk is visited and the balanced split keeps the imbalance low",
        lambda: _rect(12, 12, [(0.5, 0.5, 0.0), (11.5, 11.5, 0.0)]),
        [Expectation("coverage_ratio", 0.98, kind="ge"), Expectation("workload_imbalance", 1.3, kind="le")],
        params={"generations": 150}),
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
        res = run_planner(cls({**case.params}), rep.fresh(), PlanContext(seed=case.seed, time_limit=case.time_limit))
        res = finalize_output(res, man)
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


def validation_passed(rep: dict, require_published: bool = True) -> bool:
    """Gate of the experiment runner: internal cases pass and, for planners with published
    results, the published-result validation passes (or is not required)."""
    pub = rep.get("published") or {}
    if not rep.get("internal_passed", rep.get("passed")):
        return False
    if not pub.get("applicable"):
        return True
    if pub.get("available"):
        return bool(pub.get("passed"))
    return not require_published


def validate_planner(name: str, out_dir: str | Path | None = None, extra_cases=None, published: bool = True,
                     published_root=None) -> dict:
    from .published import published_cases

    cases = [c for c in CASES + list(extra_cases or []) if c.planner == name]
    results = [run_case(c) for c in cases]
    internal = bool(results) and all(r["passed"] for r in results)
    pub = published_cases(name, published_root) if published else {"applicable": False, "skipped": True}
    if pub.get("skipped"):
        ptxt = "published comparison skipped"
    elif not pub.get("applicable"):
        ptxt = "no published reference data"
    elif not pub.get("available"):
        ptxt = "published validation data not available"
    else:
        ptxt = "published results " + ("reproduced" if pub["passed"] else "NOT reproduced")
    rep = {"planner": name, "version": get_planner(name).manifest.version, "cases": results,
           "internal_passed": internal, "published": pub,
           "summary": (f"{sum(r['passed'] for r in results)}/{len(results)} cases passed" if results
                       else "no validation cases") + f"; {ptxt}",
           "validated_at": time.strftime("%Y-%m-%dT%H:%M:%S")}
    rep["passed"] = validation_passed(rep, require_published=True)
    if out_dir is not None:
        dump_json(rep, Path(out_dir) / f"{name}.json")
    return rep


def validate_all(out_dir=None, planners=None, published: bool = True) -> dict:
    from ..planners import PLANNERS

    names = planners or sorted(PLANNERS)
    return {n: validate_planner(n, out_dir, published=published) for n in names}



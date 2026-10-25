"""Planners of Figure 4.1 added to the benchmark: structural properties and smoke runs."""

import numpy as np
import pytest
from shapely.geometry import box

from uninbench.adapters import make_adapter
from uninbench.datasets.safelog import import_safelog
from uninbench.evaluator import evaluate
from uninbench.instance.model import default_robot, instance_from_polygon
from uninbench.instance.team import with_team
from uninbench.planners import PLANNERS, PlanContext, get_planner
from uninbench.planners.execution import finalize_output, run_planner
from uninbench.planners.karapetyan import fhk_split
from uninbench.planners.ls_mcpp import Decomposed, estc
from uninbench.planners.mfc import rtc_min_max
from uninbench.planners.mip import RMMTCModel
from uninbench.planners.tmstc_star import minimum_brick_tiling
from uninbench.planners.tree_cover import coarse_graph

NEW = ["mstc_star", "tmstc_star", "mfc", "mip", "ls_mcpp", "gm_vpc", "crc", "scopp", "mcfs", "ga_targets"]


def _inst(poly, m=2):
    base = instance_from_polygon("t", poly, default_robot(start=(0.5, 0.5, 0.0)))
    return with_team(base, m, "dispersed") if m > 1 else base


@pytest.mark.parametrize("name", NEW)
def test_new_planner_produces_valid_plans(name):
    assert name in PLANNERS
    inst = _inst(box(0, 0, 14, 10).difference(box(5, 4, 9, 6)), m=2)
    man = get_planner(name).manifest
    rep, _ = make_adapter(man.adapter, **man.adapter_params)(inst)
    params = {"max_iterations": 200} if name == "ls_mcpp" else {"generations": 60} if name == "ga_targets" else {}
    res = finalize_output(run_planner(get_planner(name)(params), rep.fresh(), PlanContext(seed=1, time_limit=30)),
                          man)
    ev = evaluate(inst, res.trajectories, rep.reconstruction, sensitivity=False)
    assert ev.validity.valid, ev.validity.reasons
    assert ev.metrics["coverage_ratio"] > 0.7


def test_estc_equals_stc_on_complete_terrain_and_covers_incomplete_terrain():
    for poly, exact in ((box(0, 0, 8, 6), True), (box(0, 0, 9, 7), False)):
        inst = instance_from_polygon("t", poly, default_robot(start=(0.5, 0.5, 0.0)))
        rep, _ = make_adapter("grid", coarse="partial", connectivity=4)(inst)
        D = Decomposed(rep)
        walk, steps = estc(D, set(D.V), D.starts[0])
        assert set(walk) == set(D.V) and walk[0] == walk[-1]
        assert all(walk[t + 1] in D.nbr[walk[t]] for t in range(len(walk) - 1))
        if exact:
            assert steps == len(D.V)  # Hamiltonian cycle: every sub-cell exactly once


def test_rooted_tree_cover_and_mip_start_vector_are_feasible():
    inst = _inst(box(0, 0, 12, 8), m=3)
    rep, _ = make_adapter("grid", coarse=True)(inst)
    cg = coarse_graph(rep)
    trees, mx, _ = rtc_min_max(cg.n, cg.edges, cg.w_edge, cg.roots)
    covered = set().union(*[t[0] for t in trees])
    assert covered == set(range(cg.n))
    model = RMMTCModel(cg.n, cg.edges, cg.w_edge, cg.roots)
    x0 = model.start_vector(trees)
    _, cons, _, bnds = model.build()
    ax = cons.A @ x0
    assert np.all(ax >= cons.lb - 1e-9) and np.all(ax <= cons.ub + 1e-9)
    assert np.all(x0 >= bnds.lb - 1e-9) and np.all(x0 <= bnds.ub + 1e-9)


def test_minimum_brick_tiling_of_the_papers_example():
    inst = instance_from_polygon("t", box(0, 0, 8, 6), default_robot(start=(0.5, 0.5, 0.0)))
    rep, _ = make_adapter("grid", coarse=True)(inst)
    cg = coarse_graph(rep)  # 4 x 3 terrain graph
    removed = minimum_brick_tiling(cg, rep.coarse["grid"].nx)
    assert cg.n - len(removed) == 3


def test_fhk_split_partitions_the_tour():
    tour = [(e, e, e + 1, True) for e in range(10)]
    parts = fhk_split(tour, lambda t: 1.0, lambda v: 0.0, 3)
    assert sum(parts, []) == list(range(10)) and all(parts)


def test_published_safelog_format(tmp_path):
    d = tmp_path / "benchmark-planning"
    d.mkdir()
    (d / "test_nodes.txt").write_text("\n".join(f"{x * 3 + y} {x} {y}" for x in range(3) for y in range(3)))
    lat = [(0, 1), (1, 2), (0, 3), (3, 6), (6, 7), (7, 8), (4, 5)]  # ids are x * 3 + y (column-major)
    (d / "test_edges0.txt").write_text("\n".join(f"{a} {b}\n{b} {a}" for a, b in lat))
    (d / "test_edges.txt").write_text("0 1\n")
    insts = import_safelog(tmp_path)
    assert [i.id for i in insts] == ["safelog_test_edges0"]
    g = insts[0].metadata["graph"]
    assert g["rows"] == 3 and g["cols"] == 3 and len(g["edges"]) == len(lat)

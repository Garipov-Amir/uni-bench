import numpy as np
import pytest
from shapely.geometry import LineString, box

from uninbench.adapters import make_adapter
from uninbench.datasets.safelog import graph_to_instance, lattice_edges
from uninbench.evaluator import evaluate
from uninbench.instance.team import with_team
from uninbench.planners import PLANNERS, PlanContext, StagedPlanner, admission, get_planner
from uninbench.planners.base import Stage
from uninbench.profiles import apply_profile
from uninbench.validation import CASES, run_case


def test_geometric_adapter_is_faithful(room_with_hole):
    rep, rec = make_adapter("geometric")(room_with_hole)
    assert rec.fidelity["rel_error"] < 1e-3 and not rec.flagged
    assert rec.params["angle"] is not None and rec.runtime >= 0


def test_grid_adapter_flags_conservative_loss(room_with_hole):
    rep, rec = make_adapter("grid", coarse=True, resolution_factor=1.5)(room_with_hole)
    assert rec.fidelity["missing_area"] > 0
    assert rec.params["resolution"] == pytest.approx(1.5)
    # every graph edge keeps the robot body inside the free space
    cs = room_with_hole.team[0].body_fp.erode(room_with_hole.free).buffer(1e-6)
    C = rep.center(rep.edges.ravel()).reshape(-1, 2, 2)
    assert all(cs.covers(LineString(e)) for e in C)


def test_grid_edges_respect_safelog_walls():
    edges = [e for e in lattice_edges(3, 3) if e != (0, 1)]  # wall between vertex 0 and 1
    inst = graph_to_instance("g", 3, 3, edges, cell_size=2.0)
    rep, _ = make_adapter("grid", coarse=True)(inst)
    cedges = {tuple(sorted(e)) for e in rep.coarse["edges"].tolist()}
    assert (0, 1) not in cedges and (0, 3) in cedges
    rep2, rec2 = make_adapter("graph", coarse=True)(inst)
    assert {tuple(sorted(e)) for e in rep2.coarse["edges"].tolist()} == cedges
    res = get_planner("stc")().plan(rep, PlanContext())
    ev = evaluate(inst, res.trajectories, rep.reconstruction, sensitivity=False)
    assert ev.status == "success"


@pytest.mark.parametrize("case", CASES, ids=[c.name for c in CASES])
def test_validation_cases(case):
    r = run_case(case)
    assert r["passed"], r


def test_admission_rules(rect_instance):
    stc = PLANNERS["stc"].manifest
    assert admission(stc, rect_instance)[0] == "admitted"
    assert admission(stc, with_team(rect_instance, 2, "dispersed"))[0] == "not_applicable"
    agri = apply_profile(rect_instance, "agricultural")
    st, why = admission(PLANNERS["darp_stc"].manifest, agri)
    assert st == "not_applicable" and "turning_radius" in why
    assert admission(PLANNERS["swath_bf"].manifest, agri)[0] == "admitted"
    other = rect_instance.copy()
    other.task.task_class = "persistent_coverage"
    assert admission(stc, other)[0] == "excluded"


def test_staged_composition_is_type_checked():
    class Bad(StagedPlanner):
        manifest = PLANNERS["mstc"].manifest
        stages = [type("S", (Stage,), {"kind": "route_construction", "name": "x", "consumes": ("tour",),
                                       "produces": "routes"})()]

    with pytest.raises(TypeError):
        Bad.check_composition()


def test_deterministic_and_stochastic_planners(room_with_hole):
    inst = with_team(room_with_hole, 3, "dispersed")
    for name in ("mstc", "swath_bf", "bcd"):
        man = PLANNERS[name].manifest
        rep, _ = make_adapter(man.adapter, **man.adapter_params)(inst)
        a = PLANNERS[name]().plan(rep, PlanContext(seed=1)).trajectories.to_dict()
        b = PLANNERS[name]().plan(rep, PlanContext(seed=2)).trajectories.to_dict()
        assert a == b
    man = PLANNERS["kmeans_tsp"].manifest
    rep, _ = make_adapter(man.adapter, **man.adapter_params)(inst)
    a = PLANNERS["kmeans_tsp"]().plan(rep, PlanContext(seed=1)).trajectories.to_dict()
    a2 = PLANNERS["kmeans_tsp"]().plan(rep, PlanContext(seed=1)).trajectories.to_dict()
    b = PLANNERS["kmeans_tsp"]().plan(rep, PlanContext(seed=5)).trajectories.to_dict()
    assert a == a2 and a != b


def test_agricultural_plan_is_feasible():
    from uninbench.instance.model import default_robot, instance_from_polygon

    field = instance_from_polygon("field", box(0, 0, 80, 50), default_robot(width=3.0, start=(10, 10, 0)))
    inst = apply_profile(with_team(field, 2, "dispersed"), "agricultural", {"capacity": 0.3})
    from uninbench.profiles import profile_adapter_params

    man = PLANNERS["swath_bf"].manifest
    rep, _ = make_adapter("swath", **{**man.adapter_params, **profile_adapter_params(inst, "swath")})(inst)
    res = PLANNERS["swath_bf"]().plan(rep, PlanContext(profiles={"agricultural"}, return_to_depot=True))
    ev = evaluate(inst, res.trajectories, rep.reconstruction, sensitivity=False)
    assert ev.status in ("success", "incomplete"), ev.validity.reasons
    assert ev.metrics["coverage_ratio"] > 0.9
    assert ev.metrics["capacity_excess"] <= 1e-6
    assert res.info["dubins_fallbacks"] == 0
    _ = np

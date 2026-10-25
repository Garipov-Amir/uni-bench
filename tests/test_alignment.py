"""Tests of the design requirements that the evaluation and runner fixes address."""

import math

import numpy as np
import pytest
from shapely.geometry import box

from uninbench.adapters import make_adapter
from uninbench.evaluator import evaluate
from uninbench.instance.model import default_robot, instance_from_polygon
from uninbench.instance.splits import assign_splits
from uninbench.instance.team import with_team
from uninbench.planners import PlanContext, admission, get_planner
from uninbench.planners.execution import conservative_labels, framework_stages, run_planner
from uninbench.profiles import apply_profile
from uninbench.reporting.aggregate import success_table, variant
from uninbench.reporting.stats import pairwise_wilcoxon, wilcoxon_paired
from uninbench.runner.config import load_config
from uninbench.runner.experiment import check_resolutions, derive_instance
from uninbench.runner.worker import failure_attribution
from uninbench.trajectory import SERVICE, Trajectory, TrajectorySet, from_labelled_polyline, headings_from_xy
from uninbench.validation import validation_passed


# ------------------------------------------------------------------ planning time
def test_every_planner_call_gets_a_cold_representation(room_with_hole):
    rep, _ = make_adapter("swath", mode="parallel")(room_with_hole)
    a = rep.fresh()
    a.transit(a.team[0])  # a planner builds its transit graph lazily ...
    assert any(k[0] == "transit" for k in a._cache)
    b = rep.fresh()  # ... the next call must build its own
    assert not any(k[0] == "transit" for k in b._cache)
    assert ("cspace", rep.team[0].body_fp) in b._cache  # adapter output is kept


def test_grid_shortest_path_cache_is_per_call(rect_instance):
    rep, _ = make_adapter("grid", coarse=True)(rect_instance)
    a = rep.fresh()
    a.distances(0)
    assert a._sp_cache and not rep.fresh()._sp_cache


# ------------------------------------------------------------------ trajectory normal form
def test_heading_changes_cost_turn_time_without_turn_segments(rect_instance):
    r = rect_instance.team[0]
    pts = np.array([[0.5, 0.5], [9.5, 0.5], [9.5, 1.5], [0.5, 1.5]])
    with_turns = TrajectorySet()
    with_turns.add(from_labelled_polyline(r.id, pts, [SERVICE] * 3, start_psi=0.0))
    K = len(pts) - 1
    bare = TrajectorySet()
    bare.add(Trajectory(r.id, pts, headings_from_xy(pts), [SERVICE] * K, [True] * K, np.zeros(K)))
    a = evaluate(rect_instance, with_turns, sensitivity=False)
    b = evaluate(rect_instance, bare, sensitivity=False)
    assert b.metrics["makespan"] == pytest.approx(a.metrics["makespan"])
    assert b.metrics["turns"] == a.metrics["turns"] == 2
    assert any("explicit turns" in w for w in b.validity.warnings)


def test_turn_from_the_declared_start_heading_is_charged(rect_instance):
    r = rect_instance.team[0]  # starts facing +x
    ts = TrajectorySet()
    ts.add(from_labelled_polyline(r.id, np.array([[0.5, 0.5], [0.5, 3.5]]), [SERVICE], start_psi=math.pi / 2))
    ev = evaluate(rect_instance, ts, sensitivity=False)
    assert ev.metrics["makespan"] == pytest.approx(3.0 + math.pi / 2 * r.turn_cost)


def test_conservative_labels_for_planners_without_transit():
    ts = TrajectorySet()
    ts.add(from_labelled_polyline("r0", np.array([[0, 0], [1, 0], [1, 1]]), ["service", "transit"]))
    out = conservative_labels(ts)
    tr = out["r0"]
    assert "transit" not in set(tr.labels) and tr.active[tr.labels == "service"].all()


# ------------------------------------------------------------------ statistics
def test_wilcoxon_keeps_failed_runs():
    b = np.linspace(1.2, 2.0, 12)
    a = b * 0.97
    a[6:] = math.inf  # A is slightly better where it succeeds but fails on half of the instances
    r = wilcoxon_paired(a, b)
    assert r["n"] == 12 and r["failures_a"] == 6
    assert r["rank_biserial"] > 0  # A is not better than B
    both = wilcoxon_paired(np.array([math.inf, 1.0]), np.array([math.inf, 2.0]))
    assert both["n"] == 2  # a failure of both is a tie
    rows = pairwise_wilcoxon(np.c_[a, b], ["A", "B"])
    assert rows[0]["failures_a"] == 6


# ------------------------------------------------------------------ attribution and provenance
def test_failures_on_flagged_representations_are_attributed_to_the_adapter():
    assert failure_attribution({"outcome": "incomplete", "adapter_flagged": True}) == "adapter"
    assert failure_attribution({"outcome": "timeout", "adapter_flagged": False}) == "planner"
    assert failure_attribution({"outcome": "crash", "adapter_error": True}) == "adapter"
    assert failure_attribution({"outcome": "success", "adapter_flagged": True}) is None
    base = {"family": "F", "team_size": 1, "config_label": "default", "planner": "p"}
    recs = [dict(base, outcome="success"), dict(base, outcome="incomplete", failure_attribution="adapter"),
            dict(base, outcome="incomplete", failure_attribution="planner")]
    row = success_table(recs)[0]
    assert row["success_rate"] == pytest.approx(0.5) and row["raw_success_rate"] == pytest.approx(1 / 3)
    assert row["adapter_failures"] == 1


def test_non_original_results_are_marked():
    assert variant({"planner": "mstc", "provenance": "original"}) == "mstc"
    assert variant({"planner": "bcd", "provenance": "composition"}) == "bcd*"
    assert variant({"planner": "kmeans_tsp", "provenance": "baseline"}) == "kmeans_tsp*"
    assert variant({"planner": "mstc", "provenance": "composition", "framework_stages": ["prioritized_waiting"]}) \
        == "mstc+pw*"
    for name in ("bcd", "swath_bf", "darp_bf", "kmeans_tsp", "ga_targets"):
        assert get_planner(name).manifest.provenance != "original"
    assert "split" not in get_planner("mstc").manifest.defaults()  # no non-original variants to tune
    assert "metric" not in get_planner("darp_stc").manifest.defaults()


def test_framework_deconfliction_is_a_declared_composition(room_with_hole):
    inst = apply_profile(with_team(room_with_hole, 2, "dispersed"), "deconfliction")
    man = get_planner("mstc").manifest
    assert admission(man, inst)[0] == "not_applicable"
    assert admission(man, inst, "prioritized_waiting")[0] == "admitted"
    assert framework_stages(man, inst, "prioritized_waiting") == ["prioritized_waiting"]
    one = apply_profile(room_with_hole, "deconfliction")
    assert admission(get_planner("stc").manifest, one)[0] == "admitted"  # one robot cannot conflict
    rep, _ = make_adapter(man.adapter, **man.adapter_params)(inst)
    res = run_planner(get_planner("mstc")(), rep.fresh(), PlanContext(profiles={"deconfliction"}),
                      ["prioritized_waiting"])
    assert "deconfliction" in res.info


# ------------------------------------------------------------------ instances and resolutions
def test_numeric_factor_is_stratified_over_its_whole_range():
    insts = []
    for k in range(21):
        inst = instance_from_polygon(f"g{k}", box(0, 0, 4, 4), default_robot(start=(0.5, 0.5, 0.0)), family="IF4")
        inst.metadata["factors"] = {"graph_density": 0.5 + k * 0.025}
        insts.append(inst)
    s = assign_splits(insts)
    dens = sorted(i.metadata["factors"]["graph_density"] for i in insts if s[i.id] == "tuning")
    assert len(dens) == 4 and dens[-1] - dens[0] > 0.25


def test_evaluation_resolution_must_be_finer_than_planning_resolution(rect_instance):
    check_resolutions(rect_instance, "grid", {"resolution_factor": 1.0, "coarse": True})
    with pytest.raises(ValueError):
        check_resolutions(rect_instance, "grid", {"resolution_factor": 0.1})


def test_active_metric_set(rect_instance):
    inst = rect_instance.copy()
    inst.evaluation.metrics = ["makespan"]
    ts = TrajectorySet()
    ts.add(from_labelled_polyline("r0", np.array([[0.5, 0.5], [9.5, 0.5]]), [SERVICE], start_psi=0.0))
    ev = evaluate(inst, ts, sensitivity=False)
    assert "makespan" in ev.metrics and "coverage_ratio" in ev.metrics and "overlap_ratio" not in ev.metrics


def test_sensitivity_check_covers_the_uncovered_area_split(rect_instance):
    ts = TrajectorySet()
    ts.add(from_labelled_polyline("r0", np.array([[0.5, 0.5], [9.5, 0.5]]), [SERVICE], start_psi=0.0))
    ev = evaluate(rect_instance, ts, sensitivity=True)
    assert ev.sensitivity["checks"] and "metric" in ev.sensitivity["checks"][0]


def test_validation_gate():
    rep = {"internal_passed": True, "published": {"applicable": True, "available": False}}
    assert not validation_passed(rep, require_published=True)
    assert validation_passed(rep, require_published=False)
    assert validation_passed({"internal_passed": True, "published": {"applicable": False}})
    assert not validation_passed({"internal_passed": False, "published": {"applicable": False}})


def test_defaults_follow_the_design(tmp_path):
    cfg = load_config({"name": "x", "instances": ["a"], "planners": ["stc"]})
    assert cfg.require_validation and cfg.raw["require_published_validation"]
    assert cfg.raw["max_tasks_per_worker"] == 1 and cfg.raw["deconfliction_stage"] is None

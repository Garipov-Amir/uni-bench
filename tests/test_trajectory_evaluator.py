import math

import numpy as np
import pytest
from shapely.geometry import box

from uninbench.evaluator import evaluate
from uninbench.evaluator.coverage import sweep_counts
from uninbench.geometry.dubins import dubins_paths
from uninbench.geometry.footprint import Footprint
from uninbench.geometry.raster import RasterGrid
from uninbench.instance.model import Robot, default_robot, instance_from_polygon
from uninbench.reconstruction import GeometryRegion
from uninbench.trajectory import (SERVICE, TRANSIT, TURN, Trajectory, TrajectoryBuilder, TrajectorySet, retime,
                                  turning_radius_ok)


def _one(traj):
    ts = TrajectorySet()
    ts.add(traj)
    return ts


def test_builder_inserts_turns_and_retime():
    b = TrajectoryBuilder("r0", 0, 0, 0)
    b.move_to(2, 0, SERVICE).move_to(2, 3, TRANSIT).wait_for(1.5)
    tr = retime(b.build(), speed=2.0, turn_cost=0.5)
    assert list(tr.labels) == [SERVICE, TURN, TRANSIT, "wait"]
    assert tr.t[-1] == pytest.approx(2 / 2 + (math.pi / 2) * 0.5 + 3 / 2 + 1.5)
    back = Trajectory.from_dict(tr.to_dict())
    assert np.allclose(back.xy, tr.xy) and list(back.labels) == list(tr.labels)


def test_straight_pass_area_and_single_count():
    grid = RasterGrid.covering((-2, -2, 12, 2), 0.02)
    b = TrajectoryBuilder("r0", 0, 0, 0)
    for x in np.linspace(0.5, 10, 20):  # many waypoints on one continuous pass
        b.move_to(x, 0, SERVICE)
    tr = retime(b.build(), 1.0, 0.0)
    n, _ = sweep_counts(grid, tr, Footprint("disc", 1.0))
    assert n.max() == 1
    assert (n >= 1).sum() * grid.cell_area == pytest.approx(10 * 1.0 + math.pi * 0.25, rel=5e-3)


def test_return_is_counted_again():
    grid = RasterGrid.covering((-1, -1, 6, 1), 0.05)
    b = TrajectoryBuilder("r0", 0, 0, 0)
    b.move_to(5, 0, SERVICE).move_to(0, 0, SERVICE)
    n, _ = sweep_counts(grid, retime(b.build(), 1.0, 0.0), Footprint("disc", 1.0))
    assert n.max() == 2  # an inactive in-place turn separates the two sweeps
    b = TrajectoryBuilder("r0", 0, 0, 0, tool_active_in_turns=True)
    b.move_to(5, 0, SERVICE).move_to(0, 0, SERVICE)
    n2, _ = sweep_counts(grid, retime(b.build(), 1.0, 0.0), Footprint("disc", 1.0))
    # with the tool active through the turn, cells near the turning point are still swept twice
    # (they leave the footprint and come back) but the turning point itself is swept once
    assert n2.max() == 2 and n2.sum() <= n.sum()


def test_evaluate_boustrophedon(rect_instance):
    b = TrajectoryBuilder("r0", 0.5, 0.5, 0, tool_active_in_turns=True)
    for k, y in enumerate([0.5, 1.5, 2.5, 3.5]):
        xs = (0.5, 9.5) if k % 2 == 0 else (9.5, 0.5)
        b.move_to(xs[0], y).move_to(xs[1], y)
    ev = evaluate(rect_instance, _one(b.build()), GeometryRegion(rect_instance.free))
    m = ev.metrics
    assert ev.status == "success"
    assert m["coverage_ratio"] > 0.99
    assert m["path_length"] == pytest.approx(39.0)
    assert m["makespan"] == pytest.approx(39.0 + 3 * math.pi * 0.25)
    assert m["turns"] == 6
    assert m["uncovered_representation"] == pytest.approx(0.0, abs=1e-9)
    assert m["normalized_makespan"] == pytest.approx(m["makespan"] / m["T_LB"])
    assert ev.sensitivity["enabled"] and ev.sensitivity["stable"]


def test_invalid_start_and_collision(rect_instance):
    b = TrajectoryBuilder("r0", 3, 3, 0)
    b.move_to(4, 3)
    ev = evaluate(rect_instance, _one(b.build()))
    assert ev.status == "invalid" and "start" in ev.validity.reasons[0]
    b = TrajectoryBuilder("r0", 0.5, 0.5, 0)
    b.move_to(0.5, 3.9)  # the disc body leaves the workspace
    ev = evaluate(rect_instance, _one(b.build()))
    assert ev.status == "invalid" and "collision" in ev.validity.reasons[0]


def test_conflicts_reported_and_enforced():
    inst = instance_from_polygon("c", box(0, 0, 10, 3), default_robot(start=(0.5, 1.5, 0)))
    inst = inst.copy(team=[Robot("a", (0.5, 1.5, 0), Footprint("disc", 1)), Robot("b", (9.5, 1.5, math.pi),
                                                                                    Footprint("disc", 1))])
    ta = TrajectoryBuilder("a", 0.5, 1.5, 0).move_to(9.5, 1.5).build()
    tb = TrajectoryBuilder("b", 9.5, 1.5, math.pi).move_to(0.5, 1.5).build()
    ts = TrajectorySet({"a": ta, "b": tb})
    ev = evaluate(inst, ts, sensitivity=False)
    assert ev.metrics["conflict_instants"] > 0 and ev.status != "invalid"
    ev = evaluate(inst, ts, sensitivity=False, profiles=["deconfliction"])
    assert ev.status == "invalid"


def test_turning_radius_fillet_criterion():
    b = TrajectoryBuilder("r", 0, 0, 0)
    b.follow_curve(dubins_paths((0, 0, 0), (0, 20, math.pi), 6.0, 0.3)[0][2])
    ok, rad = turning_radius_ok(b.build(), 6.0)
    assert ok and rad == pytest.approx(6.0, rel=0.02)
    assert not turning_radius_ok(b.build(), 7.0)[0]
    b = TrajectoryBuilder("r", 0, 0, 0)
    b.move_to(10, 0).move_to(10, 10)
    assert turning_radius_ok(b.build(), 5.0)[0]  # a radius-5 fillet fits into 10 m legs
    b = TrajectoryBuilder("r", 0, 0, 0)
    b.move_to(1, 0).move_to(1, 1)
    assert not turning_radius_ok(b.build(), 5.0)[0]

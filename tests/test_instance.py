import math

import numpy as np
import pytest
from shapely.geometry import Point, box

from uninbench.datasets.synthetic import generate, regenerate
from uninbench.instance.coverable import makespan_lower_bound
from uninbench.instance.model import Instance
from uninbench.instance.splits import assign_splits
from uninbench.instance.team import with_team


def test_roundtrip_and_hash(rect_instance, tmp_path):
    p = rect_instance.save(tmp_path)
    back = Instance.load(p)
    assert back.hash == rect_instance.hash
    assert back.free.equals(rect_instance.free)
    other = rect_instance.copy()
    other.team[0].speed = 2.0
    assert other.hash != rect_instance.hash


def test_coverable_set_excludes_corners_and_pockets():
    # a pocket connected through a gap narrower than the footprint is excluded
    poly = box(0, 0, 10, 6).union(box(10, 2.8, 10.5, 3.2)).union(box(10.5, 1, 14, 5))
    inst = Instance.from_dict({**_inst_dict(poly), "id": "pocket"})
    d = inst.derived
    assert not d.coverable.intersects(Point(12, 3))
    corners = 4 * (0.25 - math.pi * 0.25 / 4)  # square corners a disc tool cannot reach
    assert d.coverable_area == pytest.approx(60 - corners, rel=1e-3)


def _inst_dict(poly):
    from uninbench.instance.model import default_robot, instance_from_polygon

    return instance_from_polygon("x", poly, default_robot(start=(0.5, 0.5, 0))).to_dict(include_hash=False)


def test_makespan_lower_bound(rect_instance):
    area = rect_instance.derived.coverable_area
    lb = makespan_lower_bound(area, rect_instance.team)
    assert lb == pytest.approx((area - math.pi * 0.25) / (2 * 0.5 * 1.0))
    four = with_team(rect_instance, 4, "dispersed")
    assert four.derived.T_LB == pytest.approx((area - 4 * math.pi * 0.25) / 4.0, rel=1e-6)


@pytest.mark.parametrize("config", ["clustered", "dispersed"])
def test_team_starts_are_valid_and_deterministic(room_with_hole, config):
    a = with_team(room_with_hole, 5, config)
    b = with_team(room_with_hole, 5, config)
    assert [r.start for r in a.team] == [r.start for r in b.team]
    cs = a.team[0].body_fp.erode(a.free)
    pts = np.array([r.start[:2] for r in a.team])
    assert all(cs.buffer(1e-6).covers(Point(p)) for p in pts)
    d = np.hypot(*(pts[:, None] - pts[None]).transpose(2, 0, 1))
    assert d[np.triu_indices(5, 1)].min() >= 1.0  # bodies do not overlap
    assert a.metadata["factors"]["team_size"] == 5


def test_synthetic_regeneration_is_exact():
    inst = generate(seed=11, width=25, height=18, obstacle_density=0.1, nonconvexity=1)
    again = regenerate(inst)
    assert again.hash == inst.hash


def test_splits_keep_pairs_together():
    insts = []
    for k in range(10):
        for variant in ("empty", "furnished"):
            i = generate(seed=k, width=12, height=10, obstacle_density=0.0)
            i = i.copy(id=f"room{k}_{variant}")
            i.metadata["pair"] = f"room{k}"
            insts.append(i)
    s = assign_splits(insts, 0.2)
    assert sum(v == "tuning" for v in s.values()) == 4
    for k in range(10):
        assert s[f"room{k}_empty"] == s[f"room{k}_furnished"]
    assert s == assign_splits(insts, 0.2)

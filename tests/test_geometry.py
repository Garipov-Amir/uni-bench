import math

import numpy as np
import pytest
import shapely
from shapely.geometry import LineString, Point, box

from uninbench.geometry.decomposition import boustrophedon_decomposition, parallel_swaths
from uninbench.geometry.dubins import dubins_paths
from uninbench.geometry.footprint import Footprint
from uninbench.geometry.raster import RasterGrid, mask_to_geometry
from uninbench.geometry.transit import TransitPlanner


def test_disc_erosion_dilation():
    fp = Footprint("disc", 1.0)
    b = box(0, 0, 10, 10)
    assert fp.erode(b).area == pytest.approx(81.0, rel=1e-9)
    assert fp.dilate(b).area == pytest.approx(100 + 40 * 0.5 + math.pi * 0.25, rel=1e-3)
    assert fp.radius == 0.5


def test_rect_minkowski_exact():
    fp = Footprint("rect", 2.0, 1.0)  # lateral 2 (y at heading 0), longitudinal 1 (x)
    b = box(0, 0, 10, 10)
    assert fp.erode(b).area == pytest.approx(9 * 8)
    assert fp.dilate(b).area == pytest.approx(11 * 12)
    assert fp.radius == pytest.approx(0.5 * math.hypot(2, 1))


@pytest.mark.parametrize("fp", [Footprint("disc", 1.0), Footprint("rect", 1.0, 0.4)])
def test_segment_intervals_match_sampling(fp):
    rng = np.random.default_rng(1)
    q = rng.uniform(-1, 4, size=(400, 2))
    p0, p1 = np.array([0.2, 0.1]), np.array([3.0, 1.7])
    psi = math.atan2(*(p1 - p0)[::-1])
    ok, s0, s1 = fp.segment_intervals(q[:, 0], q[:, 1], p0, p1, psi)
    L = np.hypot(*(p1 - p0))
    ss = np.linspace(0, L, 2001)
    for k in range(len(q)):
        inside = [fp.contains(q[k, 0], q[k, 1], *(p0 + (p1 - p0) * s / L), psi) for s in ss]
        inside = np.array(inside, dtype=bool).ravel()
        assert inside.any() == bool(ok[k])
        if inside.any():
            assert ss[inside].min() == pytest.approx(s0[k], abs=2e-3)
            assert ss[inside].max() == pytest.approx(s1[k], abs=2e-3)


def test_rasterize_measures_area():
    g = RasterGrid.covering((0, 0, 10, 10), 0.05)
    poly = Point(5, 5).buffer(3, quad_segs=64).difference(box(4, 4, 6, 6))
    m = g.rasterize(poly)
    assert g.measure(m) == pytest.approx(poly.area, rel=5e-3)
    assert mask_to_geometry(g, m).area == pytest.approx(g.measure(m))


def test_boustrophedon_single_hole_has_four_cells(room_with_hole):
    cells, adj = boustrophedon_decomposition(room_with_hole.free, 0.0)
    assert len(cells) == 4
    assert shapely.unary_union(cells).area == pytest.approx(room_with_hole.free.area)
    assert len(adj) == 4


def test_parallel_swaths_spacing():
    sw = parallel_swaths(box(0.5, 0.5, 9.5, 5.5), 0.0, 1.0)
    ys = sorted({round(s["line"].coords[0][1], 9) for s in sw})
    assert ys == pytest.approx([0.5, 1.5, 2.5, 3.5, 4.5, 5.5])


def test_dubins_reaches_goal_and_is_shortest_first():
    rng = np.random.default_rng(3)
    for _ in range(100):
        s = (*rng.uniform(-10, 10, 2), rng.uniform(-3, 3))
        g = (*rng.uniform(-10, 10, 2), rng.uniform(-3, 3))
        paths = dubins_paths(s, g, 2.0, 0.05)
        assert paths
        L, _, pts = paths[0]
        assert np.allclose(pts[-1][:2], g[:2])
        assert all(L <= p[0] + 1e-9 for p in paths)
        assert np.hypot(*np.diff(pts[:, :2], axis=0).T).sum() <= L + 1e-6


def test_transit_avoids_obstacle():
    free = box(0, 0, 10, 10).difference(box(3, 0, 4, 8))
    tp = TransitPlanner(free)
    path = tp.path((1, 1), (8, 1))
    assert len(path) > 2
    assert free.buffer(1e-6).covers(LineString(path))
    # the shortest path bends around the obstacle corner (3, 8)-(4, 8)
    expected = math.hypot(2, 7) + 1 + math.hypot(4, 7)
    assert tp.length((1, 1), (8, 1)) == pytest.approx(expected, rel=1e-6)

import json

import numpy as np
import pytest
from PIL import Image

from uninbench.datasets.areacoverage import import_areacoverage, reference_trajectories
from uninbench.datasets.fields import import_fields
from uninbench.datasets.rooms import import_rooms
from uninbench.datasets.safelog import synthetic_safelog
from uninbench.evaluator import evaluate
from uninbench.validation.reference import check_reference_routes


def _write_ac(root):
    d = root / "AC300" / "AC1_0000"
    (d / "mem_inf").mkdir(parents=True)
    (d / "outer_polygon").write_text("0 0\n30 0\n30 12\n0 12\n")
    (d / "holes").write_text("12 4\n18 4\n18 8\n12 8\n")
    rows = []
    for k, y in enumerate([0.0, 3, 6, 9, 12]):  # tracks on a 3 m pitch, first on the boundary
        xs = [(0, 12), (18, 30)] if 3 < y < 9 else [(0, 30)]
        for a, b in (xs if k % 2 == 0 else [(b, a) for a, b in xs[::-1]]):
            rows += [f"{a} {y} 1", f"{b} {y} 0"]
    # transit legs between pieces are deadheading (flag of the point they leave: 0)
    (d / "mem_inf" / "mem_inf_route_data0").write_text("\n".join(rows) + "\n")
    return root


def test_areacoverage_import_and_reference_check(tmp_path):
    root = _write_ac(tmp_path / "ac")
    insts = import_areacoverage(root)
    assert len(insts) == 1 and insts[0].family == "IF1"
    ri, ts = reference_trajectories(insts[0], root / "AC300" / "AC1_0000", "mem_inf")
    ev = evaluate(ri, ts, sensitivity=False)
    assert ev.metrics["service_length"] > 0
    rep = check_reference_routes(root, verbose=False)
    assert len(rep["rows"]) == 1


def test_fields_geojson_projection(tmp_path):
    lon0, lat0 = 5.0, 52.0
    ring = [[lon0, lat0], [lon0 + 0.002, lat0], [lon0 + 0.002, lat0 + 0.001], [lon0, lat0 + 0.001], [lon0, lat0]]
    (tmp_path / "f.geojson").write_text(json.dumps({"type": "FeatureCollection", "features": [
        {"type": "Feature", "properties": {}, "geometry": {"type": "Polygon", "coordinates": [ring]}}]}))
    inst = import_fields(tmp_path)[0]
    minx, miny, maxx, maxy = inst.environment.bounds
    assert maxx - minx == pytest.approx(0.002 * 111319.5 * np.cos(np.radians(52.0005)), rel=1e-3)
    assert maxy - miny == pytest.approx(0.001 * 111319.5, rel=1e-3)
    assert inst.metadata["projection"]["type"] == "equirectangular"


def test_rooms_pairs(tmp_path):
    free = np.zeros((120, 200), np.uint8)
    free[10:110, 10:95] = 255
    free[10:110, 105:190] = 255
    free[55:65, 95:105] = 255  # door
    seg = free.copy()
    seg[:, 99:101] = 0  # ground-truth room boundary
    furn = free.copy()
    furn[30:50, 30:60] = 0
    for name, a in (("m", free), ("m_furnitures", furn), ("m_gt_segmentation", seg)):
        Image.fromarray(a).save(tmp_path / f"{name}.png")
    insts = import_rooms(tmp_path, resolution=0.05, width=0.3, min_room_area=1.0)
    assert len(insts) == 4
    variants = {(i.metadata["room"], i.metadata["variant"]) for i in insts}
    assert len({r for r, _ in variants}) == 2
    furnished = [i for i in insts if i.metadata["variant"] == "furnished" and i.environment.holes]
    assert furnished and all(i.environment.source_resolution == 0.05 for i in insts)


def test_safelog_density_sweep():
    insts = synthetic_safelog(rows=6, cols=6, densities=[0.6, 1.0])
    assert insts[0].metadata["factors"]["graph_density"] < insts[1].metadata["factors"]["graph_density"]
    assert len(insts[1].environment.holes) == 0 and len(insts[0].environment.holes) > 0

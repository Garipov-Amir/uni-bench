"""IF2: agricultural fields (Fields2Benchmark / Fields2Cover formats).

Supported inputs, one field per file:

* Fields2Cover GML/XML (``<Field>`` with ``gml:outerBoundaryIs``/``innerBoundaryIs``),
  optionally with ``swath_width`` / ``sprayer_width`` attributes;
* GeoJSON (Polygon / MultiPolygon features);
* WKT.

Geographic coordinates (longitude/latitude) are projected to a local
metric frame (equirectangular projection about the field centroid, error well
below a centimetre at field scale); the projection is stored in the metadata.
"""

from __future__ import annotations

import json
import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import shapely
from shapely.geometry import Point, Polygon, shape
from shapely.ops import nearest_points

from ..geometry.decomposition import polygons_of
from ..geometry.footprint import Footprint
from ..instance.model import Environment, Instance, Robot

R_EARTH = 6378137.0


def _coords(text: str) -> np.ndarray:
    vals = [v for v in re.split(r"[\s]+", text.strip()) if v]
    pts = []
    for v in vals:
        parts = v.split(",")
        pts.append([float(parts[0]), float(parts[1])])
    return np.array(pts)


def read_gml(path: Path):
    root = ET.parse(path).getroot()
    polys, attrs = [], {}
    for el in root.iter():
        tag = el.tag.split("}")[-1]
        if tag == "Field":
            attrs = dict(el.attrib)
        if tag == "Polygon":
            outer, inner = None, []
            for ch in el.iter():
                t = ch.tag.split("}")[-1]
                if t in ("outerBoundaryIs", "exterior"):
                    c = [x for x in ch.iter() if x.tag.split("}")[-1] in ("coordinates", "posList")]
                    outer = _coords(c[0].text)
                elif t in ("innerBoundaryIs", "interior"):
                    c = [x for x in ch.iter() if x.tag.split("}")[-1] in ("coordinates", "posList")]
                    inner.append(_coords(c[0].text))
            if outer is not None:
                polys.append(Polygon(outer, inner))
    return shapely.unary_union(polys), attrs


def read_geojson(path: Path):
    d = json.loads(Path(path).read_text())
    feats = d["features"] if d.get("type") == "FeatureCollection" else [d]
    geoms = [shape(f["geometry"] if "geometry" in f else f) for f in feats]
    geoms = [shapely.force_2d(g) for g in geoms if g.geom_type in ("Polygon", "MultiPolygon")]
    props = feats[0].get("properties", {}) if feats else {}
    return shapely.unary_union(geoms), props or {}


def read_field(path) -> tuple:
    path = Path(path)
    suf = path.suffix.lower()
    if suf in (".xml", ".gml"):
        return read_gml(path)
    if suf in (".json", ".geojson"):
        return read_geojson(path)
    if suf == ".wkt":
        return shapely.force_2d(shapely.from_wkt(path.read_text())), {}
    raise ValueError(f"unsupported field format {suf}")


def is_geographic(g) -> bool:
    minx, miny, maxx, maxy = g.bounds
    return -180 <= minx and maxx <= 180 and -90 <= miny and maxy <= 90 and (maxx - minx) < 1 and (maxy - miny) < 1


def project_local(g):
    c = g.centroid
    lon0, lat0 = c.x, c.y
    kx = math.radians(1) * R_EARTH * math.cos(math.radians(lat0))
    ky = math.radians(1) * R_EARTH
    proj = shapely.transform(g, lambda xy: np.c_[(xy[:, 0] - lon0) * kx, (xy[:, 1] - lat0) * ky])
    minx, miny, _, _ = proj.bounds
    proj = shapely.transform(proj, lambda xy: xy - np.array([minx, miny]))
    return proj, {"type": "equirectangular", "lon0": lon0, "lat0": lat0, "offset": [minx, miny]}


def field_instance(iid: str, geom, width: float = 3.0, attrs=None, robot_kind: str = "rect", **meta) -> Instance:
    projection = None
    if is_geographic(geom):
        geom, projection = project_local(geom)
    geom = shapely.make_valid(geom)
    polys = polygons_of(geom)
    if not polys:
        raise ValueError(f"{iid}: no polygon")
    boundary = shapely.unary_union([Polygon(p.exterior) for p in polys])
    holes = [Polygon(h) for p in polys for h in p.interiors]
    env = Environment(boundary=boundary, holes=holes)
    attrs = attrs or {}
    if "swath_width" in attrs:
        width = float(attrs["swath_width"])
    tool = Footprint(robot_kind, width, 0.2 * width if robot_kind == "rect" else None)
    body = Footprint("disc", width)
    cs = body.erode(env.free)
    comps = polygons_of(cs) or polygons_of(env.free)
    main = max(comps, key=lambda c: c.area)
    corner = Point(env.bounds[0], env.bounds[1])
    q = nearest_points(main.buffer(-1e-6 * width), corner)[0]
    robot = Robot(id="r0", start=(q.x, q.y, 0.0), tool=tool, body=body, speed=2.0, turn_cost=0.5)
    m = {"source": "fields", "projection": projection, "attributes": attrs,
         "factors": {"area_ha": round(env.area / 1e4, 3), "holes": len(holes)}, **meta}
    return Instance(id=iid, environment=env, team=[robot], family="IF2", metadata=m)


def import_fields(src, out_dir=None, width: float = 3.0, pattern: str = "*", limit: int | None = None):
    src = Path(src)
    files = [src] if src.is_file() else sorted(p for p in src.glob(pattern)
                                                  if p.suffix.lower() in (".xml", ".gml", ".json", ".geojson",
                                                                          ".wkt"))
    out = []
    for f in files[:limit] if limit else files:
        try:
            g, attrs = read_field(f)
            inst = field_instance(f"field_{f.stem}", g, width, attrs, source_file=f.name)
        except (ValueError, ET.ParseError, KeyError) as e:
            print(f"skipping {f}: {e}")
            continue
        out.append(inst)
        if out_dir is not None:
            inst.save(Path(out_dir))
    return out

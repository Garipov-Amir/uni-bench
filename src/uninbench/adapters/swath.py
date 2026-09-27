"""Swath/isoline adapter: parallel passes spaced by the working width, or offset curves."""

from __future__ import annotations

import numpy as np
import shapely
from shapely.geometry import LineString, MultiLineString

from ..geometry.decomposition import lines_of, offset_swaths, parallel_swaths
from ..reconstruction import GeometryRegion
from .base import Adapter, Representation, reachable_cspace
from .geometric import resolve_angle


class SwathRepresentation(Representation):
    family = "swath"

    def __init__(self, team, free, params, swaths, angle, spacing, headland=None):
        super().__init__(team, free, params)
        self.swaths = swaths  # list of {"id", "line", "row", "closed"}
        self.angle = angle
        self.spacing = spacing
        self.headland = headland
        self.info = {}

    def layers(self) -> dict:
        out = {"swaths": {"type": "lines", "coords": [np.round(np.asarray(s["line"].coords), 4).tolist()
                                                      for s in self.swaths]}}
        if self.headland is not None and not self.headland.is_empty:
            out["headland"] = {"type": "polygons", "wkt": [shapely.to_wkt(self.headland, rounding_precision=5)]}
        return out


class SwathAdapter(Adapter):
    name = "swath"
    family = "swath"
    version = "1.0"
    defaults = {
        "mode": "parallel",  # 'parallel' | 'offset'
        "angle": "auto",  # pass direction; 'auto' (min. width), 'longest_edge' or radians
        "overlap": 0.0,  # fractional overlap between neighbouring swaths
        "headland": 0.0,  # width of a headland band kept free for turning (agricultural profile)
        "min_length": 0.0,
        "perimeter": True,  # one contour pass along every boundary of the (reachable) configuration space
        "headland_passes": True,  # contour passes covering the headland band (smoothed to the turning radius)
    }

    def resolved_params(self, inst) -> dict:
        p = dict(self.params)
        p["angle"] = resolve_angle(p["angle"], inst.free)
        p["spacing"] = inst.min_width * (1.0 - float(p["overlap"]))
        return p

    def convert(self, inst) -> SwathRepresentation:
        p = self.resolved_params(inst)
        robot = inst.team[0]
        free = inst.free
        cspace = reachable_cspace(free, robot.body_fp, [r.start[:2] for r in inst.team])
        region = cspace
        headland = None
        if p["headland"] > 0:
            region = cspace.buffer(-float(p["headland"]))
            headland = shapely.difference(free, free.buffer(-float(p["headland"]) - robot.body_fp.radius))
        spacing = p["spacing"]
        if p["mode"] == "parallel":
            sw = parallel_swaths(region, p["angle"], spacing, inset=0.0, min_length=float(p["min_length"]))
            if p["perimeter"]:
                # contour pass along every boundary; swath parts within half a width of the
                # contour are redundant and trimmed (the global lattice phase is kept, so no
                # gaps open next to boundaries parallel to the swaths)
                rings = offset_swaths(region, spacing, max_rings=1)
                for r in rings:
                    r["row"] = -1
                band = region.boundary.buffer(0.5 * spacing)
                trimmed = []
                for s in sw:
                    for piece in lines_of(s["line"].difference(band)):
                        if piece.length > max(float(p["min_length"]), 1e-6):
                            c = np.asarray(piece.coords)
                            trimmed.append({"line": LineString(c[[0, -1]]), "row": s["row"]})
                sw = rings + trimmed
            if p["headland"] > 0 and p["headland_passes"]:
                # smoothed slightly beyond the turning radius (planners keep a safety margin)
                sw = sw + headland_rings(cspace, float(p["headland"]), spacing,
                                         1.15 * max((r.min_turn_radius or 0.0) for r in inst.team))
        elif p["mode"] == "offset":
            sw = offset_swaths(region, spacing)
        else:
            raise ValueError(p["mode"])
        for k, s in enumerate(sw):
            s["id"] = k
            s.setdefault("closed", False)
        rep = SwathRepresentation(inst.team, free, p, sw, p["angle"], spacing, headland)
        rep.__dict__.setdefault("_cache", {})[("cspace", robot.body_fp)] = cspace
        lines = [s["line"] for s in sw]
        swept = robot.tool.sweep(MultiLineString(lines)) if lines else shapely.Polygon()
        rep.reconstruction = GeometryRegion(swept)
        rep.info = {"swaths": len(sw), "angle": p["angle"], "spacing": spacing,
                    "swath_length": float(sum(s["line"].length for s in sw))}
        return rep


def smooth(region, r: float):
    """Opening and closing with a disc of radius r: every boundary curve then has a
    radius of curvature of at least r (up to the polygonal approximation)."""
    if r <= 0:
        return region
    g = region.buffer(-r).buffer(r)
    g2 = g.buffer(r).buffer(-r)
    return g2 if region.buffer(1e-6 * max(r, 1.0)).covers(g2) else g


def headland_rings(cspace, headland: float, spacing: float, r_min: float) -> list[dict]:
    """Closed contour passes inside the headland band, outermost first."""
    n = max(1, int(np.ceil(headland / spacing - 1e-6)))
    margin = 0.01 * spacing  # keeps discretised arcs strictly inside the configuration space
    out = []
    for k in range(n):
        region = smooth(cspace.buffer(-k * spacing - margin), r_min)
        for ring in offset_swaths(region, spacing, max_rings=1):
            ring["row"] = -1 - k
            ring["headland"] = True
            out.append(ring)
    return out


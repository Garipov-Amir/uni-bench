"""Geometric adapter: polygon with holes, optionally decomposed into boustrophedon cells."""

from __future__ import annotations

import math

import shapely

from ..geometry.decomposition import (boustrophedon_decomposition, longest_edge_angle, min_width_angle,
                                      polygons_of)
from ..reconstruction import GeometryRegion
from .base import Adapter, Representation


def resolve_angle(spec, geom) -> float:
    if spec in (None, "auto", "min_width"):
        return min_width_angle(geom)
    if spec == "longest_edge":
        return longest_edge_angle(geom)
    return float(spec) % math.pi


class PolygonRepresentation(Representation):
    family = "geometric"

    def __init__(self, team, free, params, cells=None, adjacency=None, angle=None):
        super().__init__(team, free, params)
        self.cells = cells or []
        self.adjacency = adjacency or []
        self.angle = angle
        self.info = {}

    def layers(self) -> dict:
        out = {"free": {"type": "polygons", "wkt": [shapely.to_wkt(p, rounding_precision=6) for p in
                                                    polygons_of(self.free)]}}
        if self.cells:
            out["cells"] = {"type": "polygons", "wkt": [shapely.to_wkt(c, rounding_precision=6) for c in self.cells]}
        return out


class GeometricAdapter(Adapter):
    name = "geometric"
    family = "geometric"
    version = "1.0"
    defaults = {
        "mode": "boustrophedon",  # 'polygon' (pass-through) | 'boustrophedon'
        "angle": "auto",  # pass direction for the decomposition
        "inflation": 0.0,
        "simplify": 0.0,
    }

    def resolved_params(self, inst) -> dict:
        p = dict(self.params)
        if p["mode"] == "boustrophedon":
            p["angle"] = resolve_angle(p["angle"], inst.free)
        return p

    def convert(self, inst) -> PolygonRepresentation:
        p = self.resolved_params(inst)
        free = inst.free
        if p["inflation"] > 0:
            free = free.buffer(-float(p["inflation"]))
        if p["simplify"] > 0:
            free = free.simplify(float(p["simplify"]), preserve_topology=True)
        if p["mode"] == "polygon":
            rep = PolygonRepresentation(inst.team, free, p)
            rep.reconstruction = GeometryRegion(free)
            return rep
        tol = 1e-9 * max(1.0, max(inst.environment.bounds[2] - inst.environment.bounds[0],
                                  inst.environment.bounds[3] - inst.environment.bounds[1]))
        cells, adj = boustrophedon_decomposition(free, p["angle"], tol=tol)
        rep = PolygonRepresentation(inst.team, free, p, cells, adj, p["angle"])
        rep.reconstruction = GeometryRegion(shapely.unary_union(cells) if cells else shapely.Polygon())
        rep.info = {"cells": len(cells), "angle": p["angle"]}
        return rep

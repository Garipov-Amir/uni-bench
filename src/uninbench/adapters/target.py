"""Target adapter: coverage targets sampled on a lattice at a declared spacing."""

from __future__ import annotations

import math

import numpy as np
import shapely

from ..geometry.decomposition import polygons_of
from ..reconstruction import DiscUnionRegion
from .base import Adapter, Representation, reachable_cspace


class TargetRepresentation(Representation):
    family = "target"

    def __init__(self, team, free, params, targets: np.ndarray, radius: float, spacing: float):
        super().__init__(team, free, params)
        self.targets = targets
        self.radius = radius
        self.spacing = spacing
        self.info = {}

    def layers(self) -> dict:
        return {"targets": {"type": "points", "coords": np.round(self.targets, 4).tolist()}}


class TargetAdapter(Adapter):
    name = "target"
    family = "target"
    version = "1.0"
    defaults = {
        "lattice": "square",  # 'square' | 'hex'
        "spacing": "auto",  # 'auto' -> covering spacing for the tool disc, or metres
        "boundary_samples": True,  # add samples along the configuration-space boundary
    }

    @staticmethod
    def radius(inst) -> float:
        """Coverage radius of a target: targets are swept while moving, so the lateral
        half-width of the tool (not the inradius of a rectangular implement) is used."""
        return 0.5 * inst.team[0].tool.working_width

    def spacing(self, inst) -> float:
        s = self.params["spacing"]
        r = self.radius(inst)
        if s == "auto" or s is None:
            return r * (math.sqrt(3) if self.params["lattice"] == "hex" else math.sqrt(2)) * 0.999
        return float(s)

    def resolved_params(self, inst) -> dict:
        p = dict(self.params)
        p["spacing"] = self.spacing(inst)
        return p

    def convert(self, inst) -> TargetRepresentation:
        p = self.resolved_params(inst)
        robot = inst.team[0]
        cs = reachable_cspace(inst.free, robot.body_fp, [r.start[:2] for r in inst.team])
        s = p["spacing"]
        minx, miny, maxx, maxy = cs.bounds if not cs.is_empty else (0, 0, 0, 0)
        if p["lattice"] == "hex":
            dy = s * math.sqrt(3) / 2
            rows = np.arange(miny, maxy + dy, dy)
            pts = []
            for k, y in enumerate(rows):
                off = 0.5 * s if k % 2 else 0.0
                xs = np.arange(minx + off, maxx + s, s)
                pts.append(np.c_[xs, np.full_like(xs, y)])
            pts = np.concatenate(pts) if pts else np.zeros((0, 2))
        else:
            xs = np.arange(minx, maxx + s, s)
            ys = np.arange(miny, maxy + s, s)
            X, Y = np.meshgrid(xs, ys)
            pts = np.c_[X.ravel(), Y.ravel()]
        shapely.prepare(cs)
        pts = pts[shapely.contains_xy(cs, pts[:, 0], pts[:, 1])] if len(pts) else pts
        if p["boundary_samples"] and not cs.is_empty:
            bpts = []
            for poly in polygons_of(cs):
                for ring in [poly.exterior, *poly.interiors]:
                    n = max(1, int(math.ceil(ring.length / s)))
                    d = np.linspace(0, ring.length, n, endpoint=False)
                    bpts.append(np.array([[q.x, q.y] for q in (ring.interpolate(v) for v in d)]))
            if bpts:
                bpts = np.concatenate(bpts)
                if len(pts):
                    from scipy.spatial import cKDTree

                    dd, _ = cKDTree(pts).query(bpts)
                    bpts = bpts[dd > 0.5 * s]
                pts = np.concatenate([pts, bpts]) if len(pts) else bpts
        rep = TargetRepresentation(inst.team, inst.free, p, pts, self.radius(inst), s)
        rep.__dict__.setdefault("_cache", {})[("cspace", robot.body_fp)] = cs
        rep.reconstruction = DiscUnionRegion(pts, self.radius(inst))
        rep.info = {"targets": int(len(pts)), "spacing": s}
        return rep

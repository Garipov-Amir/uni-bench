"""Robot footprints (tool and body) and the Minkowski operations built on them.

Two footprint shapes are supported:

* ``disc``  -- a disc of diameter ``width``; heading independent.
* ``rect``  -- a rectangle of lateral extent ``width`` (the working width) and
  longitudinal extent ``length`` that rotates with the robot heading.

Both are centred at the robot reference point and are point symmetric, so
``-B == B`` and erosion/dilation use the same shape.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
import shapely
from shapely.geometry import Point, Polygon
from shapely.geometry.base import BaseGeometry

QUAD_SEGS = 16


@dataclass(frozen=True)
class Footprint:
    kind: str = "disc"
    width: float = 1.0
    length: float | None = None

    def __post_init__(self):
        if self.kind not in ("disc", "rect"):
            raise ValueError(f"unknown footprint kind {self.kind!r}")
        if not self.width > 0:
            raise ValueError("footprint width must be positive")

    # ------------------------------------------------------------------ sizes
    @property
    def along(self) -> float:
        """Longitudinal extent."""
        if self.kind == "disc":
            return self.width
        return self.width if self.length is None else float(self.length)

    @property
    def working_width(self) -> float:
        return float(self.width)

    @property
    def radius(self) -> float:
        """Radius ``b`` of the smallest disc containing the footprint."""
        if self.kind == "disc":
            return self.width / 2.0
        return 0.5 * math.hypot(self.width, self.along)

    @property
    def inradius(self) -> float:
        if self.kind == "disc":
            return self.width / 2.0
        return 0.5 * min(self.width, self.along)

    @property
    def area(self) -> float:
        if self.kind == "disc":
            return math.pi * (self.width / 2.0) ** 2
        return self.width * self.along

    # ------------------------------------------------------------ geometry
    def local_vertices(self, psi: float = 0.0) -> np.ndarray:
        """Vertices of the (polygonal) footprint in the local frame rotated by psi."""
        if self.kind == "disc":
            n = 4 * QUAD_SEGS
            a = np.linspace(0, 2 * np.pi, n, endpoint=False)
            return np.c_[np.cos(a), np.sin(a)] * (self.width / 2.0)
        hl, hw = self.along / 2.0, self.width / 2.0
        pts = np.array([[hl, hw], [-hl, hw], [-hl, -hw], [hl, -hw]])
        c, s = math.cos(psi), math.sin(psi)
        return pts @ np.array([[c, s], [-s, c]])

    def polygon(self, x: float = 0.0, y: float = 0.0, psi: float = 0.0) -> Polygon:
        if self.kind == "disc":
            return Point(x, y).buffer(self.width / 2.0, quad_segs=QUAD_SEGS)
        return Polygon(self.local_vertices(psi) + np.array([x, y]))

    def dilate(self, geom: BaseGeometry, psi: float = 0.0) -> BaseGeometry:
        """Minkowski sum ``geom (+) B``."""
        if geom.is_empty:
            return geom
        if self.kind == "disc":
            return geom.buffer(self.width / 2.0, quad_segs=QUAD_SEGS)
        return shapely.union(geom, _boundary_sweep(geom, self.local_vertices(psi)))

    def erode(self, geom: BaseGeometry, psi: float = 0.0) -> BaseGeometry:
        """Minkowski difference ``geom (-) B = {x : x + B subset geom}``."""
        if geom.is_empty:
            return geom
        if self.kind == "disc":
            return geom.buffer(-self.width / 2.0, quad_segs=QUAD_SEGS)
        return shapely.difference(geom, _boundary_sweep(geom, self.local_vertices(psi)))

    def sweep(self, geom: BaseGeometry, psi: float = 0.0, flat: bool = False) -> BaseGeometry:
        """Area swept when the footprint follows ``geom`` (lines/points)."""
        if self.kind == "disc":
            return geom.buffer(self.width / 2.0, quad_segs=QUAD_SEGS)
        if flat:
            return geom.buffer(self.width / 2.0, cap_style="flat")
        return self.dilate(geom, psi)

    # ------------------------------------------------------------ intervals
    def segment_intervals(self, qx, qy, p0, p1, psi: float):
        """Parameter interval during which query points lie in the footprint.

        The footprint moves with constant heading ``psi`` from ``p0`` to ``p1``;
        the returned ``s_in <= s_out`` are arclengths along the segment in
        ``[0, L]``.  Because footprint and segment are convex the set of
        parameters is always a single interval.
        """
        qx = np.asarray(qx, dtype=float)
        qy = np.asarray(qy, dtype=float)
        dx, dy = float(p1[0] - p0[0]), float(p1[1] - p0[1])
        L = math.hypot(dx, dy)
        rx = qx - p0[0]
        ry = qy - p0[1]
        if L > 1e-12:
            ux, uy = dx / L, dy / L
        else:
            ux = uy = 0.0
        if self.kind == "disc":
            r = self.width / 2.0
            a = rx * ux + ry * uy
            c2 = rx * rx + ry * ry - a * a
            h2 = r * r - c2
            ok = h2 >= -1e-12
            h = np.sqrt(np.maximum(h2, 0.0))
            if L <= 1e-12:
                ok = rx * rx + ry * ry <= r * r + 1e-12
                z = np.zeros_like(rx)
                return ok, z, z.copy()
            s0 = np.maximum(a - h, 0.0)
            s1 = np.minimum(a + h, L)
            ok &= s0 <= s1
            return ok, s0, s1
        c, s = math.cos(psi), math.sin(psi)
        lo = np.zeros_like(rx)
        hi = np.full_like(rx, L)
        ok = np.ones(rx.shape, dtype=bool)
        for (ex, ey, half) in ((c, s, self.along / 2.0), (-s, c, self.width / 2.0)):
            proj = rx * ex + ry * ey
            coef = ux * ex + uy * ey
            if abs(coef) < 1e-12:
                ok &= np.abs(proj) <= half + 1e-12
            else:
                sa = (proj - half) / coef
                sb = (proj + half) / coef
                lo = np.maximum(lo, np.minimum(sa, sb))
                hi = np.minimum(hi, np.maximum(sa, sb))
        ok &= lo <= hi + 1e-12
        return ok, lo, np.maximum(lo, hi)

    def contains(self, qx, qy, x: float, y: float, psi: float = 0.0):
        ok, _, _ = self.segment_intervals(qx, qy, (x, y), (x, y), psi)
        return ok

    # ------------------------------------------------------------ io
    def to_dict(self) -> dict:
        d = {"kind": self.kind, "width": float(self.width)}
        if self.kind == "rect" and self.length is not None:
            d["length"] = float(self.length)
        return d

    @classmethod
    def from_dict(cls, d: dict) -> "Footprint":
        return cls(kind=d.get("kind", "disc"), width=float(d["width"]), length=d.get("length"))

    @classmethod
    def from_polygon(cls, coords) -> "Footprint":
        """Build a footprint from a robot polygon (e.g. AreaCoverage robot_polygon)."""
        pts = np.asarray(coords, dtype=float)
        poly = Polygon(pts)
        minx, miny, maxx, maxy = poly.bounds
        w, l = maxx - minx, maxy - miny
        if abs(poly.area - w * l) < 1e-6 * max(w * l, 1e-12):
            return cls(kind="rect", width=float(max(w, l)), length=float(min(w, l)))
        return cls(kind="disc", width=float(2 * math.sqrt(poly.area / math.pi)))


def _boundary_sweep(geom: BaseGeometry, verts: np.ndarray) -> BaseGeometry:
    """Union over boundary edges e of ``e (+) B`` for a convex polygon B."""
    segs = []
    for ring in _rings(geom):
        c = np.asarray(ring.coords)
        if len(c) < 2:
            continue
        segs.append(np.stack([c[:-1], c[1:]], axis=1))
    if not segs:
        return Polygon()
    segs = np.concatenate(segs)  # (S, 2, 2)
    pts = np.concatenate([segs[:, :1, :] + verts[None], segs[:, 1:, :] + verts[None]], axis=1)
    hulls = shapely.convex_hull(shapely.multipoints(pts))
    return shapely.unary_union(hulls)


def _rings(geom: BaseGeometry):
    if geom.is_empty:
        return
    if geom.geom_type == "Polygon":
        yield geom.exterior
        yield from geom.interiors
    elif hasattr(geom, "geoms"):
        for g in geom.geoms:
            yield from _rings(g)
    elif geom.geom_type in ("LineString", "LinearRing"):
        yield geom

"""Boustrophedon cellular decomposition and swath (pass) generation."""

from __future__ import annotations

import math

import numpy as np
import shapely
from shapely import affinity
from shapely.geometry import LineString, Polygon
from shapely.geometry.base import BaseGeometry


def polygons_of(geom: BaseGeometry) -> list[Polygon]:
    if geom is None or geom.is_empty:
        return []
    if geom.geom_type == "Polygon":
        return [geom]
    out = []
    for g in getattr(geom, "geoms", []):
        out.extend(polygons_of(g))
    return out


def lines_of(geom: BaseGeometry) -> list[LineString]:
    if geom is None or geom.is_empty:
        return []
    if geom.geom_type == "LineString":
        return [geom]
    if geom.geom_type == "LinearRing":
        return [LineString(geom.coords)]
    out = []
    for g in getattr(geom, "geoms", []):
        out.extend(lines_of(g))
    return out


def rotate(geom, angle: float, inverse: bool = False):
    return affinity.rotate(geom, -angle if not inverse else angle, origin=(0.0, 0.0), use_radians=True)


# ---------------------------------------------------------------------- swaths
def parallel_swaths(region: BaseGeometry, angle: float, spacing: float, inset: float = 0.0,
                    min_length: float = 0.0) -> list[dict]:
    """Parallel passes in direction ``angle`` clipped to ``region``.

    Pass offsets are distributed evenly between ``min + inset`` and
    ``max - inset`` (perpendicular to the passes) with a spacing of at most
    ``spacing``.  Returns dicts ``{"line": LineString, "row": int}`` in the
    world frame, ordered by row and then along the pass direction.
    """
    if region.is_empty:
        return []
    rot = rotate(region, angle)
    minx, miny, maxx, maxy = rot.bounds
    lo, hi = miny + inset, maxy - inset
    if hi < lo:
        offsets = np.array([(miny + maxy) / 2.0])
    else:
        n = int(math.ceil((hi - lo) / spacing - 1e-6)) + 1
        offsets = np.array([(lo + hi) / 2.0]) if n <= 1 else np.linspace(lo, hi, n)
    lines = shapely.linestrings(
        np.stack([np.stack([np.full_like(offsets, minx - 1), offsets], 1),
                  np.stack([np.full_like(offsets, maxx + 1), offsets], 1)], 1))
    shapely.prepare(rot)
    inter = shapely.intersection(lines, rot)
    out = []
    for row, g in enumerate(inter):
        pieces = lines_of(shapely.line_merge(g) if g.geom_type in ("MultiLineString",) else g)
        pieces = [p for p in pieces if p.length > max(min_length, 1e-9)]
        pieces.sort(key=lambda p: min(p.coords[0][0], p.coords[-1][0]))
        for p in pieces:
            c = np.asarray(p.coords)
            c = c[np.argsort(c[:, 0])][[0, -1]]
            out.append({"line": rotate(LineString(c), angle, inverse=True), "row": row})
    return out


def offset_swaths(region: BaseGeometry, spacing: float, max_rings: int = 10000) -> list[dict]:
    """Inward offset curves (isolines) of ``region`` spaced by ``spacing``."""
    out = []
    k = 0
    cur = region
    while not cur.is_empty and k < max_rings:
        for poly in polygons_of(cur):
            for ring in [poly.exterior, *poly.interiors]:
                if ring.length > 1e-9:
                    out.append({"line": LineString(ring.coords), "row": k, "closed": True})
        k += 1
        cur = region.buffer(-k * spacing)
    return out


# --------------------------------------------------------------- decomposition
def boustrophedon_decomposition(poly: BaseGeometry, angle: float, tol: float = 1e-9):
    """Boustrophedon cells of ``poly`` for passes in direction ``angle``.

    In the frame rotated by ``-angle`` the passes are horizontal and the sweep
    line moves along y.  The region is cut into horizontal slabs at every
    vertex ordinate, and slab pieces are merged whenever they are connected
    one-to-one across a slab boundary.

    Returns ``(cells, adjacency)`` with cells as world-frame polygons and
    adjacency as a list of index pairs.
    """
    rot = rotate(poly, angle)
    if rot.is_empty:
        return [], []
    minx, miny, maxx, maxy = rot.bounds
    ys = []
    for p in polygons_of(rot):
        for ring in [p.exterior, *p.interiors]:
            ys.extend(np.asarray(ring.coords)[:, 1])
    ys = np.sort(np.r_[miny, np.asarray(ys, dtype=float), maxy])
    keep = np.r_[True, np.diff(ys) > tol]  # cluster ordinates closer than tol, keeping exact values
    ys = ys[keep]
    bands = [(a, b) for a, b in zip(ys[:-1], ys[1:]) if b - a > tol]
    boxes = shapely.box(minx - 1, [a for a, _ in bands], maxx + 1, [b for _, b in bands])
    parts = shapely.intersection(boxes, rot)
    pieces = []  # (band index, polygon)
    for k, g in enumerate(parts):
        for p in polygons_of(g):
            if p.area > tol:
                pieces.append((k, p))
    by_band: dict[int, list[int]] = {}
    for idx, (k, _) in enumerate(pieces):
        by_band.setdefault(k, []).append(idx)
    up: dict[int, list[int]] = {i: [] for i in range(len(pieces))}
    down: dict[int, list[int]] = {i: [] for i in range(len(pieces))}
    for k in by_band:
        for a in by_band.get(k, []):
            pa = pieces[a][1]
            ax0, _, ax1, _ = pa.bounds
            for b in by_band.get(k + 1, []):
                pb = pieces[b][1]
                bx0, _, bx1, _ = pb.bounds
                if bx1 < ax0 - tol or ax1 < bx0 - tol:
                    continue
                if pa.intersection(pb).length > tol:
                    up[a].append(b)
                    down[b].append(a)
    parent = list(range(len(pieces)))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    for a in range(len(pieces)):
        if len(up[a]) == 1:
            b = up[a][0]
            if len(down[b]) == 1:
                parent[find(b)] = find(a)
    groups: dict[int, list[int]] = {}
    for i in range(len(pieces)):
        groups.setdefault(find(i), []).append(i)
    roots = sorted(groups, key=lambda r: min(pieces[i][0] for i in groups[r]))
    index = {r: n for n, r in enumerate(roots)}
    cells = []
    for r in roots:
        g = shapely.unary_union([pieces[i][1] for i in groups[r]])
        cells.append(rotate(g, angle, inverse=True))
    adj = set()
    for a in range(len(pieces)):
        for b in up[a]:
            ca, cb = index[find(a)], index[find(b)]
            if ca != cb:
                adj.add((min(ca, cb), max(ca, cb)))
    return cells, sorted(adj)


def longest_edge_angle(geom: BaseGeometry) -> float:
    """Direction of the longest boundary edge (a common sweep-direction heuristic)."""
    best, ang = -1.0, 0.0
    for p in polygons_of(geom):
        c = np.asarray(p.exterior.coords)
        d = np.diff(c, axis=0)
        L = np.hypot(d[:, 0], d[:, 1])
        k = int(np.argmax(L))
        if L[k] > best:
            best, ang = L[k], math.atan2(d[k, 1], d[k, 0])
    return float(ang % math.pi)


def min_width_angle(geom: BaseGeometry, samples: int = 36) -> float:
    """Pass direction that minimises the number of passes (min. altitude)."""
    hull = geom.convex_hull
    best, ang = math.inf, 0.0
    for a in np.linspace(0, math.pi, samples, endpoint=False):
        r = rotate(hull, a)
        _, miny, _, maxy = r.bounds
        if maxy - miny < best:
            best, ang = maxy - miny, a
    return float(ang)

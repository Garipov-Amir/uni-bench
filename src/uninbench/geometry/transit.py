"""Collision-free shortest paths in a polygonal configuration space.

A reduced visibility graph over the reflex vertices of the free configuration
space is built once; all-pairs shortest paths between graph nodes are
precomputed so that each query only needs to connect its two end points.
"""

from __future__ import annotations

import math

import numpy as np
import shapely
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import dijkstra
from shapely.geometry import LineString, Point
from shapely.geometry.base import BaseGeometry
from shapely.geometry.polygon import orient
from shapely.ops import nearest_points


class TransitPlanner:
    def __init__(self, free: BaseGeometry, tol: float | None = None, simplify: float = 0.0, max_nodes: int = 1500):
        """``simplify`` > 0 places graph nodes on a slightly shrunken, simplified copy of
        the free space (fewer nodes, paths keep a clearance of about ``simplify``)."""
        self.free = free
        minx, miny, maxx, maxy = free.bounds if not free.is_empty else (0, 0, 1, 1)
        scale = max(maxx - minx, maxy - miny, 1.0)
        self.tol = tol if tol is not None else 1e-7 * scale
        self.check = free.buffer(self.tol)
        shapely.prepare(self.check)
        tol_s = max(simplify, 0.0)
        nodes = self._nodes(free, tol_s)
        while len(nodes) > max_nodes and tol_s < 0.01 * scale:
            tol_s = max(tol_s * 4, 1e-4 * scale)
            nodes = self._nodes(free, tol_s)
        self.nodes = nodes if len(nodes) else np.zeros((0, 2))
        self._build()

    @staticmethod
    def _nodes(free, s: float) -> np.ndarray:
        if s <= 0:
            return _reflex_vertices(free)
        inner = free.buffer(-s, quad_segs=4).simplify(0.5 * s, preserve_topology=True)
        if inner.is_empty:
            return _reflex_vertices(free)
        return _reflex_vertices(inner)

    # ------------------------------------------------------------------ build
    def _build(self):
        n = len(self.nodes)
        self.dist = np.zeros((n, n))
        self.pred = np.full((n, n), -9999, dtype=np.int32)
        if n == 0:
            return
        ii, jj = np.triu_indices(n, k=1)
        a = self.nodes[ii]
        b = self.nodes[jj]
        lines = shapely.linestrings(np.stack([a, b], axis=1))
        vis = shapely.covers(self.check, lines)
        ii, jj = ii[vis], jj[vis]
        w = np.hypot(*(self.nodes[ii] - self.nodes[jj]).T)
        m = coo_matrix((np.r_[w, w], (np.r_[ii, jj], np.r_[jj, ii])), shape=(n, n)).tocsr()
        self.dist, self.pred = dijkstra(m, directed=False, return_predecessors=True)

    # ------------------------------------------------------------------ query
    def visible(self, p, pts: np.ndarray) -> np.ndarray:
        if len(pts) == 0:
            return np.zeros(0, dtype=bool)
        lines = shapely.linestrings(np.stack([np.broadcast_to(p, pts.shape), pts], axis=1))
        return shapely.covers(self.check, lines)

    def segment_free(self, a, b) -> bool:
        if np.allclose(a, b):
            return bool(self.check.covers(Point(a)))
        return bool(self.check.covers(LineString([tuple(a), tuple(b)])))

    def snap(self, p) -> np.ndarray:
        p = np.asarray(p, dtype=float)
        if self.check.covers(Point(p)):
            return p
        q = nearest_points(self.free, Point(p))[0]
        return np.array([q.x, q.y])

    def path(self, a, b) -> np.ndarray:
        """Shortest collision-free polyline from a to b (inclusive).

        End points outside the free space are connected to their nearest free
        point; if no path exists the straight segment is returned (and will be
        reported by the evaluator as a collision).
        """
        a = np.asarray(a, dtype=float)
        b = np.asarray(b, dtype=float)
        if self.segment_free(a, b):
            return np.array([a, b])
        a2, b2 = self.snap(a), self.snap(b)
        pre = [a] if not np.allclose(a, a2) else []
        post = [b] if not np.allclose(b, b2) else []
        if self.segment_free(a2, b2):
            return np.array(pre + [a2, b2] + post)
        va = np.flatnonzero(self.visible(a2, self.nodes))
        vb = np.flatnonzero(self.visible(b2, self.nodes))
        if len(va) == 0 or len(vb) == 0:
            return np.array([a, b])
        da = np.hypot(*(self.nodes[va] - a2).T)
        db = np.hypot(*(self.nodes[vb] - b2).T)
        tot = da[:, None] + self.dist[np.ix_(va, vb)] + db[None, :]
        k = np.argmin(tot)
        if not np.isfinite(tot.flat[k]):
            return np.array([a, b])
        u, v = va[k // len(vb)], vb[k % len(vb)]
        chain = [v]
        while chain[-1] != u:
            p = self.pred[u, chain[-1]]
            if p < 0:
                break
            chain.append(p)
        chain.reverse()
        pts = [a2] + [self.nodes[c] for c in chain] + [b2]
        return _dedupe(np.array(pre + pts + post))

    def length(self, a, b) -> float:
        p = self.path(a, b)
        return float(np.hypot(*np.diff(p, axis=0).T).sum())

    def lengths_from(self, a, targets: np.ndarray) -> np.ndarray:
        """Approximate shortest-path lengths from ``a`` to many targets."""
        a = self.snap(a)
        targets = np.asarray(targets, dtype=float)
        out = np.hypot(*(targets - a).T)
        vis = self.visible(a, targets)
        hidden = np.flatnonzero(~vis)
        if len(hidden) == 0 or len(self.nodes) == 0:
            return out
        va = np.flatnonzero(self.visible(a, self.nodes))
        if len(va) == 0:
            out[hidden] = np.inf
            return out
        da = np.hypot(*(self.nodes[va] - a).T)
        dn = (da[:, None] + self.dist[va]).min(axis=0)  # a -> every node
        for h in hidden:
            t = targets[h]
            vt = np.flatnonzero(self.visible(t, self.nodes))
            if len(vt) == 0:
                out[h] = np.inf
                continue
            out[h] = float((dn[vt] + np.hypot(*(self.nodes[vt] - t).T)).min())
        return out


def _dedupe(pts: np.ndarray) -> np.ndarray:
    keep = [0]
    for k in range(1, len(pts)):
        if np.hypot(*(pts[k] - pts[keep[-1]])) > 1e-12:
            keep.append(k)
    return pts[keep] if len(keep) > 1 else pts[[0, -1]]


def _reflex_vertices(geom: BaseGeometry) -> np.ndarray:
    out = []
    polys = [geom] if geom.geom_type == "Polygon" else list(getattr(geom, "geoms", []))
    for poly in polys:
        if poly.is_empty or poly.geom_type != "Polygon":
            continue
        poly = orient(poly, sign=1.0)
        for ring in [poly.exterior, *poly.interiors]:
            c = np.asarray(ring.coords)[:-1]
            if len(c) < 3:
                continue
            prev = np.roll(c, 1, axis=0)
            nxt = np.roll(c, -1, axis=0)
            e1 = c - prev
            e2 = nxt - c
            cross = e1[:, 0] * e2[:, 1] - e1[:, 1] * e2[:, 0]
            # free region is on the left of every oriented ring: a right turn is reflex
            out.append(c[cross < -1e-12])
    if not out:
        return np.zeros((0, 2))
    return np.unique(np.concatenate(out), axis=0)


def polyline_length(pts) -> float:
    pts = np.asarray(pts, dtype=float)
    if len(pts) < 2:
        return 0.0
    return float(np.hypot(*np.diff(pts, axis=0).T).sum())


def heading(a, b) -> float:
    return math.atan2(b[1] - a[1], b[0] - a[0])

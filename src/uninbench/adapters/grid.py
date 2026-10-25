"""Grid/graph adapter: rasterize F_c at the planning resolution delta_p.

Cells are classified by a declared rule (``center``, ``full`` containment or
area ``fraction``), a 4- or 8-connected graph is built whose edges are
checked against the configuration space, and -- for STC-type planners --
coarse cells of 2x2 sub-cells are formed:

* ``coarse=True``: only *complete* coarse cells (all four sub-cells free and
  mutually connected) are used; sub-cells of partly free coarse cells are
  given up (representation loss, as in STC, MSTC, DARP, MFC, MIP);
* ``coarse="partial"``: every traversable sub-cell is used and coarse cells
  with at least one sub-cell are kept (``coarse["any"]``), for planners that
  cover incomplete terrain vertices (Extended-STC of LS-MCPP).

Coarse vertex weights (``coarse["weights"]``) are the mean terrain cost of
the four sub-cell centres (1 without a terrain layer).
"""

from __future__ import annotations

import math

import numpy as np
import shapely
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import dijkstra

from ..geometry.raster import RasterGrid
from ..geometry.transit import TransitPlanner
from ..reconstruction import RasterRegion
from ..util import pack_mask
from .base import Adapter, Representation


class GridRepresentation(Representation):
    family = "grid"
    lazy_caches = ("_sp_cache",)

    def __init__(self, team, free, params, grid: RasterGrid, cells: np.ndarray, edges: np.ndarray,
                 weights: np.ndarray, cspace, coarse: dict | None = None):
        super().__init__(team, free, params)
        self.grid = grid
        self.cells = cells  # (ny, nx) bool: traversable cells
        self.edges = edges  # (E, 2) flat indices
        self.weights = weights
        self._cspace = cspace
        self.coarse = coarse
        N = grid.nx * grid.ny
        if len(edges):
            self.adj = coo_matrix((np.r_[weights, weights], (np.r_[edges[:, 0], edges[:, 1]],
                                                            np.r_[edges[:, 1], edges[:, 0]])), shape=(N, N)).tocsr()
        else:
            self.adj = coo_matrix((N, N)).tocsr()
        self.starts: dict = {}
        self._sp_cache: dict = {}
        self.info: dict = {}

    # ------------------------------------------------------------ indexing
    @property
    def nx(self):
        return self.grid.nx

    def idx(self, i, j):
        return np.asarray(i) * self.grid.nx + np.asarray(j)

    def rc(self, idx):
        idx = np.asarray(idx)
        return idx // self.grid.nx, idx % self.grid.nx

    def center(self, idx):
        i, j = self.rc(idx)
        x, y = self.grid.center(i, j)
        return np.stack([x, y], axis=-1)

    @property
    def nodes(self) -> np.ndarray:
        return np.flatnonzero(self.cells.ravel())

    def neighbors(self, idx: int) -> np.ndarray:
        row = self.adj.getrow(idx)
        return row.indices

    # ------------------------------------------------------------ paths
    def shortest(self, src: int):
        if src not in self._sp_cache:
            if len(self._sp_cache) > 256:
                self._sp_cache.clear()
            self._sp_cache[src] = dijkstra(self.adj, directed=False, indices=src, return_predecessors=True)
        return self._sp_cache[src]

    def distances(self, src: int) -> np.ndarray:
        return self.shortest(src)[0]

    def path(self, a: int, b: int) -> list[int]:
        if a == b:
            return [a]
        dist, pred = self.shortest(a)
        if not np.isfinite(dist[b]):
            return []
        out = [b]
        while out[-1] != a:
            out.append(int(pred[out[-1]]))
        return out[::-1]

    def cspace(self, robot=None):
        return self._cspace

    def transit(self, robot=None) -> TransitPlanner:
        cache = self.__dict__.setdefault("_cache", {})
        if "transit" not in cache:
            cache["transit"] = TransitPlanner(self._cspace)
        return cache["transit"]

    # ------------------------------------------------------------ coarse cells
    def sub_cells(self, I, J):
        """Sub-cell flat indices (BL, BR, TL, TR) of coarse cell (I, J)."""
        i, j = 2 * I, 2 * J
        return (self.idx(i, j), self.idx(i, j + 1), self.idx(i + 1, j), self.idx(i + 1, j + 1))

    def coarse_of(self, idx):
        i, j = self.rc(idx)
        return i // 2, j // 2

    def layers(self) -> dict:
        out = {"grid": {"type": "raster", "grid": self.grid.to_dict(), "mask": pack_mask(self.cells)}}
        if self.coarse is not None:
            out["coarse_grid"] = {"type": "raster", "grid": self.coarse["grid"].to_dict(),
                                  "mask": pack_mask(self.coarse["cells"])}
        return out


class GridAdapter(Adapter):
    name = "grid"
    family = "grid"
    version = "1.2"
    defaults = {
        "resolution": "auto",  # 'auto' -> resolution_factor * working width, or metres
        "resolution_factor": 1.0,
        "rule": "center",  # 'center' | 'full' | 'fraction'
        "fraction": 0.5,
        "connectivity": 4,
        "inflation": 0.0,
        "coarse": False,  # STC: coarse cells of 2x2 sub-cells (True | 'partial')
        "weighted": False,
        "check_edges": True,
        "require_center_in_cspace": True,
    }

    def resolution(self, inst) -> float:
        r = self.params["resolution"]
        if r == "auto" or r is None:
            return float(self.params["resolution_factor"]) * inst.min_width
        return float(r)

    def resolved_params(self, inst) -> dict:
        p = dict(self.params)
        p["resolution"] = self.resolution(inst)
        return p

    def convert(self, inst) -> GridRepresentation:
        p = self.params
        res = self.resolution(inst)
        free = inst.free
        if p["inflation"] > 0:
            free = free.buffer(-float(p["inflation"]))
        body = inst.team[0].body_fp
        cspace = body.erode(free)
        tol = 1e-7 * max(1.0, res)
        cs_tol = cspace.buffer(tol)
        shapely.prepare(cs_tol)

        grid = RasterGrid.covering(inst.free.bounds, res)
        if p["coarse"]:
            grid = RasterGrid(grid.x0, grid.y0, res, grid.nx + grid.nx % 2, grid.ny + grid.ny % 2)
        X, Y = grid.centers()
        cx, cy = X.ravel(), Y.ravel()
        frac, dist_b = _cell_fractions(grid, free, cx, cy)
        rule = p["rule"]
        if rule == "center":
            cells = shapely.contains_xy(free, cx, cy)
        elif rule == "full":
            cells = frac >= 1.0 - 1e-9
        elif rule == "fraction":
            cells = frac >= float(p["fraction"])
        else:
            raise ValueError(f"unknown cell rule {rule!r}")
        if p["require_center_in_cspace"]:
            cells &= shapely.covers_xy(cs_tol, cx, cy) if hasattr(shapely, "covers_xy") else \
                shapely.intersects_xy(cs_tol, cx, cy)
        cells = cells.reshape(grid.shape)

        # ------------------------------------------------ graph
        offs = [(0, 1), (1, 0)] + ([(1, 1), (1, -1)] if int(p["connectivity"]) == 8 else [])
        edges = []
        for di, dj in offs:
            a = cells[max(0, -di):grid.ny - max(0, di), max(0, -dj):grid.nx - max(0, dj)]
            b = cells[max(0, di):, max(0, dj):][:a.shape[0], :a.shape[1]]
            ii, jj = np.nonzero(a & b)
            ii = ii + max(0, -di)
            jj = jj + max(0, -dj)
            ea = ii * grid.nx + jj
            eb = (ii + di) * grid.nx + (jj + dj)
            edges.append(np.stack([ea, eb], 1))
        edges = np.concatenate(edges) if edges else np.zeros((0, 2), dtype=int)
        if p["check_edges"] and len(edges):
            seg = res * math.hypot(1, 1) if int(p["connectivity"]) == 8 else res
            safe = np.minimum(dist_b[edges[:, 0]], dist_b[edges[:, 1]]) > body.radius + seg + tol
            chk = np.flatnonzero(~safe)
            if len(chk):
                pa = np.c_[cx[edges[chk, 0]], cy[edges[chk, 0]]]
                pb = np.c_[cx[edges[chk, 1]], cy[edges[chk, 1]]]
                ok = shapely.covers(cs_tol, shapely.linestrings(np.stack([pa, pb], 1)))
                keep = np.ones(len(edges), dtype=bool)
                keep[chk[~ok]] = False
                edges = edges[keep]
        lengths = np.hypot(cx[edges[:, 0]] - cx[edges[:, 1]], cy[edges[:, 0]] - cy[edges[:, 1]])
        weights = lengths
        if p["weighted"] and inst.environment.terrain is not None:
            c = inst.environment.terrain.cost_at(cx, cy)
            weights = lengths * 0.5 * (c[edges[:, 0]] + c[edges[:, 1]])

        coarse = None
        used = cells
        if p["coarse"]:
            coarse = _coarse(grid, cells, edges)
            tc = inst.environment.terrain.cost_at(cx, cy).reshape(grid.shape) if inst.environment.terrain \
                is not None else np.ones(grid.shape)
            ny2, nx2 = coarse["cells"].shape
            coarse["weights"] = tc[:2 * ny2, :2 * nx2].reshape(ny2, 2, nx2, 2).mean(axis=(1, 3))
            if p["coarse"] == "partial":
                coarse["any"] = cells[:2 * ny2, :2 * nx2].reshape(ny2, 2, nx2, 2).any(axis=(1, 3))
            else:
                used = np.zeros_like(cells)
                CI, CJ = np.nonzero(coarse["cells"])
                for di in (0, 1):
                    for dj in (0, 1):
                        used[2 * CI + di, 2 * CJ + dj] = True
        rep = GridRepresentation(inst.team, free, self.resolved_params(inst), grid, cells, edges, weights,
                                 cspace, coarse)
        rep.reconstruction = RasterRegion(grid, used)
        rep.used = used
        _attach_starts(rep, inst, used)
        rep.info = {"cells": int(cells.sum()), "used_cells": int(used.sum()), "edges": int(len(edges)),
                    "delta_p": res}
        if coarse is not None:
            rep.info["coarse_cells"] = int(coarse["cells"].sum())
            if "any" in coarse:
                rep.info["coarse_cells_any"] = int(coarse["any"].sum())
        return rep


def _cell_fractions(grid: RasterGrid, free, cx, cy):
    """Free-area fraction of each cell and distance of cell centres to the boundary."""
    shapely.prepare(free)
    inside = shapely.contains_xy(free, cx, cy)
    bnd = free.boundary
    shapely.prepare(bnd)
    dist_b = shapely.distance(bnd, shapely.points(cx, cy))
    half_diag = grid.res * math.sqrt(0.5)
    frac = np.where(inside, 1.0, 0.0)
    near = np.flatnonzero(dist_b < half_diag + 1e-12)
    if len(near):
        h = grid.res / 2
        boxes = shapely.box(cx[near] - h, cy[near] - h, cx[near] + h, cy[near] + h)
        frac[near] = shapely.area(shapely.intersection(boxes, free)) / grid.cell_area
    return frac, dist_b


def _coarse(grid: RasterGrid, cells: np.ndarray, edges: np.ndarray) -> dict:
    ny2, nx2 = grid.ny // 2, grid.nx // 2
    eset = set(map(tuple, np.sort(edges, axis=1).tolist()))

    def has(a, b):
        return (min(a, b), max(a, b)) in eset

    nx = grid.nx
    cfree = np.zeros((ny2, nx2), dtype=bool)
    for I in range(ny2):
        for J in range(nx2):
            i, j = 2 * I, 2 * J
            if not (cells[i, j] and cells[i, j + 1] and cells[i + 1, j] and cells[i + 1, j + 1]):
                continue
            bl, br, tl, tr = i * nx + j, i * nx + j + 1, (i + 1) * nx + j, (i + 1) * nx + j + 1
            cfree[I, J] = has(bl, br) and has(br, tr) and has(tr, tl) and has(tl, bl)
    cedges = []
    for I in range(ny2):
        for J in range(nx2):
            if not cfree[I, J]:
                continue
            i, j = 2 * I, 2 * J
            if J + 1 < nx2 and cfree[I, J + 1]:
                if has(i * nx + j + 1, i * nx + j + 2) and has((i + 1) * nx + j + 1, (i + 1) * nx + j + 2):
                    cedges.append((I * nx2 + J, I * nx2 + J + 1))
            if I + 1 < ny2 and cfree[I + 1, J]:
                if has((i + 1) * nx + j, (i + 2) * nx + j) and has((i + 1) * nx + j + 1, (i + 2) * nx + j + 1):
                    cedges.append((I * nx2 + J, (I + 1) * nx2 + J))
    cgrid = RasterGrid(grid.x0, grid.y0, grid.res * 2, nx2, ny2)
    return {"grid": cgrid, "cells": cfree, "edges": np.array(cedges, dtype=int).reshape(-1, 2)}


def _attach_starts(rep: GridRepresentation, inst, used: np.ndarray):
    """Connect every start pose to a reachable cell of the representation."""
    cand = np.flatnonzero(used.ravel())
    if len(cand) == 0:
        return
    C = rep.center(cand)
    cs_tol = rep.cspace().buffer(1e-7 * max(1.0, rep.grid.res))
    shapely.prepare(cs_tol)
    for r in inst.team:
        sx, sy = r.start[:2]
        d = np.hypot(C[:, 0] - sx, C[:, 1] - sy)
        order = np.argsort(d, kind="stable")[:64]
        lines = shapely.linestrings(np.stack([np.broadcast_to([sx, sy], (len(order), 2)), C[order]], 1))
        ok = shapely.covers(cs_tol, lines)
        if np.any(ok):
            k = order[int(np.argmax(ok))]
            path = np.array([[sx, sy], C[k]])
        else:
            k = order[0]
            path = rep.transit().path((sx, sy), C[k])
        rep.starts[r.id] = {"cell": int(cand[k]), "path": np.asarray(path, dtype=float)}

"""Axis-aligned raster grids and exact scan-line rasterization of vector geometry.

Rows ``i`` index the y axis (row 0 at the bottom), columns ``j`` the x axis.
A cell is inside a region when its centre is (centre rule); the measure of a
rasterized region is ``cell_area * count``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
import shapely
from shapely.geometry.base import BaseGeometry


@dataclass(frozen=True)
class RasterGrid:
    x0: float
    y0: float
    res: float
    nx: int
    ny: int

    @classmethod
    def covering(cls, bounds, res: float, pad: float = 0.0) -> "RasterGrid":
        minx, miny, maxx, maxy = bounds
        minx -= pad
        miny -= pad
        maxx += pad
        maxy += pad
        nx = max(1, int(math.ceil((maxx - minx) / res - 1e-9)))
        ny = max(1, int(math.ceil((maxy - miny) / res - 1e-9)))
        return cls(float(minx), float(miny), float(res), nx, ny)

    # ------------------------------------------------------------ basics
    @property
    def shape(self) -> tuple[int, int]:
        return (self.ny, self.nx)

    @property
    def cell_area(self) -> float:
        return self.res * self.res

    @property
    def bounds(self):
        return (self.x0, self.y0, self.x0 + self.nx * self.res, self.y0 + self.ny * self.res)

    def xs(self) -> np.ndarray:
        return self.x0 + (np.arange(self.nx) + 0.5) * self.res

    def ys(self) -> np.ndarray:
        return self.y0 + (np.arange(self.ny) + 0.5) * self.res

    def centers(self) -> tuple[np.ndarray, np.ndarray]:
        return np.meshgrid(self.xs(), self.ys())

    def center(self, i, j):
        return self.x0 + (np.asarray(j) + 0.5) * self.res, self.y0 + (np.asarray(i) + 0.5) * self.res

    def cell_of(self, x, y):
        j = np.floor((np.asarray(x) - self.x0) / self.res).astype(int)
        i = np.floor((np.asarray(y) - self.y0) / self.res).astype(int)
        return i, j

    def inside(self, i, j):
        i = np.asarray(i)
        j = np.asarray(j)
        return (i >= 0) & (i < self.ny) & (j >= 0) & (j < self.nx)

    def window(self, xmin, ymin, xmax, ymax):
        """Row/column index ranges (half open) of cells whose centre lies in the box."""
        j0 = max(0, int(math.ceil((xmin - self.x0) / self.res - 0.5)))
        j1 = min(self.nx, int(math.floor((xmax - self.x0) / self.res - 0.5)) + 1)
        i0 = max(0, int(math.ceil((ymin - self.y0) / self.res - 0.5)))
        i1 = min(self.ny, int(math.floor((ymax - self.y0) / self.res - 0.5)) + 1)
        return i0, i1, j0, j1

    def refine(self, factor: int = 2) -> "RasterGrid":
        return RasterGrid(self.x0, self.y0, self.res / factor, self.nx * factor, self.ny * factor)

    def cell_box(self, i: int, j: int):
        x = self.x0 + j * self.res
        y = self.y0 + i * self.res
        return shapely.box(x, y, x + self.res, y + self.res)

    # ------------------------------------------------------------ io
    def to_dict(self) -> dict:
        return {"x0": self.x0, "y0": self.y0, "res": self.res, "nx": self.nx, "ny": self.ny}

    @classmethod
    def from_dict(cls, d: dict) -> "RasterGrid":
        return cls(float(d["x0"]), float(d["y0"]), float(d["res"]), int(d["nx"]), int(d["ny"]))

    # ------------------------------------------------------------ rasterize
    def rasterize(self, geom: BaseGeometry | None) -> np.ndarray:
        """Boolean mask of cells whose centre lies in ``geom`` (scan-line fill)."""
        mask = np.zeros(self.shape, dtype=bool)
        if geom is None or geom.is_empty:
            return mask
        gminx, gminy, gmaxx, gmaxy = geom.bounds
        i0, i1, _, _ = self.window(gminx, gminy, gmaxx, gmaxy)
        if i1 <= i0:
            return mask
        ys = self.y0 + (np.arange(i0, i1) + 0.5) * self.res
        xa = min(gminx, self.x0) - 1.0
        xb = max(gmaxx, self.x0 + self.nx * self.res) + 1.0
        lines = shapely.linestrings(
            np.stack([np.stack([np.full_like(ys, xa), ys], 1), np.stack([np.full_like(ys, xb), ys], 1)], 1)
        )
        shapely.prepare(geom)
        inter = shapely.intersection(lines, geom)
        for k, g in enumerate(inter):
            if g.is_empty:
                continue
            i = i0 + k
            for xs0, xs1 in _x_intervals(g):
                j0 = max(0, int(math.ceil((xs0 - self.x0) / self.res - 0.5)))
                j1 = min(self.nx, int(math.floor((xs1 - self.x0) / self.res - 0.5)) + 1)
                if j1 > j0:
                    mask[i, j0:j1] = True
        return mask

    def measure(self, mask: np.ndarray) -> float:
        return float(np.count_nonzero(mask)) * self.cell_area


def _x_intervals(g: BaseGeometry):
    t = g.geom_type
    if t == "LineString":
        xs = np.asarray(g.coords)[:, 0]
        yield float(xs.min()), float(xs.max())
    elif t == "Point":
        yield g.x, g.x
    elif hasattr(g, "geoms"):
        for h in g.geoms:
            yield from _x_intervals(h)


def resample_mask(src_grid: RasterGrid, src_mask: np.ndarray, dst_grid: RasterGrid) -> np.ndarray:
    """Nearest-cell resampling: a destination centre takes the value of the source
    cell that contains it (exact for regions that are unions of source cells)."""
    X, Y = dst_grid.centers()
    i, j = src_grid.cell_of(X, Y)
    ok = src_grid.inside(i, j)
    out = np.zeros(dst_grid.shape, dtype=bool)
    out[ok] = src_mask[i[ok], j[ok]]
    return out


def mask_to_geometry(grid: RasterGrid, mask: np.ndarray) -> BaseGeometry:
    """Union of the cells of a mask as a polygonal geometry (row-run merging).

    Runs are merged in integer cell coordinates (exact) and scaled afterwards,
    so that adjacent rows share edges exactly."""
    boxes = []
    for i in range(grid.ny):
        row = mask[i]
        if not row.any():
            continue
        d = np.diff(np.r_[0, row.astype(np.int8), 0])
        starts = np.flatnonzero(d == 1)
        ends = np.flatnonzero(d == -1)
        for a, b in zip(starts, ends):
            boxes.append(shapely.box(float(a), float(i), float(b), float(i + 1)))
    if not boxes:
        return shapely.Polygon()
    g = shapely.unary_union(boxes)
    return shapely.transform(g, lambda xy: xy * grid.res + np.array([grid.x0, grid.y0]))

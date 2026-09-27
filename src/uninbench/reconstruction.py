"""Reconstructed free space  \\hat F_i = Phi_i(R_i)  of a representation (R3).

Reconstructions are regions that can be evaluated on any raster grid, so the
fidelity check and the evaluator's loss decomposition work at the evaluation
resolution delta_e regardless of the representation's own resolution.
"""

from __future__ import annotations

import numpy as np
from scipy.spatial import cKDTree
from shapely.geometry.base import BaseGeometry

from .geometry.raster import RasterGrid, mask_to_geometry, resample_mask
from .instance.model import from_wkt, to_wkt
from .util import pack_mask, unpack_mask


class Reconstruction:
    kind = "abstract"

    def mask(self, grid: RasterGrid) -> np.ndarray:
        raise NotImplementedError

    def geometry(self) -> BaseGeometry | None:
        return None

    def to_dict(self) -> dict:
        raise NotImplementedError

    @staticmethod
    def from_dict(d: dict) -> "Reconstruction":
        kind = d["kind"]
        if kind == "geometry":
            return GeometryRegion(from_wkt(d["wkt"]))
        if kind == "raster":
            return RasterRegion(RasterGrid.from_dict(d["grid"]), unpack_mask(d["mask"]))
        if kind == "discs":
            return DiscUnionRegion(np.asarray(d["points"], dtype=float), float(d["radius"]))
        raise ValueError(kind)


class GeometryRegion(Reconstruction):
    kind = "geometry"

    def __init__(self, geom: BaseGeometry):
        self.geom = geom

    def mask(self, grid: RasterGrid) -> np.ndarray:
        return grid.rasterize(self.geom)

    def geometry(self) -> BaseGeometry:
        return self.geom

    def to_dict(self) -> dict:
        return {"kind": self.kind, "wkt": to_wkt(self.geom)}


class RasterRegion(Reconstruction):
    """Union of the cells of a (planning) grid."""

    kind = "raster"

    def __init__(self, grid: RasterGrid, cells: np.ndarray):
        self.grid = grid
        self.cells = np.asarray(cells, dtype=bool)

    def mask(self, grid: RasterGrid) -> np.ndarray:
        return resample_mask(self.grid, self.cells, grid)

    def geometry(self) -> BaseGeometry:
        return mask_to_geometry(self.grid, self.cells)

    def to_dict(self) -> dict:
        return {"kind": self.kind, "grid": self.grid.to_dict(), "mask": pack_mask(self.cells)}


class DiscUnionRegion(Reconstruction):
    """Union of tool discs placed at coverage targets."""

    kind = "discs"

    def __init__(self, points: np.ndarray, radius: float):
        self.points = np.asarray(points, dtype=float).reshape(-1, 2)
        self.radius = float(radius)

    def mask(self, grid: RasterGrid) -> np.ndarray:
        out = np.zeros(grid.shape, dtype=bool)
        if len(self.points) == 0:
            return out
        X, Y = grid.centers()
        tree = cKDTree(self.points)
        d, _ = tree.query(np.c_[X.ravel(), Y.ravel()], k=1, distance_upper_bound=self.radius * (1 + 1e-9))
        return np.isfinite(d).reshape(grid.shape)

    def to_dict(self) -> dict:
        return {"kind": self.kind, "points": np.round(self.points, 9).tolist(), "radius": self.radius}

"""Identity adapter for graph-native sources (e.g. SafeLog maps).

The instance metadata carries the original graph ``{"rows", "cols",
"cell_size", "origin", "edges"}`` on a 4-connected lattice; graph-native
planners receive it unchanged instead of a re-rasterized geometry.
"""

from __future__ import annotations

import numpy as np

from ..geometry.raster import RasterGrid
from ..reconstruction import RasterRegion
from .base import Adapter
from .grid import GridRepresentation, _attach_starts


class GraphAdapter(Adapter):
    name = "graph"
    family = "grid"
    version = "1.1"
    defaults = {"coarse": False}

    def convert(self, inst) -> GridRepresentation:
        g = inst.metadata.get("graph")
        if not g:
            raise ValueError(f"instance {inst.id} carries no source graph; use the grid adapter")
        rows, cols = int(g["rows"]), int(g["cols"])
        s = float(g["cell_size"])
        x0, y0 = map(float, g.get("origin", (0.0, 0.0)))
        vertices = set(int(v) for v in g.get("vertices", range(rows * cols)))
        gedges = [tuple(sorted(map(int, e))) for e in g["edges"]]
        body = inst.team[0].body_fp
        cspace = body.erode(inst.free)
        if not self.params["coarse"]:
            grid = RasterGrid(x0, y0, s, cols, rows)
            cells = np.zeros(grid.shape, dtype=bool)
            for v in vertices:
                cells[v // cols, v % cols] = True
            edges = np.array(gedges, dtype=int).reshape(-1, 2)
            weights = np.full(len(edges), s)
            rep = GridRepresentation(inst.team, inst.free, dict(self.params), grid, cells, edges, weights, cspace)
            used = cells
        else:
            grid = RasterGrid(x0, y0, s / 2, 2 * cols, 2 * rows)
            nx = grid.nx
            cells = np.zeros(grid.shape, dtype=bool)
            cfree = np.zeros((rows, cols), dtype=bool)
            sub = []
            for v in vertices:
                I, J = v // cols, v % cols
                cfree[I, J] = True
                i, j = 2 * I, 2 * J
                cells[i:i + 2, j:j + 2] = True
                bl, br, tl, tr = i * nx + j, i * nx + j + 1, (i + 1) * nx + j, (i + 1) * nx + j + 1
                sub += [(bl, br), (br, tr), (tl, tr), (bl, tl)]
            cedges = []
            for a, b in gedges:
                Ia, Ja, Ib, Jb = a // cols, a % cols, b // cols, b % cols
                i, j = 2 * Ia, 2 * Ja
                if Ib == Ia and Jb == Ja + 1:
                    sub += [(i * nx + j + 1, i * nx + j + 2), ((i + 1) * nx + j + 1, (i + 1) * nx + j + 2)]
                elif Jb == Ja and Ib == Ia + 1:
                    sub += [((i + 1) * nx + j, (i + 2) * nx + j), ((i + 1) * nx + j + 1, (i + 2) * nx + j + 1)]
                else:
                    raise ValueError(f"edge ({a},{b}) is not between 4-neighbours")
                cedges.append((a, b))
            edges = np.array(sub, dtype=int).reshape(-1, 2)
            vw = np.ones(rows * cols)
            for key, val in (g.get("vertex_weights") or {}).items():
                vw[int(key)] = float(val)
            coarse = {"grid": RasterGrid(x0, y0, s, cols, rows), "cells": cfree,
                      "edges": np.array(cedges, dtype=int).reshape(-1, 2), "weights": vw.reshape(rows, cols)}
            rep = GridRepresentation(inst.team, inst.free, dict(self.params), grid, cells, edges,
                                     np.full(len(edges), s / 2), cspace, coarse)
            used = cells
        rep.reconstruction = RasterRegion(grid, used)
        rep.used = used
        _attach_starts(rep, inst, used)
        rep.info = {"vertices": len(vertices), "edges": len(gedges)}
        return rep

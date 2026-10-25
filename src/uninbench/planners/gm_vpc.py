"""GM-VPC (Nair & Guruprasad, 2020) -- generalized Voronoi partition and cover, staged mode.

The area is divided into 2D x 2D cells (D: robot footprint, the coarse cells of
the grid adapter) and partitioned by a Voronoi partition in the combined
geodesic-Manhattan metric: the distance from a robot to a cell is the length
of the shortest 4-connected path of cells, which is the Manhattan distance in
free space and respects obstacles.  The partition is computed as a
simultaneous wavefront from the robots' cells (ties go to the lower robot
index), so every Voronoi region is contiguous and every cell belongs to exactly
one region.  Each robot then covers its region with Spanning Tree Coverage
(Gabriely & Rimon, 2001), one of the two single-robot planners used in the paper.
"""

from __future__ import annotations

from collections import deque

import numpy as np

from .base import Manifest, Stage
from .darp import CoarseRegions, STCPerRegion, _DARPBase
from .grid_common import DIRECTION_PARAM, TREE_PARAM, partition_metrics


class GMVoronoi(Stage):
    kind = "work_distribution"
    name = "gm_voronoi"
    consumes = ("model",)
    produces = "partition"

    def run(self, data, rep, ctx, params):
        model = data["model"]
        nodes, adj = model["nodes"], model["adj"]
        pos = {int(n): k for k, n in enumerate(nodes)}
        seeds, used, seed_cells = [], set(), {}
        for r, root in zip(rep.team, model["roots"]):
            k = pos[root]
            if k in used:  # two robots in one cell: the next free cell of the wavefront
                dq, seen = deque([k]), {k}
                while dq:
                    x = dq.popleft()
                    if x not in used:
                        k = x
                        break
                    for y in adj.indices[adj.indptr[x]:adj.indptr[x + 1]]:
                        if int(y) not in seen:
                            seen.add(int(y))
                            dq.append(int(y))
            used.add(k)
            seeds.append(k)
            seed_cells[r.id] = int(nodes[k])
        labels = np.full(len(nodes), -1, dtype=int)
        frontier = []
        for i, k in enumerate(seeds):
            labels[k] = i
            frontier.append(k)
        while frontier:  # synchronous wavefront, robots in index order
            nxt = []
            for x in frontier:
                for y in adj.indices[adj.indptr[x]:adj.indptr[x + 1]]:
                    if labels[y] < 0:
                        labels[y] = labels[x]
                        nxt.append(int(y))
            frontier = nxt
        return {"labels": labels, "nodes": nodes, "seed_cells": seed_cells, "info": {}, "adj": adj}

    def metrics(self, out, rep):
        return partition_metrics(out["labels"], len(rep.team), out["adj"])


class GMVPC(_DARPBase):
    manifest = Manifest(
        name="gm_vpc",
        version="1.0",
        reference="Nair & Guruprasad (2020), GM-VPC: An algorithm for multi-robot coverage of known spaces using "
                  "generalized Voronoi partition (with Spanning Tree Coverage)",
        family="grid",
        adapter="grid",
        adapter_params={"coarse": True, "connectivity": 4},
        constraints={"multi_robot"},
        mode="staged",
        stages=["coarse_regions", "gm_voronoi", "stc_per_region"],
        hyperparameters=[TREE_PARAM, DIRECTION_PARAM],
        core_modules=["uninbench/planners/gm_vpc.py", "uninbench/planners/grid_common.py"],
        description="Geodesic-Manhattan Voronoi partition of the 2D x 2D cells, STC inside every region.",
    )
    stages = [CoarseRegions(), GMVoronoi(), STCPerRegion()]

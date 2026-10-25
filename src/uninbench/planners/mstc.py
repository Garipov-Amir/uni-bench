"""Multi-robot Spanning Tree Coverage (Hazon & Kaminka, 2005) -- staged mode.

Route construction comes *before* work distribution: one STC tour is built
over the whole region and then split among the robots with the original
non-backtracking rule: each robot follows the tour from its own position until
it reaches the position of the next robot along the tour.  The optimal
makespan split of the same tour is MSTC* (:mod:`uninbench.planners.mstc_star`).
"""

from __future__ import annotations

import numpy as np

from ..trajectory import SERVICE
from .base import Manifest, PlanResult, Stage, StagedPlanner
from .grid_common import (DIRECTION_PARAM, TREE_PARAM, cell_routes_to_trajectories, coarse_component, stc_route,
                          tree_layer)


class CoarseGridModel(Stage):
    kind = "environment_model"
    name = "coarse_grid"
    consumes = ("representation",)
    produces = "model"

    def run(self, data, rep, ctx, params):
        nx2 = rep.coarse["grid"].nx
        comps = {}
        for r in rep.team:
            I, J = rep.coarse_of(rep.starts[r.id]["cell"])
            root = int(I) * nx2 + int(J)
            if not any(root in c for c in comps.values()):
                comps[root] = coarse_component(rep, root)
        return {"components": comps}

    def summary(self, out):
        return {"components": len(out["components"]),
                "coarse_cells": int(sum(len(c) for c in out["components"].values()))}


class STCTour(Stage):
    kind = "route_construction"
    name = "stc_tour"
    consumes = ("model",)
    produces = "tour"

    def run(self, data, rep, ctx, params):
        comps = data["model"]["components"]
        r0 = rep.team[0]
        start = rep.starts[r0.id]["cell"]
        nx2 = rep.coarse["grid"].nx
        I, J = rep.coarse_of(start)
        root = int(I) * nx2 + int(J)
        nodes = next(c for c in comps.values() if root in c)
        tour, tree = stc_route(rep, nodes, start, params["tree"], params["direction"], ctx.rng)
        return {"cells": tour, "tree": tree}

    def summary(self, out):
        return {"tour_cells": len(out["cells"])}


class TourSplit(Stage):
    """Non-backtracking split of the STC tour (Hazon & Kaminka, 2005)."""

    kind = "work_distribution"
    name = "tour_split"
    consumes = ("tour",)
    produces = "routes"

    def run(self, data, rep, ctx, params):
        tour = data["tour"]["cells"]
        pos = {c: k for k, c in enumerate(tour)}
        N = len(tour)
        placed = []
        routes = {}
        for r in rep.team:
            c = rep.starts[r.id]["cell"]
            if c in pos:
                placed.append((pos[c], r.id))
            else:
                routes[r.id] = []
        placed.sort()
        for k, (p, rid) in enumerate(placed):
            nxt = placed[(k + 1) % len(placed)][0]
            if len(placed) == 1:
                seg = tour[p:] + tour[:p]
            elif nxt > p:
                seg = tour[p:nxt]
            elif nxt < p:
                seg = tour[p:] + tour[:nxt]
            else:
                seg = [] if k > 0 and placed[k - 1][0] == p else tour[p:] + tour[:p]
            routes[rid] = [(c, SERVICE) for c in seg[1:]] if seg else []
        return {"routes": routes, "segments": {rid: len(v) + 1 for rid, v in routes.items()}, "N": N}

    def metrics(self, out, rep):
        sizes = np.array(list(out["segments"].values()), dtype=float)
        return {"segment_sizes": sizes.astype(int).tolist(),
                "imbalance": float(sizes.max() / sizes.mean()) if sizes.mean() > 0 else None}


class MSTC(StagedPlanner):
    manifest = Manifest(
        name="mstc",
        version="1.1",
        reference="Hazon & Kaminka (2005), Redundancy, efficiency and robustness in multi-robot coverage",
        family="grid",
        adapter="grid",
        adapter_params={"coarse": True, "connectivity": 4},
        constraints={"multi_robot"},
        mode="staged",
        stages=["coarse_grid", "stc_tour", "tour_split"],
        hyperparameters=[TREE_PARAM, DIRECTION_PARAM],
        core_modules=["uninbench/planners/mstc.py", "uninbench/planners/grid_common.py"],
        description="One STC tour over the whole region, split among the robots.",
    )
    stages = [CoarseGridModel(), STCTour(), TourSplit()]

    def finish(self, data, rep, ctx) -> PlanResult:
        routes = data["routes"]["routes"]
        ts = cell_routes_to_trajectories(rep, routes, self.manifest, ctx)
        return PlanResult(ts, native={rid: [c for c, _ in rt] for rid, rt in routes.items()},
                          layers={"spanning_tree": tree_layer(rep, data["tour"]["tree"])})

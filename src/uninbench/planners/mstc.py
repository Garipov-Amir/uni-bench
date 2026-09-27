"""Multi-robot Spanning Tree Coverage (Hazon & Kaminka, 2005) -- staged mode.

Route construction comes *before* work distribution: one STC tour is built
over the whole region and then split among the robots.

* ``nonbacktracking`` split (original MSTC): each robot follows the tour from
  its own position until it reaches the next robot's position;
* ``balanced`` split: the cyclic tour is cut into m contiguous pieces of equal
  size, pieces are assigned to robots minimising the largest completion time
  (transit + service), and each piece may be traversed in either direction.
"""

from __future__ import annotations

import numpy as np
from scipy.optimize import linear_sum_assignment

from ..trajectory import SERVICE
from .base import HyperParam, Manifest, PlanResult, Stage, StagedPlanner
from .grid_common import (DIRECTION_PARAM, TREE_PARAM, cell_routes_to_trajectories, cell_turn_angles,
                          coarse_component, stc_route, tour_segment_route, tree_layer)


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
    kind = "work_distribution"
    name = "tour_split"
    consumes = ("tour",)
    produces = "routes"

    def run(self, data, rep, ctx, params):
        tour = data["tour"]["cells"]
        mode = params["split"]
        if mode == "nonbacktracking":
            return self._nonbacktracking(rep, tour)
        return self._balanced(rep, tour, params)

    def _nonbacktracking(self, rep, tour):
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

    def _balanced(self, rep, tour, params):
        N = len(tour)
        m = len(rep.team)
        robots = list(rep.team)
        q = N / m
        C = rep.center(np.asarray(tour))
        step = np.hypot(*np.diff(np.r_[C, C[:1]], axis=0).T)  # tour[k] -> tour[k+1]
        turns = np.roll(cell_turn_angles(rep, tour + tour[:2])[1:N + 1], 1)  # turn at tour[k]
        v = np.array([r.speed for r in robots])
        tc = np.array([r.turn_cost for r in robots])
        dist = np.stack([rep.distances(rep.starts[r.id]["cell"])[np.asarray(tour)] for r in robots])  # (m, N)
        csum_len = np.r_[0.0, np.cumsum(np.r_[step, step])]
        csum_turn = np.r_[0.0, np.cumsum(np.r_[turns, turns])]
        best = None
        for off in np.unique(np.floor(np.linspace(0, q, min(int(np.ceil(q)), 24), endpoint=False)).astype(int)):
            bounds = [int(round(off + k * q)) for k in range(m + 1)]
            segs = [(bounds[k], bounds[k + 1]) for k in range(m)]
            cost = np.zeros((m, m))
            rev = np.zeros((m, m), dtype=bool)
            for s, (a, b) in enumerate(segs):
                n = b - a
                if n <= 0:
                    continue
                length = csum_len[b - 1] - csum_len[a]
                turn = csum_turn[b - 1] - csum_turn[a + 1] if n > 2 else 0.0
                i_first, i_last = a % N, (b - 1) % N
                t_fwd = dist[:, i_first] / v
                t_bwd = dist[:, i_last] / v
                rev[:, s] = t_bwd < t_fwd
                cost[:, s] = np.minimum(t_fwd, t_bwd) + length / v + turn * tc
            cost = np.where(np.isfinite(cost), cost, 1e12)
            ri, si = linear_sum_assignment(cost ** 4 / max(cost.max(), 1e-9) ** 3)
            mk = cost[ri, si].max()
            if best is None or mk < best[0]:
                best = (mk, segs, dict(zip(ri, si)), rev)
        _, segs, assign, rev = best
        routes = {}
        for k, r in enumerate(robots):
            s = assign[k]
            a, b = segs[s]
            seg = [tour[i % N] for i in range(a, b)]
            routes[r.id] = tour_segment_route(rep, seg, rep.starts[r.id]["cell"], reverse=bool(rev[k, s]))
        return {"routes": routes, "segments": {r.id: segs[assign[k]][1] - segs[assign[k]][0]
                                               for k, r in enumerate(robots)}, "N": N}

    def metrics(self, out, rep):
        sizes = np.array(list(out["segments"].values()), dtype=float)
        return {"segment_sizes": sizes.astype(int).tolist(),
                "imbalance": float(sizes.max() / sizes.mean()) if sizes.mean() > 0 else None}


class MSTC(StagedPlanner):
    manifest = Manifest(
        name="mstc",
        version="1.0",
        reference="Hazon & Kaminka (2005), Redundancy, efficiency and robustness in multi-robot coverage",
        family="grid",
        adapter="grid",
        adapter_params={"coarse": True, "connectivity": 4},
        constraints={"multi_robot", "deconfliction"},
        mode="staged",
        stages=["coarse_grid", "stc_tour", "tour_split"],
        hyperparameters=[TREE_PARAM, DIRECTION_PARAM,
                         HyperParam("split", "categorical", "nonbacktracking", choices=["nonbacktracking", "balanced"],
                                    doc="non-backtracking (original) or balanced tour split")],
        core_modules=["uninbench/planners/mstc.py", "uninbench/planners/grid_common.py"],
        description="One STC tour over the whole region, split among the robots.",
    )
    stages = [CoarseGridModel(), STCTour(), TourSplit()]

    def finish(self, data, rep, ctx) -> PlanResult:
        routes = data["routes"]["routes"]
        ts = cell_routes_to_trajectories(rep, routes, self.manifest, ctx)
        return PlanResult(ts, native={rid: [c for c, _ in rt] for rid, rt in routes.items()},
                          layers={"spanning_tree": tree_layer(rep, data["tour"]["tree"])})

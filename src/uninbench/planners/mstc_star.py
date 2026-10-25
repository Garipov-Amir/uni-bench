"""MSTC* (Tang, Sun, Zhang & Ma, 2021) -- staged mode.

Balanced-MSTC*: a single Spiral-STC tour is built around the minimum spanning
tree of the terrain graph and cut into ``k`` contiguous partitions, one per
robot.  Starting from the naive cut into partitions of equal node counts,
Balanced-Cut repeatedly takes the heaviest and the lightest partition of the
unfolded tour and shifts the key nodes between them (through all in-between
partitions) by a number of nodes found by binary search, until the maximum
weight no longer decreases.

The weight of a partition for its robot is the cost of travelling from the
robot's depot to the first node of the partition, along the partition, and
back to the depot ("cover and return" in the paper), with shortest paths of
the decomposed graph.  Details the paper leaves open follow the authors'
implementation: partitions are assigned to robots in robot order, the naive
split halves recursively, and the loop is balanced along the shorter way round
between the heaviest and the lightest partition.
"""

from __future__ import annotations

import numpy as np

from ..trajectory import SERVICE, TRANSIT
from .base import Manifest, PlanResult, Stage, StagedPlanner
from .grid_common import cell_routes_to_trajectories, circumnavigation, connect_cells, walk_cycle
from .tree_cover import coarse_graph, coverage_time, forest_layer, kruskal


class TerrainGraph(Stage):
    kind = "environment_model"
    name = "terrain_graph"
    consumes = ("representation",)
    produces = "model"

    def run(self, data, rep, ctx, params):
        return {"graph": coarse_graph(rep)}

    def summary(self, out):
        cg = out["graph"]
        return {"vertices": int(cg.n), "edges": int(len(cg.edges))}


class MSTTour(Stage):
    """Spiral-STC tour around the minimum spanning tree of every component with robots."""

    kind = "route_construction"
    name = "mst_tour"
    consumes = ("model",)
    produces = "tour"

    def run(self, data, rep, ctx, params):
        cg = data["model"]["graph"]
        tree = kruskal(cg.n, cg.edges, cg.w_edge)
        tours, trees = {}, []
        for c in sorted({int(cg.comp[r]) for r in cg.roots}):
            robots = [k for k, r in enumerate(cg.roots) if int(cg.comp[r]) == c]
            verts = [v for v in range(cg.n) if int(cg.comp[v]) == c]
            vset = set(verts)
            tedges = [e for e in tree if int(cg.edges[e, 0]) in vset]
            flat = {int(cg.nodes[v]) for v in verts}
            ft = [(int(cg.nodes[cg.edges[e, 0]]), int(cg.nodes[cg.edges[e, 1]])) for e in tedges]
            first = cg.start_cells[robots[0]]
            cyc = walk_cycle(circumnavigation(rep, flat, ft), first, params.get("direction", "ccw"))
            tours[c] = {"cells": cyc, "robots": robots}
            trees.append(tedges)
        return {"tours": tours, "trees": trees}

    def summary(self, out):
        return {"tour_cells": int(sum(len(t["cells"]) for t in out["tours"].values()))}


class BalancedCut(Stage):
    """Balanced-MSTC* partition of the tour (Algorithms 1 and 2 of the paper)."""

    kind = "work_distribution"
    name = "balanced_cut"
    consumes = ("tour", "model")
    produces = "routes"

    def run(self, data, rep, ctx, params):
        routes, weights, walks = {}, {}, {}
        for c, t in data["tour"]["tours"].items():
            r_idx = t["robots"]
            robots = [rep.team[k] for k in r_idx]
            cells = t["cells"]
            res = balanced_split(rep, cells, robots, True, turn_cost=False,
                                 max_iter=int(params.get("max_iter", 10000)), deadline=lambda: ctx.out_of_time(0.9))
            for r, (a, b), w in zip(res["order"], res["segments"], res["weights"]):
                seg = [cells[i % len(cells)] for i in range(a, b)]
                routes[r.id], walks[r.id] = segment_route(rep, seg, r, ctx.return_to_depot)
                weights[r.id] = w
        cg = data["model"]["graph"]
        return {"routes": routes, "weights": weights,
                "coverage_time": {rid: coverage_time(rep, cg, w) for rid, w in walks.items()}}

    def metrics(self, out, rep):
        w = np.array(list(out["weights"].values()), dtype=float)
        return {"max_weight": float(w.max()) if len(w) else 0.0,
                "imbalance": float(w.max() / w.mean()) if len(w) and w.mean() > 0 else None,
                "coverage_time_max": float(max(out["coverage_time"].values())) if out["coverage_time"] else 0.0}


def segment_route(rep, seg: list[int], robot, return_to_depot: bool):
    """Cell route of a robot covering ``seg`` from its depot, and the closed walk used for the cost."""
    start = int(rep.starts[robot.id]["cell"])
    if not seg:
        return [], [start]
    lead = connect_cells(rep, start, seg[0]) if start != seg[0] else [start]
    route = [(c, TRANSIT) for c in lead[1:-1]]
    if len(lead) > 1:
        route.append((seg[0], SERVICE))
    route += [(c, SERVICE) for c in seg[1:]]
    back = connect_cells(rep, seg[-1], start)[1:] if seg[-1] != start else []
    walk = lead[:-1] + seg + back
    if walk[-1] != start:
        walk.append(start)
    return route, walk


def _halving_split(N: int, K: int) -> list[int]:
    """Node counts of the naive split: recursive halving into near-equal parts."""
    if K == 1:
        return [N]
    left = K // 2
    ln = int(round(N * left / K))
    return _halving_split(ln, left) + _halving_split(N - ln, K - left)


def balanced_split(rep, cells: list[int], robots, return_to_depot: bool = True, turn_cost: bool = False,
                   max_iter: int = 10000, deadline=None) -> dict:
    """Balanced-MSTC* cut of a closed tour ``cells`` (first cell not repeated) among ``robots``.

    Partition ``j`` of the loop belongs to the ``j``-th robot (robot order, as in the authors'
    implementation); the loop starts at the depot of the first robot.  Starting from the naive
    split into near-equal node counts, the heaviest and the lightest partitions are balanced by
    shifting the key nodes between them along the shorter way round the loop, with the shift found
    by binary search (Balanced-Cut), until the maximum weight no longer decreases.

    The weight of a partition is the cost of the robot's "cover and return" walk: depot to the
    first node, along the partition, back to the depot (``return_to_depot=False`` drops the
    return).  With ``turn_cost`` the weight is the robot's time -- length / speed plus the turn
    cost of the heading changes along the partition (TMSTC*).
    Returns {"order": robots, "segments": [(a, b)] (loop indices, taken modulo len(cells)),
    "weights": [...], "iterations": n}.
    """
    N = len(cells)
    k = len(robots)
    C = rep.center(np.asarray(cells))
    d = np.diff(np.r_[C, C[:1]], axis=0)  # step t: cells[t] -> cells[t+1]
    step = np.hypot(d[:, 0], d[:, 1])
    head = np.arctan2(d[:, 1], d[:, 0])
    turn_at = np.abs((head - np.roll(head, 1) + np.pi) % (2 * np.pi) - np.pi)  # turn at cells[t]
    cum = np.r_[0.0, np.cumsum(step)]
    cum2 = np.r_[cum, cum[-1] + cum[1:], 2 * cum[-1] + cum[1:]]
    ct = np.r_[0.0, np.cumsum(turn_at)]
    ct2 = np.r_[ct, ct[-1] + ct[1:], 2 * ct[-1] + ct[1:]]
    dist = np.stack([rep.distances(int(rep.starts[r.id]["cell"]))[np.asarray(cells)] for r in robots])
    dist3 = np.concatenate([dist, dist, dist], axis=1)
    v = np.array([r.speed for r in robots])
    tc = np.array([r.turn_cost for r in robots])
    pos = {c: i for i, c in enumerate(cells)}
    c0 = int(rep.starts[robots[0].id]["cell"])
    base = pos.get(c0, 0)

    def w(j, kk):
        a, b = kk[j], kk[j + 1]
        if b <= a:
            return 0.0
        x = dist3[j, a] + (cum2[b - 1] - cum2[a]) + (dist3[j, b - 1] if return_to_depot else 0.0)
        if not turn_cost:
            return float(x)
        turns = (ct2[b - 1] - ct2[a + 1]) if b - 1 > a + 1 else 0.0
        return float(x / v[j] + tc[j] * turns)

    sizes = _halving_split(N, k)
    keys = [base + int(x) for x in np.r_[0, np.cumsum(sizes)[:-1]]] + [base + N]
    W = [w(j, keys) for j in range(k)]
    it = 0
    while k > 1 and it < max_iter:
        it += 1
        jmax, jmin = int(np.argmax(W)), int(np.argmin(W))
        if W[jmax] - W[jmin] <= 1e-9 * max(1.0, W[jmax]):
            break
        # the heaviest partition hands nodes towards the lightest along the shorter way round
        fwd = (jmin - jmax) % k  # jmax -> jmin following the loop direction
        if fwd <= k - fwd:
            moves = [((jmax + t) % k + 1) for t in range(fwd)]  # right keys of jmax .. before jmin
            sign = -1
        else:
            moves = [((jmax - t) % k) for t in range(k - fwd)]  # left keys of jmax .. after jmin
            sign = 1
        size_max = keys[jmax + 1] - keys[jmax]
        if size_max <= 1:
            break

        def shifted(s):
            kk = list(keys)
            for i in moves:
                if i == k:  # the right key of the last partition is the first key + N
                    kk[0] += sign * s
                    kk[k] += sign * s
                elif i == 0:
                    kk[0] += sign * s
                    kk[k] += sign * s
                else:
                    kk[i] += sign * s
            return kk

        lo, hi = 0, size_max - 1
        while lo < hi:  # binary search: smallest shift with W_min >= W_max
            mid = (lo + hi) // 2
            kk = shifted(mid)
            if w(jmin, kk) < w(jmax, kk):
                lo = mid + 1
            else:
                hi = mid
        best = None
        for s in sorted({lo, max(0, lo - 1)}):
            kk = shifted(s)
            if any(kk[i + 1] - kk[i] < 1 for i in range(k)) or kk[0] < 0:
                continue
            ww = [w(j, kk) for j in range(k)]
            cand = (max(ww), sum(ww))
            if best is None or cand < best[0]:
                best = (cand, kk, ww)
        if best is None or best[0][0] >= max(W) - 1e-9 * max(1.0, max(W)):
            break
        _, keys, W = best
        if keys[0] >= N:  # keep indices in range of the tripled prefix arrays
            keys = [x - N for x in keys]
        if deadline is not None and deadline():
            break
    return {"order": list(robots), "segments": [(keys[j], keys[j + 1]) for j in range(k)], "weights": W,
            "iterations": it}


class MSTCStar(StagedPlanner):
    manifest = Manifest(
        name="mstc_star",
        version="1.0",
        reference="Tang, Sun, Zhang & Ma (2021), MSTC*: Multi-robot coverage path planning under physical "
                  "constraints (Balanced-MSTC*)",
        family="grid",
        adapter="grid",
        adapter_params={"coarse": True, "connectivity": 4},
        constraints={"multi_robot", "return_to_depot"},
        mode="staged",
        stages=["terrain_graph", "mst_tour", "balanced_cut"],
        hyperparameters=[],
        core_modules=["uninbench/planners/mstc_star.py", "uninbench/planners/tree_cover.py",
                      "uninbench/planners/grid_common.py"],
        description="One Spiral-STC tour around the minimum spanning tree, cut into balanced partitions.",
    )
    stages = [TerrainGraph(), MSTTour(), BalancedCut()]

    def finish(self, data, rep, ctx) -> PlanResult:
        routes = data["routes"]["routes"]
        ts = cell_routes_to_trajectories(rep, routes, self.manifest, ctx)
        cg = data["model"]["graph"]
        return PlanResult(ts, native={rid: [c for c, _ in rt] for rid, rt in routes.items()},
                          layers={"spanning_tree": forest_layer(cg, data["tour"]["trees"])},
                          info={"coverage_time": data["routes"]["coverage_time"],
                                "partition_weights": data["routes"]["weights"]})

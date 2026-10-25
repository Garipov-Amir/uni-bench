"""TMSTC* (Lu, Zeng, Tang, Lam & Wen, 2023) -- staged mode.

Turn-minimising MSTC*:

1. **Minimum brick tiling** of the terrain graph: a border between two adjacent
   free coarse cells may be removed (the cells merge); removing two
   perpendicular borders that share an end point would create an L-shape, so
   the removable borders form an independent set of the bipartite conflict
   graph between vertical and horizontal borders.  A maximum independent set
   -- the complement of a minimum vertex cover, obtained from a maximum
   matching by Koenig's theorem -- gives the minimum number of bricks
   (straight bars of cells).
2. **Greedy merge** of the bricks into a spanning tree (Algorithm 1): unused
   edges are taken from a min-heap keyed by the change in the number of turns
   they cause at their end points, ``g(e) = f+(u) + f+(v) - f(u) - f(v)``,
   with lazy re-evaluation of outdated keys.
3. **Improved MSTC*** partition of the STC tour around that tree, with the
   robot's time -- length / speed plus the turn cost of every heading change --
   as the partition weight.

The paper's time model (acceleration, stops at every twist) is replaced by the
benchmark's motion model of the instance, which the evaluator uses as well.
"""

from __future__ import annotations

import heapq
from collections import defaultdict

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import maximum_bipartite_matching

from .base import Manifest, PlanResult, Stage, StagedPlanner
from .grid_common import cell_routes_to_trajectories, circumnavigation, walk_cycle
from .mstc_star import TerrainGraph, balanced_split, segment_route
from .tree_cover import DisjointSet, coverage_time, forest_layer


def minimum_brick_tiling(cg, nx2: int) -> list[int]:
    """Edges of G whose borders are removed in a minimum brick tiling (indices into cg.edges)."""
    E = cg.edges
    flat = cg.nodes
    horiz = np.array([flat[v] - flat[u] == 1 for u, v in E], dtype=bool)  # cells side by side (vertical border)
    L = np.flatnonzero(horiz)  # merge horizontally
    R = np.flatnonzero(~horiz)  # merge vertically
    # border end points in lattice coordinates (x = column, y = row)
    pts = {}
    for e in range(len(E)):
        I, J = divmod(int(flat[E[e, 0]]), nx2)
        if horiz[e]:
            pts[e] = ((J + 1, I), (J + 1, I + 1))
        else:
            pts[e] = ((J, I + 1), (J + 1, I + 1))
    by_point = defaultdict(lambda: ([], []))
    for e, (a, b) in pts.items():
        for q in (a, b):
            by_point[q][0 if horiz[e] else 1].append(e)
    lpos = {int(e): k for k, e in enumerate(L)}
    rpos = {int(e): k for k, e in enumerate(R)}
    rows, cols = [], []
    for ls, rs in by_point.values():
        for a in ls:
            for b in rs:
                rows.append(lpos[a])
                cols.append(rpos[b])
    if not rows:
        return [int(e) for e in range(len(E))]
    G = coo_matrix((np.ones(len(rows)), (rows, cols)), shape=(len(L), len(R))).tocsr()
    G.sum_duplicates()
    G.data[:] = 1
    match_l = maximum_bipartite_matching(G, perm_type="column")  # L -> R or -1
    match_r = np.full(len(R), -1)
    for a, b in enumerate(match_l):
        if b >= 0:
            match_r[b] = a
    # Koenig: Z = vertices reachable from unmatched left vertices by alternating paths
    zl = np.zeros(len(L), dtype=bool)
    zr = np.zeros(len(R), dtype=bool)
    stack = [a for a in range(len(L)) if match_l[a] < 0]
    for a in stack:
        zl[a] = True
    while stack:
        a = stack.pop()
        for b in G.indices[G.indptr[a]:G.indptr[a + 1]]:
            if not zr[b]:
                zr[b] = True
                a2 = match_r[b]
                if a2 >= 0 and not zl[a2]:
                    zl[a2] = True
                    stack.append(a2)
    # minimum vertex cover = (L \\ Z) u (R n Z); maximum independent set = (L n Z) u (R \\ Z)
    return sorted([int(L[a]) for a in range(len(L)) if zl[a]] + [int(R[b]) for b in range(len(R)) if not zr[b]])


def _turns(v: int, nbrs: list[int], flat, nx2: int) -> int:
    """f(rho): turns of the circumnavigation around a tree vertex with the given neighbours."""
    d = len(nbrs)
    if d == 0:
        return 4
    if d == 2:
        a, b = (divmod(int(flat[x]), nx2) for x in nbrs)
        c = divmod(int(flat[v]), nx2)
        straight = (a[0] == b[0] == c[0]) or (a[1] == b[1] == c[1])
        return 0 if straight else 2
    if d in (1, 3):
        return 2
    return 4


def merge_bricks(cg, nx2: int, brick_edges: list[int]) -> list[int]:
    """Algorithm 1 of the paper: greedy spanning tree from the bricks (edge indices)."""
    n = cg.n
    flat = cg.nodes
    ds = DisjointSet(n)
    tree = set()
    nb = defaultdict(list)
    for e in brick_edges:
        u, v = int(cg.edges[e, 0]), int(cg.edges[e, 1])
        ds.union(u, v)
        tree.add(e)
        nb[u].append(v)
        nb[v].append(u)

    def cost(e):
        u, v = int(cg.edges[e, 0]), int(cg.edges[e, 1])
        before = _turns(u, nb[u], flat, nx2) + _turns(v, nb[v], flat, nx2)
        after = _turns(u, nb[u] + [v], flat, nx2) + _turns(v, nb[v] + [u], flat, nx2)
        return after - before

    heap = [(cost(e), e) for e in range(len(cg.edges)) if e not in tree]
    heapq.heapify(heap)
    while heap:
        c, e = heapq.heappop(heap)
        u, v = int(cg.edges[e, 0]), int(cg.edges[e, 1])
        if ds.find(u) == ds.find(v):
            continue
        c2 = cost(e)
        if c2 == c:
            ds.union(u, v)
            tree.add(e)
            nb[u].append(v)
            nb[v].append(u)
        else:
            heapq.heappush(heap, (c2, e))
    return sorted(tree)


class BrickTree(Stage):
    kind = "route_construction"
    name = "brick_tree"
    consumes = ("model",)
    produces = "tour"

    def run(self, data, rep, ctx, params):
        cg = data["model"]["graph"]
        nx2 = rep.coarse["grid"].nx
        bricks = minimum_brick_tiling(cg, nx2)
        tree = merge_bricks(cg, nx2, bricks)
        tours, trees = {}, []
        for c in sorted({int(cg.comp[r]) for r in cg.roots}):
            robots = [k for k, r in enumerate(cg.roots) if int(cg.comp[r]) == c]
            verts = {v for v in range(cg.n) if int(cg.comp[v]) == c}
            tedges = [e for e in tree if int(cg.edges[e, 0]) in verts]
            flat = {int(cg.nodes[v]) for v in verts}
            ft = [(int(cg.nodes[cg.edges[e, 0]]), int(cg.nodes[cg.edges[e, 1]])) for e in tedges]
            cyc = walk_cycle(circumnavigation(rep, flat, ft), cg.start_cells[robots[0]], "ccw")
            tours[c] = {"cells": cyc, "robots": robots}
            trees.append(tedges)
        n_bricks = cg.n - len(bricks)
        return {"tours": tours, "trees": trees, "bricks": n_bricks}

    def summary(self, out):
        return {"bricks": int(out["bricks"]), "tour_cells": int(sum(len(t["cells"]) for t in out["tours"].values()))}

    def metrics(self, out, rep):
        return {"bricks": float(out["bricks"])}


class TurnAwareCut(Stage):
    kind = "work_distribution"
    name = "turn_aware_cut"
    consumes = ("tour", "model")
    produces = "routes"

    def run(self, data, rep, ctx, params):
        routes, weights, walks = {}, {}, {}
        for c, t in data["tour"]["tours"].items():
            robots = [rep.team[k] for k in t["robots"]]
            cells = t["cells"]
            res = balanced_split(rep, cells, robots, ctx.return_to_depot, turn_cost=True,
                                 deadline=lambda: ctx.out_of_time(0.9))
            for r, (a, b), w in zip(res["order"], res["segments"], res["weights"]):
                seg = [cells[i % len(cells)] for i in range(a, b)]
                routes[r.id], walks[r.id] = segment_route(rep, seg, r, ctx.return_to_depot)
                weights[r.id] = w
        cg = data["model"]["graph"]
        return {"routes": routes, "weights": weights,
                "coverage_time": {rid: coverage_time(rep, cg, w) for rid, w in walks.items()}}

    def metrics(self, out, rep):
        w = np.array(list(out["weights"].values()), dtype=float)
        return {"max_time": float(w.max()) if len(w) else 0.0,
                "imbalance": float(w.max() / w.mean()) if len(w) and w.mean() > 0 else None}


class TMSTCStar(StagedPlanner):
    manifest = Manifest(
        name="tmstc_star",
        version="1.0",
        reference="Lu, Zeng, Tang, Lam & Wen (2023), TMSTC*: A turn-minimizing algorithm for multi-robot "
                  "coverage path planning",
        family="grid",
        adapter="grid",
        adapter_params={"coarse": True, "connectivity": 4},
        constraints={"multi_robot", "return_to_depot"},
        mode="staged",
        stages=["terrain_graph", "brick_tree", "turn_aware_cut"],
        hyperparameters=[],
        core_modules=["uninbench/planners/tmstc_star.py", "uninbench/planners/mstc_star.py",
                      "uninbench/planners/tree_cover.py", "uninbench/planners/grid_common.py"],
        description="Minimum brick tiling, greedy turn-aware spanning tree, MSTC* split with turn costs.",
    )
    stages = [TerrainGraph(), BrickTree(), TurnAwareCut()]

    def finish(self, data, rep, ctx) -> PlanResult:
        routes = data["routes"]["routes"]
        ts = cell_routes_to_trajectories(rep, routes, self.manifest, ctx)
        cg = data["model"]["graph"]
        return PlanResult(ts, native={rid: [c for c, _ in rt] for rid, rt in routes.items()},
                          layers={"spanning_tree": forest_layer(cg, data["tour"]["trees"])},
                          info={"coverage_time": data["routes"]["coverage_time"], "bricks": data["tour"]["bricks"]})

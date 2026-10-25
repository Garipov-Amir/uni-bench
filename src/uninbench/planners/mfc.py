"""Multi-Robot Forest Coverage, MFC (Zheng, Koenig, Kempe & Jain, 2010) -- staged mode.

MFC computes a rooted min-max tree cover of the terrain graph with the
Rooted-Tree-Cover algorithm of Even, Garg, Koenemann, Ravi & Sinha (2004) and
lets every robot circumnavigate its tree with STC.  For a bound ``B``:

1. remove the edges heavier than ``B``;
2. contract all roots into one vertex, compute a minimum spanning tree and
   uncontract it into a forest with one tree per root;
3. edge-decompose every tree into subtrees of weight in ``[B, 2B)`` and a
   leftover tree of weight ``< B`` that contains the root;
4. match the subtrees to roots within distance ``B`` (maximum bipartite
   matching); if some subtree stays unmatched, ``B`` is infeasible;
5. a root's tree is its leftover tree, its matched subtree and the shortest
   path connecting them (weight at most ``4B``).

``B`` is found by bisection; the best cover by its actual largest tree weight
is kept.  Edge weights are the mean terrain weights of their end points.
"""

from __future__ import annotations

import math
from collections import defaultdict

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import dijkstra, maximum_bipartite_matching

from .base import Manifest, PlanResult, Stage, StagedPlanner
from .grid_common import cell_routes_to_trajectories
from .mstc_star import TerrainGraph
from .tree_cover import (DisjointSet, coverage_time, forest_layer, kruskal, open_walk, routes_from_walks,
                         tree_tour)


# ------------------------------------------------------------------ rooted tree cover
def _mst_edges(n, edges, weights, allowed_vertices=None):
    ok = None
    if allowed_vertices is not None:
        ok = np.array([u in allowed_vertices and v in allowed_vertices for u, v in edges], dtype=bool)
    return kruskal(n, edges, weights, ok)


def _edge_decomposition(tree_adj: dict, root: int, B: float, wfun):
    """Split a tree (adjacency dict of the vertices of one forest tree) into subtrees of weight
    in [B, 2B) and a leftover tree containing ``root`` with weight < B.

    Returns (list of (vertices, edges), (leftover vertices, leftover edges)) with edges as
    vertex pairs."""
    parent = {root: None}
    order = [root]
    for u in order:
        for v in tree_adj[u]:
            if v not in parent:
                parent[v] = u
                order.append(v)
    children = defaultdict(list)
    for v in order[1:]:
        children[parent[v]].append(v)
    alive = {v: True for v in order}
    sub_w = {}
    pieces = []

    def weight_of(v):
        return sub_w[v]

    for v in reversed(order):  # bottom-up
        while True:
            kids = [c for c in children[v] if alive[c]]
            hang = {c: weight_of(c) + wfun(v, c) for c in kids}
            total = sum(hang.values())
            if total < B:
                sub_w[v] = total
                break
            group, acc = [], 0.0
            for c in sorted(kids, key=lambda c: -hang[c]):
                group.append(c)
                acc += hang[c]
                if acc >= B:
                    break
            verts, edges = [v], []
            for c in group:
                stack = [c]
                edges.append((v, c))
                while stack:
                    x = stack.pop()
                    verts.append(x)
                    for y in children[x]:
                        if alive[y]:
                            edges.append((x, y))
                            stack.append(y)
                    alive[x] = False
            pieces.append((verts, edges))
    left_v, left_e, stack = [root], [], [root]
    while stack:
        x = stack.pop()
        for y in children[x]:
            if alive[y]:
                left_v.append(y)
                left_e.append((x, y))
                stack.append(y)
    return pieces, (left_v, left_e)


def rooted_tree_cover(n: int, edges: np.ndarray, w: np.ndarray, roots: list[int], B: float):
    """One Rooted-Tree-Cover attempt with bound ``B``; roots must be distinct.

    Returns a list (per root) of (vertex set, list of edge indices) or None if ``B`` is too small."""
    keep = w <= B + 1e-12
    E = edges[keep]
    wE = w[keep]
    eidx = np.flatnonzero(keep)
    if len(E):
        M = coo_matrix((np.r_[wE, wE] + 1e-300, (np.r_[E[:, 0], E[:, 1]], np.r_[E[:, 1], E[:, 0]])), shape=(n, n))
        M = M.tocsr()
    else:
        M = coo_matrix((n, n)).tocsr()
    dist, pred = dijkstra(M, directed=False, indices=roots, return_predecessors=True)
    if not np.all(np.isfinite(dist.min(axis=0))):
        return None  # a vertex cannot be reached from any root without heavy edges
    # 2) MST of the graph with contracted roots: roots share one disjoint-set class
    rootset = set(roots)
    ds = DisjointSet(n)
    for r in roots[1:]:
        ds.union(r, roots[0])
    order = np.argsort(wE, kind="stable")
    forest_e = []
    for e in order:
        u, v = int(E[e, 0]), int(E[e, 1])
        if u in rootset and v in rootset:
            continue
        if ds.union(u, v):
            forest_e.append(int(e))
    adj = defaultdict(list)
    wmap = {}
    for e in forest_e:
        u, v = int(E[e, 0]), int(E[e, 1])
        adj[u].append(v)
        adj[v].append(u)
        wmap[(u, v)] = wmap[(v, u)] = float(wE[e])
    for r in roots:
        adj.setdefault(r, [])
    # every forest tree contains exactly one root; restrict adjacency per tree through BFS from roots
    pieces, leftovers = [], {}
    for r in roots:
        comp_adj = {}
        stack, seen = [r], {r}
        while stack:
            x = stack.pop()
            comp_adj[x] = [y for y in adj[x] if not (y in rootset and y != r)]
            for y in comp_adj[x]:
                if y not in seen:
                    seen.add(y)
                    stack.append(y)
        P, L = _edge_decomposition(comp_adj, r, B, lambda a, b: wmap[(a, b)])
        pieces += P
        leftovers[r] = L
    if len(pieces) > len(roots):
        return None
    # 4) bipartite matching of subtrees to roots within distance B
    rows, cols = [], []
    near = {}
    for s, (verts, _) in enumerate(pieces):
        for j, r in enumerate(roots):
            dv = dist[j, verts]
            k = int(np.argmin(dv))
            if dv[k] <= B + 1e-12:
                rows.append(s)
                cols.append(j)
                near[(s, j)] = verts[k]
    if pieces:
        G = coo_matrix((np.ones(len(rows)), (rows, cols)), shape=(len(pieces), len(roots))).tocsr()
        match = maximum_bipartite_matching(G, perm_type="column")
        if np.any(match < 0):
            return None
    else:
        match = np.zeros(0, dtype=int)
    pair_edge = {}
    for e, (u, v) in enumerate(E):
        pair_edge[(int(u), int(v))] = pair_edge[(int(v), int(u))] = int(eidx[e])
    out = []
    for j, r in enumerate(roots):
        verts = set(leftovers[r][0])
        epairs = list(leftovers[r][1])
        for s in np.flatnonzero(match == j):
            verts |= set(pieces[s][0])
            epairs += pieces[s][1]
            x = near[(int(s), j)]
            while x != r and x >= 0:  # 5) shortest path from the root to the subtree
                p = int(pred[j, x])
                epairs.append((p, x))
                verts.add(p)
                x = p
        idx = sorted({pair_edge[(int(a), int(b))] for a, b in epairs})
        # the union may contain cycles: keep a minimum spanning tree of it
        idx = [idx[i] for i in kruskal(n, edges[idx], w[idx])] if idx else []
        out.append((verts, idx))
    return out


def tree_weight(w, idx) -> float:
    return float(np.sum(w[idx])) if len(idx) else 0.0


def rtc_min_max(n: int, edges: np.ndarray, w: np.ndarray, roots: list[int], iters: int = 40, deadline=None):
    """Rooted min-max tree cover by bisection on B (MFC).  ``roots`` may repeat: robots sharing
    a root vertex are attached to zero-weight dummy vertices, as in the reference implementation.
    Returns (trees per robot as (vertex set, edge indices), largest tree weight, B)."""
    # dummy vertices for repeated roots
    uniq, dummy_of = [], {}
    extra_edges, extra_w = [], []
    nn = n
    for k, r in enumerate(roots):
        if r in uniq or r in dummy_of.values():
            dummy_of[nn] = r
            extra_edges.append((min(r, nn), max(r, nn)))
            extra_w.append(0.0)
            uniq.append(nn)
            nn += 1
        else:
            uniq.append(r)
    E = np.vstack([edges, np.array(extra_edges, dtype=int).reshape(-1, 2)]) if extra_edges else edges
    W = np.r_[w, np.array(extra_w)] if extra_w else w
    pos_w = W[W > 0]
    unit = float(pos_w.min()) if len(pos_w) else 1.0
    integral = bool(np.allclose(W / unit, np.round(W / unit)))
    lo = 0.0
    hi = float(W.sum()) + unit
    best = None
    for _ in range(iters):
        if hi - lo <= (unit if integral else 1e-6 * max(hi, 1.0)):
            break
        mid = 0.5 * (lo + hi)
        if integral:
            mid = math.floor(mid / unit) * unit
            if mid <= lo:
                mid = lo + unit
        res = rooted_tree_cover(nn, E, W, uniq, mid)
        if res is None:
            lo = mid
        else:
            mx = max(tree_weight(W, t[1]) for t in res)
            if best is None or mx < best[1]:
                best = (res, mx, mid)
            hi = mid
        if deadline is not None and deadline() and best is not None:
            break
    if best is None:
        res = rooted_tree_cover(nn, E, W, uniq, hi)
        best = (res, max(tree_weight(W, t[1]) for t in res), hi)
    trees = []
    ne = len(edges)
    for verts, idx in best[0]:
        verts = {dummy_of.get(v, v) for v in verts}
        idx = [i for i in idx if i < ne]
        trees.append((verts, idx))
    return trees, best[1], best[2]


# ------------------------------------------------------------------ stages
class RootedTreeCover(Stage):
    """Rooted min-max tree cover of G, one tree per robot (per component with robots)."""

    kind = "work_distribution"
    name = "rooted_tree_cover"
    consumes = ("model",)
    produces = "cover"

    def run(self, data, rep, ctx, params):
        cg = data["model"]["graph"]
        return tree_cover_by_component(cg, lambda n, E, w, roots, verts: rtc_min_max(
            n, E, w, roots, deadline=lambda: ctx.out_of_time(0.8))[0])

    def metrics(self, out, rep):
        return {"max_tree_weight": out["max_weight"], "trees": len(out["trees"])}


def tree_cover_by_component(cg, solver):
    """Run a rooted tree cover ``solver(n, edges, w, roots, verts) -> [(verts, edge idx)]`` on every
    component of G that contains robots (``verts``: the component's vertices in ``cg``); returns
    trees indexed like ``cg.roots``."""
    trees = [None] * len(cg.roots)
    for c in sorted({int(cg.comp[r]) for r in cg.roots}):
        verts = np.flatnonzero(cg.comp == c)
        loc = {int(v): k for k, v in enumerate(verts)}
        emask = np.array([int(u) in loc for u, _ in cg.edges], dtype=bool)
        eidx = np.flatnonzero(emask)
        E = np.array([[loc[int(u)], loc[int(v)]] for u, v in cg.edges[emask]], dtype=int).reshape(-1, 2)
        robots = [k for k, r in enumerate(cg.roots) if int(cg.comp[r]) == c]
        res = solver(len(verts), E, cg.w_edge[emask], [loc[cg.roots[k]] for k in robots], verts)
        for k, (tv, te) in zip(robots, res):
            trees[k] = ({int(verts[v]) for v in tv} | {cg.roots[k]}, [int(eidx[e]) for e in te])
    mx = max(float(np.sum(cg.w_edge[t[1]])) if t[1] else 0.0 for t in trees) if trees else 0.0
    return {"trees": trees, "max_weight": mx}


class STCPerTree(Stage):
    kind = "route_construction"
    name = "stc_per_tree"
    consumes = ("cover", "model")
    produces = "routes"

    def run(self, data, rep, ctx, params):
        cg = data["model"]["graph"]
        walks, ct = {}, {}
        for k, (verts, tedges) in enumerate(data["cover"]["trees"]):
            rid, start = cg.robot_ids[k], cg.start_cells[k]
            walk = tree_tour(rep, cg, verts, tedges, start, params.get("direction", "ccw"))
            ct[rid] = coverage_time(rep, cg, walk)
            walks[rid] = open_walk(walk, ctx.return_to_depot)
        return {"routes": routes_from_walks(walks), "coverage_time": ct}


class _TreeCoverPlanner(StagedPlanner):
    def finish(self, data, rep, ctx) -> PlanResult:
        routes = data["routes"]["routes"]
        ts = cell_routes_to_trajectories(rep, routes, self.manifest, ctx)
        cg = data["model"]["graph"]
        cover = data["cover"]
        return PlanResult(ts, native={rid: [c for c, _ in rt] for rid, rt in routes.items()},
                          layers={"spanning_tree": forest_layer(cg, [t[1] for t in cover["trees"]])},
                          info={"coverage_time": data["routes"]["coverage_time"],
                                "max_tree_weight": cover["max_weight"],
                                **{k: v for k, v in cover.items() if k not in ("trees", "max_weight")}})


class MFC(_TreeCoverPlanner):
    manifest = Manifest(
        name="mfc",
        version="1.0",
        reference="Zheng, Koenig, Kempe & Jain (2010), Multirobot forest coverage for weighted and unweighted "
                  "terrain (rooted tree cover of Even et al., 2004)",
        family="grid",
        adapter="grid",
        adapter_params={"coarse": True, "connectivity": 4},
        constraints={"multi_robot", "return_to_depot"},
        mode="staged",
        stages=["terrain_graph", "rooted_tree_cover", "stc_per_tree"],
        hyperparameters=[],
        core_modules=["uninbench/planners/mfc.py", "uninbench/planners/tree_cover.py",
                      "uninbench/planners/grid_common.py"],
        description="Rooted min-max tree cover (4-approximation per bound) and STC around every tree.",
    )
    stages = [TerrainGraph(), RootedTreeCover(), STCPerTree()]

"""Shared machinery of grid-based planners: spanning trees, STC circumnavigation,
tour splitting, and conversion of cell routes into the common trajectory format."""

from __future__ import annotations

import math
from collections import defaultdict, deque

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components

from ..trajectory import SERVICE, TRANSIT, TrajectoryBuilder, TrajectorySet
from ..util import pack_array
from .base import HyperParam

TREE_KINDS = ["dfs", "bfs", "horizontal", "vertical"]
TREE_PARAM = HyperParam("tree", "categorical", "dfs", choices=TREE_KINDS,
                        doc="spanning-tree construction on the coarse grid")
DIRECTION_PARAM = HyperParam("direction", "categorical", "ccw", choices=["ccw", "cw"],
                             doc="circumnavigation direction around the tree")


# ------------------------------------------------------------------ coarse graph
def coarse_adjacency(rep) -> dict[int, list[int]]:
    adj: dict[int, list[int]] = defaultdict(list)
    for a, b in rep.coarse["edges"]:
        adj[int(a)].append(int(b))
        adj[int(b)].append(int(a))
    return adj


def coarse_component(rep, root: int) -> set[int]:
    adj = coarse_adjacency(rep)
    seen = {root}
    dq = deque([root])
    while dq:
        u = dq.popleft()
        for v in adj[u]:
            if v not in seen:
                seen.add(v)
                dq.append(v)
    return seen


def spanning_tree(nodes: set[int], adj: dict[int, list[int]], root: int, kind: str, nx2: int,
                  rng: np.random.Generator | None = None) -> list[tuple[int, int]]:
    """Spanning tree of the coarse graph restricted to ``nodes``."""
    nodes = set(nodes)
    if kind in ("dfs", "bfs"):
        # neighbour order: right, up, left, down (counter-clockwise scan as in STC)
        def order(u):
            nb = [v for v in adj[u] if v in nodes]
            key = {u + 1: 0, u + nx2: 1, u - 1: 2, u - nx2: 3}
            return sorted(nb, key=lambda v: key.get(v, 4))

        seen = {root}
        edges = []
        if kind == "bfs":
            dq = deque([root])
            while dq:
                u = dq.popleft()
                for v in order(u):
                    if v not in seen:
                        seen.add(v)
                        edges.append((u, v))
                        dq.append(v)
        else:
            stack = [(root, iter(order(root)))]
            while stack:
                u, it = stack[-1]
                for v in it:
                    if v not in seen:
                        seen.add(v)
                        edges.append((u, v))
                        stack.append((v, iter(order(v))))
                        break
                else:
                    stack.pop()
        return edges
    # Kruskal with orientation-dependent (or random) weights
    cand = []
    for u in nodes:
        for v in adj[u]:
            if v in nodes and u < v:
                horizontal = v == u + 1
                if kind == "horizontal":
                    w = 1.0 if horizontal else 2.0
                elif kind == "vertical":
                    w = 2.0 if horizontal else 1.0
                elif kind == "random":
                    w = float(rng.random()) if rng is not None else 0.0
                else:
                    raise ValueError(f"unknown tree kind {kind!r}")
                cand.append((w, u, v))
    cand.sort()
    parent = {u: u for u in nodes}

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    edges = []
    for _, u, v in cand:
        ru, rv = find(u), find(v)
        if ru != rv:
            parent[ru] = rv
            edges.append((u, v))
    return edges


def circumnavigation(rep, nodes: set[int], tree: list[tuple[int, int]]) -> dict[int, list[int]]:
    """Sub-cell adjacency of the closed path around a spanning tree (STC).

    Every sub-cell of every coarse cell in ``nodes`` gets degree two and the
    sub-cells form a single Hamiltonian cycle.
    """
    nx2 = rep.coarse["grid"].nx
    tnb: dict[int, set] = defaultdict(set)
    for a, b in tree:
        tnb[a].add(b)
        tnb[b].add(a)
    adj: dict[int, list[int]] = defaultdict(list)

    def link(u, v):
        adj[int(u)].append(int(v))
        adj[int(v)].append(int(u))

    for c in nodes:
        I, J = divmod(c, nx2)
        bl, br, tl, tr = rep.sub_cells(I, J)
        if (c + nx2) not in tnb[c]:
            link(tl, tr)
        if not (J + 1 < nx2 and (c + 1) in tnb[c]):
            link(tr, br)
        if (c - nx2) not in tnb[c]:
            link(br, bl)
        if not (J > 0 and (c - 1) in tnb[c]):
            link(bl, tl)
    for a, b in tree:
        a, b = min(a, b), max(a, b)
        A = rep.sub_cells(*divmod(a, nx2))
        B = rep.sub_cells(*divmod(b, nx2))
        if b == a + 1:
            link(A[3], B[2])
            link(A[1], B[0])
        elif b == a + nx2:
            link(A[2], B[0])
            link(A[3], B[1])
        else:
            raise ValueError("tree edge between non-adjacent coarse cells")
    return adj


def walk_cycle(adj: dict[int, list[int]], start: int, direction: str = "ccw") -> list[int]:
    if start not in adj:
        return [start]
    nb = sorted(adj[start])
    nxt = nb[0] if direction == "ccw" else nb[-1]
    out = [start]
    prev, cur = start, nxt
    while cur != start:
        out.append(cur)
        a, b = adj[cur]
        prev, cur = cur, (b if a == prev else a)
        if len(out) > 4 * len(adj) + 4:
            raise RuntimeError("STC circumnavigation is not a single cycle")
    return out


def stc_route(rep, nodes: set[int], start_cell: int, tree_kind: str, direction: str, rng=None,
              close: bool = False) -> tuple[list[int], list[tuple[int, int]]]:
    """STC tour over ``nodes`` (coarse cells) starting at sub-cell ``start_cell``."""
    nx2 = rep.coarse["grid"].nx
    root = int(np.ravel_multi_index(rep.coarse_of(start_cell), (rep.coarse["grid"].ny, nx2)))
    if root not in nodes:
        raise ValueError("start sub-cell is not inside the coarse region")
    tree = spanning_tree(nodes, coarse_adjacency(rep), root, tree_kind, nx2, rng)
    cyc = walk_cycle(circumnavigation(rep, nodes, tree), start_cell, direction)
    if close and len(cyc) > 1:
        cyc = cyc + [start_cell]
    return cyc, tree


# ------------------------------------------------------------------ cell routes
def cell_turn_angles(rep, cells: list[int]) -> np.ndarray:
    """Absolute heading change at every interior cell of a cell route."""
    if len(cells) < 3:
        return np.zeros(len(cells))
    C = rep.center(np.asarray(cells))
    d = np.diff(C, axis=0)
    h = np.arctan2(d[:, 1], d[:, 0])
    dh = np.abs((np.diff(h) + np.pi) % (2 * np.pi) - np.pi)
    return np.r_[0.0, dh, 0.0]


def route_time(rep, cells: list[int], robot) -> float:
    if len(cells) < 2:
        return 0.0
    C = rep.center(np.asarray(cells))
    L = float(np.hypot(*np.diff(C, axis=0).T).sum())
    return L / robot.speed + float(cell_turn_angles(rep, cells).sum()) * robot.turn_cost


def connect_cells(rep, a: int, b: int) -> list[int]:
    """Shortest cell path a -> b (inclusive) on the representation graph."""
    p = rep.path(a, b)
    return p if p else [a, b]


def cell_routes_to_trajectories(rep, routes: dict, manifest, ctx) -> TrajectorySet:
    """Convert native cell routes into the common trajectory format.

    ``routes[rid]`` is a list of ``(cell, label)``: the robot moves from the
    previous cell centre to ``cell``'s centre with that label.  The adapter's
    start connection (start pose -> first cell) is prepended as transit.
    """
    ts = TrajectorySet(metadata={"planner": manifest.name})
    for r in rep.team:
        sx, sy, spsi = r.start
        b = TrajectoryBuilder(r.id, sx, sy, spsi, tool_active_in_turns=manifest.tool_active_in_turns)
        route = routes.get(r.id, [])
        st = rep.starts.get(r.id)
        if st is not None:
            b.follow(st["path"], TRANSIT)
            cur = st["cell"]
        else:
            cur = None
        for cell, label in route:
            if cur is not None and cell == cur and len(b.labels) == 0:
                continue
            x, y = rep.center(cell)
            b.move_to(float(x), float(y), label)
            cur = cell
        if ctx.return_to_depot:
            depot = r.depot if r.depot is not None else (sx, sy)
            if st is not None and cur is not None and cur != st["cell"]:
                for c in connect_cells(rep, cur, st["cell"])[1:]:
                    x, y = rep.center(c)
                    b.move_to(float(x), float(y), TRANSIT)
            if st is not None and np.hypot(depot[0] - sx, depot[1] - sy) < 1e-9:
                back = np.asarray(st["path"], dtype=float)[::-1]  # the adapter's start connection, reversed
            else:
                back = rep.transit(r).path(b.pos, depot)
            b.follow(back, TRANSIT)
        ts.add(b.build())
    return ts


def tour_segment_route(rep, tour: list[int], start_cell: int, reverse: bool = False) -> list[tuple[int, str]]:
    """Route: transit from start_cell to the first tour cell, then service along the tour."""
    seq = tour[::-1] if reverse else tour
    out: list[tuple[int, str]] = []
    if not seq:
        return out
    lead = connect_cells(rep, start_cell, seq[0]) if start_cell != seq[0] else [seq[0]]
    for c in lead[1:-1]:
        out.append((c, TRANSIT))
    if len(lead) > 1:
        out.append((seq[0], SERVICE))
    for c in seq[1:]:
        out.append((c, SERVICE))
    return out


# ------------------------------------------------------------------ partitions
def partition_metrics(labels: np.ndarray, m: int, adj_matrix=None) -> dict:
    """Balance and connectivity of a partition (evaluated before any routing)."""
    sizes = np.bincount(labels[labels >= 0], minlength=m).astype(float)[:m]
    mean = sizes.mean() if m else 0.0
    out = {"sizes": sizes.astype(int).tolist(),
           "imbalance": float(sizes.max() / mean) if mean > 0 else math.nan,
           "cv": float(sizes.std() / mean) if mean > 0 else math.nan}
    if adj_matrix is not None:
        comps = []
        for r in range(m):
            idx = np.flatnonzero(labels == r)
            if len(idx) == 0:
                comps.append(0)
                continue
            sub = adj_matrix[idx][:, idx]
            comps.append(int(connected_components(sub, directed=False)[0]))
        out["components"] = comps
        out["connected"] = all(c <= 1 for c in comps)
    return out


def coarse_matrix(rep, nodes: np.ndarray):
    """Sparse adjacency of the coarse graph restricted to ``nodes`` (in node order)."""
    pos = {int(n): k for k, n in enumerate(nodes)}
    rows, cols = [], []
    for a, b in rep.coarse["edges"]:
        if int(a) in pos and int(b) in pos:
            rows += [pos[int(a)], pos[int(b)]]
            cols += [pos[int(b)], pos[int(a)]]
    N = len(nodes)
    return coo_matrix((np.ones(len(rows)), (rows, cols)), shape=(N, N)).tocsr()


def partition_layer(rep, nodes: np.ndarray, labels: np.ndarray) -> dict:
    g = rep.coarse["grid"]
    lab = np.full(g.shape, -1, dtype=np.int16)
    lab.ravel()[nodes] = labels
    return {"type": "labels", "grid": g.to_dict(), "labels": pack_array(lab)}


def tree_layer(rep, tree: list[tuple[int, int]]) -> dict:
    g = rep.coarse["grid"]
    segs = []
    for a, b in tree:
        ia, ja = divmod(a, g.nx)
        ib, jb = divmod(b, g.nx)
        xa, ya = g.center(ia, ja)
        xb, yb = g.center(ib, jb)
        segs.append([[round(float(xa), 4), round(float(ya), 4)], [round(float(xb), 4), round(float(yb), 4)]])
    return {"type": "segments", "coords": segs}

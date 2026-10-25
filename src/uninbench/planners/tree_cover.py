"""Shared machinery of the tree-based planners (MFC, MSTC*, TMSTC*, MIP, LS-MCPP).

Terminology follows Tang & Ma (2023): the terrain graph ``G`` has one vertex
per coarse cell (2x2 sub-cells of the grid adapter), the decomposed graph
``H`` (``D`` in LS-MCPP) has one vertex per sub-cell.  A vertex of ``G`` with
terrain weight ``w`` gives each of its four sub-cells the weight ``w / 4``; an
edge weighs the mean of its end-point weights.  The *coverage time* of a
closed walk in ``H`` is the sum of the weights of the vertices it visits
(repetitions included), which for STC around a tree with ``n`` unit-weight
vertices equals ``n`` -- the unit used in the published tables.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components, dijkstra

from ..trajectory import SERVICE
from .grid_common import circumnavigation, walk_cycle


@dataclass
class CoarseGraph:
    """The terrain graph G restricted to the components that contain a robot."""

    nodes: np.ndarray  # flat coarse-grid indices, sorted
    edges: np.ndarray  # (E, 2) local vertex indices, u < v
    w_vertex: np.ndarray  # (N,) terrain weight of every vertex
    w_edge: np.ndarray  # (E,) mean of the end-point weights
    centers: np.ndarray  # (N, 2) coordinates of the coarse-cell centres
    roots: list[int]  # local root vertex of every robot (coarse cell of its start sub-cell)
    robot_ids: list[str]
    start_cells: list[int]  # start sub-cell of every robot
    comp: np.ndarray  # component label of every vertex
    index: dict = field(default_factory=dict)  # flat -> local

    @property
    def n(self) -> int:
        return len(self.nodes)

    def adjacency(self) -> list[list[tuple[int, int]]]:
        """adj[u] = [(v, edge index)]"""
        adj: list[list[tuple[int, int]]] = [[] for _ in range(self.n)]
        for e, (u, v) in enumerate(self.edges):
            adj[u].append((int(v), e))
            adj[v].append((int(u), e))
        return adj

    def matrix(self, weights: np.ndarray | None = None):
        w = self.w_edge if weights is None else weights
        u, v = self.edges[:, 0], self.edges[:, 1]
        return coo_matrix((np.r_[w, w], (np.r_[u, v], np.r_[v, u])), shape=(self.n, self.n)).tocsr()

    def distances(self, sources) -> np.ndarray:
        return dijkstra(self.matrix(), directed=False, indices=list(sources))


def coarse_graph(rep) -> CoarseGraph:
    """Terrain graph of the coarse cells in every component that contains a robot start."""
    g = rep.coarse["grid"]
    cedges = np.asarray(rep.coarse["edges"], dtype=int).reshape(-1, 2)
    allnodes = np.flatnonzero(np.asarray(rep.coarse["cells"]).ravel())
    pos_all = {int(n): k for k, n in enumerate(allnodes)}
    N = len(allnodes)
    if len(cedges):
        a = np.array([pos_all[int(x)] for x in cedges[:, 0]])
        b = np.array([pos_all[int(x)] for x in cedges[:, 1]])
        M = coo_matrix((np.ones(len(a)), (a, b)), shape=(N, N))
        _, lab = connected_components(M, directed=False)
    else:
        lab = np.arange(N)
    roots_flat, rids, starts = [], [], []
    for r in rep.team:
        cell = int(rep.starts[r.id]["cell"])
        I, J = rep.coarse_of(cell)
        roots_flat.append(int(I) * g.nx + int(J))
        rids.append(r.id)
        starts.append(cell)
    keep_comps = {int(lab[pos_all[f]]) for f in roots_flat}
    nodes = np.array(sorted(int(n) for n in allnodes if int(lab[pos_all[int(n)]]) in keep_comps), dtype=int)
    index = {int(n): k for k, n in enumerate(nodes)}
    E = [(index[int(u)], index[int(v)]) for u, v in cedges if int(u) in index and int(v) in index]
    E = np.array(sorted((min(u, v), max(u, v)) for u, v in E), dtype=int).reshape(-1, 2)
    W = np.asarray(rep.coarse.get("weights", np.ones(rep.coarse["cells"].shape))).ravel()[nodes].astype(float)
    wE = 0.5 * (W[E[:, 0]] + W[E[:, 1]]) if len(E) else np.zeros(0)
    ci, cj = np.divmod(nodes, g.nx)
    cx, cy = g.center(ci, cj)
    comp = np.array([lab[pos_all[int(n)]] for n in nodes])
    return CoarseGraph(nodes, E, W, wE, np.c_[cx, cy], [index[f] for f in roots_flat], rids, starts, comp, index)


# ------------------------------------------------------------------ trees
class DisjointSet:
    def __init__(self, n: int):
        self.p = list(range(n))

    def find(self, a: int) -> int:
        while self.p[a] != a:
            self.p[a] = self.p[self.p[a]]
            a = self.p[a]
        return a

    def union(self, a: int, b: int) -> bool:
        ra, rb = self.find(a), self.find(b)
        if ra == rb:
            return False
        self.p[ra] = rb
        return True


def kruskal(n: int, edges: np.ndarray, weights: np.ndarray, allowed=None) -> list[int]:
    """Minimum spanning forest; ties are broken by edge order (stable).  Returns edge indices."""
    order = np.argsort(weights, kind="stable")
    ds = DisjointSet(n)
    out = []
    for e in order:
        if allowed is not None and not allowed[e]:
            continue
        u, v = int(edges[e, 0]), int(edges[e, 1])
        if ds.union(u, v):
            out.append(int(e))
    return out


def tree_tour(rep, cg: CoarseGraph, vertices, tree_edges, start_cell: int, direction: str = "ccw") -> list[int]:
    """Closed STC circumnavigation (sub-cells) of a tree of G given by local vertices and edge
    indices, starting at ``start_cell``; the first cell is repeated at the end."""
    vertices = [int(v) for v in vertices]
    flat = {int(cg.nodes[v]) for v in vertices}
    tedges = [(int(cg.nodes[cg.edges[e, 0]]), int(cg.nodes[cg.edges[e, 1]])) for e in tree_edges]
    if start_cell not in _subcells_of(rep, flat):
        raise ValueError("start sub-cell is not part of the tree")
    cyc = walk_cycle(circumnavigation(rep, flat, tedges), start_cell, direction)
    return cyc + [start_cell] if len(cyc) > 1 else cyc


def _subcells_of(rep, flat_coarse) -> set[int]:
    nx2 = rep.coarse["grid"].nx
    out = set()
    for c in flat_coarse:
        out.update(int(s) for s in rep.sub_cells(*divmod(int(c), nx2)))
    return out


def subcell_weight(rep, cg: CoarseGraph, cell: int) -> float:
    """Weight of a sub-cell in H: a quarter of its coarse vertex's terrain weight."""
    I, J = rep.coarse_of(cell)
    f = int(I) * rep.coarse["grid"].nx + int(J)
    k = cg.index.get(f)
    return (cg.w_vertex[k] if k is not None else 1.0) / 4.0


def coverage_time(rep, cg: CoarseGraph, walk: list[int]) -> float:
    """Published coverage-time unit of a closed walk in H (sum of visited vertex weights)."""
    if len(walk) <= 1:
        return 0.0
    cells = walk[:-1] if walk[0] == walk[-1] else walk
    return float(sum(subcell_weight(rep, cg, c) for c in cells))


def routes_from_walks(walks: dict[str, list[int]]) -> dict[str, list[tuple[int, str]]]:
    """Cell routes for :func:`cell_routes_to_trajectories` (first cell = start cell)."""
    return {rid: [(c, SERVICE) for c in w[1:]] for rid, w in walks.items()}


def open_walk(walk: list[int], return_to_depot: bool) -> list[int]:
    """Drop the closing return of a closed tour when the task does not require it."""
    if return_to_depot or len(walk) < 2 or walk[0] != walk[-1]:
        return walk
    return walk[:-1]


def forest_layer(cg: CoarseGraph, trees: list[list[int]]) -> dict:
    segs = []
    for tr in trees:
        for e in tr:
            u, v = cg.edges[e]
            segs.append([np.round(cg.centers[u], 4).tolist(), np.round(cg.centers[v], 4).tolist()])
    return {"type": "segments", "coords": segs}

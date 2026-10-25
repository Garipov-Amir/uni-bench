"""LS-MCPP (Tang & Ma, 2024) -- local search on the decomposed graph, staged mode.

The planner works on the decomposed graph ``D`` (all traversable sub-cells,
grid adapter with ``coarse="partial"``) and keeps one connected subgraph
``D_i`` per robot whose union is ``D``.  Coverage paths are generated with
**Extended-STC (ESTC)**:

* the augmented terrain graph ``G'_i`` has a vertex for every coarse cell with
  sub-cells in ``D_i`` (a cell with only two diagonally opposite sub-cells
  gives two vertices), and an edge between neighbouring vertices whenever some
  of their sub-cells are adjacent;
* edges between complete vertices keep their weight, others get
  ``w_max * (sum of the incident edge weights of both end points) / 2`` so that
  incomplete vertices become leaves of the minimum spanning tree;
* the path circumnavigates the minimum spanning tree of ``G'_i``.  The
  path-deformation rule of Full-STC is realised by visiting the sub-cells in
  the order of the ideal circumnavigation, skipping absent ones and joining two
  consecutive present sub-cells that are not adjacent by a shortest path in
  ``D_i`` (the detour along the other side of the spanning edge).

The local search (Algorithm 1) samples an operator pool (grow for light,
deduplicate for heavy subgraphs, exchange from heavy to light) by roulette
wheel on the softmax of adaptive pool weights, an operator by the softmax of
its heuristic value, and accepts it if it lowers the makespan or with
probability ``exp(-dtau / t)`` (simulated annealing); forced deduplication runs
every ``S`` iterations and after each improvement.  The initial solution is the
MFC tree cover (as selected in the paper); sub-cells of incomplete coarse
cells are added to the nearest subgraph.

Costs are counted in sub-cell steps (a unit edge weight on uniform terrain).
"""

from __future__ import annotations

import math
from collections import defaultdict, deque

import numpy as np

from ..trajectory import SERVICE
from .base import HyperParam, Manifest, PlanResult, Stage, StagedPlanner
from .grid_common import cell_routes_to_trajectories
from .mfc import rtc_min_max
from .tree_cover import DisjointSet, kruskal

# quadrant codes of a sub-cell (row parity, column parity)
SW, SE, NW, NE = (0, 0), (0, 1), (1, 0), (1, 1)
# quadrant following each direction counter-clockwise (E=0, N=1, W=2, S=3)
Q_AFTER = {0: NE, 1: NW, 2: SW, 3: SE}
DIRS = {(0, 1): 0, (1, 0): 1, (0, -1): 2, (-1, 0): 3}  # (dI, dJ) -> direction


class Decomposed:
    """The decomposed graph D in the scope of the robots, with coarse-cell bookkeeping."""

    def __init__(self, rep):
        g = rep.grid
        self.nx = g.nx
        self.ny = g.ny
        self.nx2 = rep.coarse["grid"].nx
        cells = rep.cells.ravel()
        adj = rep.adj
        starts = [int(rep.starts[r.id]["cell"]) for r in rep.team]
        # components of the sub-cell graph that contain a start
        seen = set()
        for s in starts:
            if s in seen:
                continue
            dq = deque([s])
            seen.add(s)
            while dq:
                u = dq.popleft()
                for v in adj.indices[adj.indptr[u]:adj.indptr[u + 1]]:
                    if cells[v] and v not in seen and self._four(u, v):
                        seen.add(int(v))
                        dq.append(int(v))
        self.V = seen
        self.nbr = {u: [int(v) for v in adj.indices[adj.indptr[u]:adj.indptr[u + 1]]
                        if int(v) in seen and self._four(u, v)] for u in seen}
        self.coarse_cells = defaultdict(set)
        for u in seen:
            self.coarse_cells[self.coarse(u)].add(u)
        self.starts = starts

    def _four(self, u, v) -> bool:
        du, dv = divmod(int(u), self.nx), divmod(int(v), self.nx)
        return abs(du[0] - dv[0]) + abs(du[1] - dv[1]) == 1

    def coarse(self, u: int) -> int:
        i, j = divmod(int(u), self.nx)
        return (i // 2) * self.nx2 + j // 2

    def quad(self, u: int) -> tuple:
        i, j = divmod(int(u), self.nx)
        return (i % 2, j % 2)

    def sub(self, c: int, q: tuple) -> int:
        I, J = divmod(int(c), self.nx2)
        return (2 * I + q[0]) * self.nx + 2 * J + q[1]

    def neighbour(self, c: int, d: int) -> int | None:
        I, J = divmod(int(c), self.nx2)
        dI, dJ = [(0, 1), (1, 0), (0, -1), (-1, 0)][d]
        I2, J2 = I + dI, J + dJ
        if I2 < 0 or J2 < 0 or J2 >= self.nx2 or 2 * I2 >= self.ny:
            return None
        return I2 * self.nx2 + J2

    def complete(self, c: int) -> bool:
        return len(self.coarse_cells.get(c, ())) == 4


# ------------------------------------------------------------------ ESTC
def estc(D: Decomposed, Di: set, root: int) -> tuple[list[int], int]:
    """Closed coverage walk of the connected sub-cell set ``Di`` starting at ``root``; (walk, steps)."""
    if len(Di) == 1:
        return [root, root], 0
    by_cell = defaultdict(set)
    for u in Di:
        by_cell[D.coarse(u)].add(D.quad(u))
    verts, vid = [], {}
    for c, qs in by_cell.items():
        if qs in ({SW, NE}, {SE, NW}):
            for q in sorted(qs):
                vid[(c, q)] = len(verts)
                verts.append((c, frozenset([q])))
        else:
            k = len(verts)
            verts.append((c, frozenset(qs)))
            for q in qs:
                vid[(c, q)] = k
    # G' edges from sub-cell adjacency across coarse cells
    eset = set()
    for u in Di:
        a = vid[(D.coarse(u), D.quad(u))]
        for v in D.nbr[u]:
            if v in Di:
                b = vid[(D.coarse(v), D.quad(v))]
                if a != b and verts[a][0] != verts[b][0]:
                    eset.add((min(a, b), max(a, b)))
    edges = np.array(sorted(eset), dtype=int).reshape(-1, 2)
    n = len(verts)
    deg = np.bincount(edges.ravel(), minlength=n) if len(edges) else np.zeros(n, dtype=int)
    full = np.array([len(q) == 4 for _, q in verts])
    if len(edges):
        both = full[edges[:, 0]] & full[edges[:, 1]]
        w = np.where(both, 1.0, 0.5 * (deg[edges[:, 0]] + deg[edges[:, 1]]))  # w_max = 1 on uniform terrain
        tree = kruskal(n, edges, w)
    else:
        tree = []
    tadj = defaultdict(list)
    for e in tree:
        a, b = int(edges[e, 0]), int(edges[e, 1])
        tadj[a].append(b)
        tadj[b].append(a)

    def direction(a, b):
        Ia, Ja = divmod(verts[a][0], D.nx2)
        Ib, Jb = divmod(verts[b][0], D.nx2)
        return DIRS[(Ib - Ia, Jb - Ja)]

    start = vid[(D.coarse(root), D.quad(root))]
    seq = []
    # Euler traversal of the tree: at a vertex entered from direction dp the robot sweeps the
    # sectors counter-clockwise, emitting the quadrant of every sector and descending into the
    # child that lies in each direction; the root sweeps all four directions.
    def events(v, dp):
        if dp is None:
            return [e for a in range(4) for e in (("D", a), ("Q", a))]
        return [("Q", dp)] + [e for t in (1, 2, 3) for e in (("D", (dp + t) % 4), ("Q", (dp + t) % 4))]

    stack = [(start, None, events(start, None), 0)]
    parent = {start: None}
    while stack:
        v, dp, ev, k = stack.pop()
        while k < len(ev):
            kind, a = ev[k]
            k += 1
            if kind == "Q":
                q = Q_AFTER[a]
                if q in verts[v][1]:
                    seq.append(D.sub(verts[v][0], q))
            else:
                child = next((u for u in tadj[v] if u != parent[v] and direction(v, u) == a), None)
                if child is not None:
                    parent[child] = v
                    stack.append((v, dp, ev, k))
                    back = (a + 2) % 4
                    stack.append((child, back, events(child, back), 0))
                    break
    # rotate to the root and join consecutive sub-cells
    k0 = seq.index(root) if root in seq else 0
    seq = seq[k0:] + seq[:k0] + [seq[k0]]
    walk = [seq[0]]
    for s in seq[1:]:
        p = walk[-1]
        if s == p:
            continue
        if s in D.nbr[p]:
            walk.append(s)
        else:
            walk += _bfs_path(D, Di, p, s)[1:]
    return walk, len(walk) - 1


def _bfs_path(D: Decomposed, Di: set, a: int, b: int) -> list[int]:
    prev = {a: None}
    dq = deque([a])
    while dq:
        u = dq.popleft()
        if u == b:
            break
        for v in D.nbr[u]:
            if v in Di and v not in prev:
                prev[v] = u
                dq.append(v)
    if b not in prev:
        return [a, b]
    out = [b]
    while out[-1] != a:
        out.append(prev[out[-1]])
    return out[::-1]


# ------------------------------------------------------------------ operators
PAIRS = [(SW, SE), (NW, NE), (SW, NW), (SE, NE)]  # bottom, top, left, right sides
# for each pair: direction the pair faces (delta^t), its opposite (delta^b), and the two lateral ones
FACING = {(SW, SE): (3, 1, 2, 0), (NW, NE): (1, 3, 2, 0), (SW, NW): (2, 0, 3, 1), (SE, NE): (0, 2, 3, 1)}


def internal_pairs(D: Decomposed, c: int):
    for qa, qb in PAIRS:
        u, v = D.sub(c, qa), D.sub(c, qb)
        if u in D.V and v in D.V and v in D.nbr[u]:
            yield (qa, qb), u, v


def _parallel(D: Decomposed, Di: set, u: int, v: int) -> bool:
    """Is there an edge (p, q) in Di with (u, p) and (v, q) edges of D?"""
    ui, uj = divmod(u, D.nx)
    vi, vj = divmod(v, D.nx)
    if ui == vi:
        offs = [(1, 0), (-1, 0)]
    else:
        offs = [(0, 1), (0, -1)]
    for di, dj in offs:
        p = (ui + di) * D.nx + uj + dj
        q = (vi + di) * D.nx + vj + dj
        if p in Di and q in Di and p in D.nbr[u] and q in D.nbr[v] and q in D.nbr[p]:
            return True
    return False


def _connected_without(D: Decomposed, Di: set, u: int, v: int) -> bool:
    rest = [x for x in D.nbr[u] + D.nbr[v] if x in Di and x != u and x != v]
    if not rest:
        return len(Di) <= 2
    target = set(rest)
    seen = {rest[0]}
    dq = deque([rest[0]])
    found = {rest[0]} & target
    while dq and len(found) < len(target):
        x = dq.popleft()
        for y in D.nbr[x]:
            if y in Di and y != u and y != v and y not in seen:
                seen.add(y)
                dq.append(y)
                if y in target:
                    found.add(y)
    return len(found) == len(target)


def _cells_in(D: Decomposed, Di: set, c) -> list[int]:
    if c is None:
        return []
    return [x for x in D.coarse_cells.get(c, ()) if x in Di]


def dedup_valid(D: Decomposed, Di: set, root: int, pair, u: int, v: int, counts=None) -> bool:
    """Valid deduplicate operator o_d(i, e); ``counts`` None skips the duplication condition (exchange)."""
    if u not in Di or v not in Di or root in (u, v):
        return False
    if counts is not None and (counts[u] < 2 or counts[v] < 2):
        return False
    c = D.coarse(u)
    if D.complete(c):
        t, b, l, r = FACING[pair]
        ct, cb = D.neighbour(c, t), D.neighbour(c, b)
        if _cells_in(D, Di, ct):  # (1) the facing neighbour is not in G_i
            return False
        present_b = D.coarse_cells.get(cb, set()) if cb is not None else set()
        if not present_b or any(x not in Di for x in present_b):  # (2) the opposite neighbour fully in D_i
            return False
        for side in (l, r):  # (3) lateral neighbours in G_i are full, and so is their corner with delta^b
            cs = D.neighbour(c, side)
            if cs is None or not _cells_in(D, Di, cs):
                continue
            corner = D.neighbour(cs, b)
            for cell in (cs, corner):
                if cell is not None and any(x not in Di for x in D.coarse_cells.get(cell, ())):
                    return False
    return _connected_without(D, Di, u, v)


def grow_valid(D: Decomposed, Di: set, u: int, v: int) -> bool:
    return u not in Di and v not in Di and _parallel(D, Di, u, v)


# ------------------------------------------------------------------ local search
class Solution:
    def __init__(self, D, subs, roots):
        self.D = D
        self.subs = [set(s) for s in subs]
        self.roots = roots
        self.counts = defaultdict(int)
        for s in self.subs:
            for x in s:
                self.counts[x] += 1
        self.walks, self.cost = [], []
        for i in range(len(subs)):
            w, c = estc(D, self.subs[i], roots[i])
            self.walks.append(w)
            self.cost.append(c)

    @property
    def tau(self) -> int:
        return max(self.cost)

    def copy(self):
        new = Solution.__new__(Solution)
        new.D, new.roots = self.D, self.roots
        new.subs = [set(s) for s in self.subs]
        new.counts = defaultdict(int, self.counts)
        new.walks = [list(w) for w in self.walks]
        new.cost = list(self.cost)
        return new

    def ops(self, i: int):
        """(grow, dedup, exchange) operators of subgraph i."""
        D, Di, root = self.D, self.subs[i], self.roots[i]
        cells = {D.coarse(x) for x in Di}
        near = set(cells)
        for c in cells:
            for d in range(4):
                nb = D.neighbour(c, d)
                if nb is not None and nb in D.coarse_cells:
                    near.add(nb)
        grow, dedup, exch = [], [], []
        for c in near:
            for pair, u, v in internal_pairs(D, c):
                if grow_valid(D, Di, u, v):
                    grow.append((u, v))
                    for j, Dj in enumerate(self.subs):
                        if j != i and u in Dj and v in Dj and dedup_valid(D, Dj, self.roots[j], pair, u, v):
                            exch.append((j, u, v))
                elif u in Di and v in Di and dedup_valid(D, Di, root, pair, u, v, self.counts):
                    dedup.append((u, v))
        return grow, dedup, exch

    def apply(self, i: int, add=(), remove=()):
        s = self.subs[i]
        for x in add:
            if x not in s:
                s.add(x)
                self.counts[x] += 1
        for x in remove:
            if x in s:
                s.discard(x)
                self.counts[x] -= 1
        self.walks[i], self.cost[i] = estc(self.D, s, self.roots[i])


def _softmax_pick(values, rng):
    v = np.asarray(values, dtype=float)
    v = v - v.max()
    p = np.exp(v)
    p /= p.sum()
    return int(rng.choice(len(v), p=p))


def u_turns(sol: Solution, i: int):
    """U-turns (u, v) of path i: p -> u -> v -> q with p and q adjacent, u, v duplicated in D_i."""
    w = sol.walks[i]
    D = sol.D
    out = []
    for t in range(1, len(w) - 2):
        p, u, v, q = w[t - 1], w[t], w[t + 1], w[t + 2]
        if (u != v and sol.counts[u] > 1 and sol.counts[v] > 1 and q in D.nbr[p] and u != sol.roots[i]
                and v != sol.roots[i] and D.coarse(u) == D.coarse(v)):
            out.append((u, v))
    return out


def forced_deduplication(sol: Solution, k: int):
    order = sorted(range(len(sol.subs)), key=lambda i: -sol.cost[i])
    for i in order:
        guard = 0
        while guard < 10 * len(sol.subs[i]):
            guard += 1
            done = False
            for u, v in u_turns(sol, i):
                if _connected_without(sol.D, sol.subs[i], u, v):
                    sol.apply(i, remove=(u, v))
                    done = True
                    break
            if not done:
                break
    for i in order:
        guard = 0
        while guard < 10 * len(sol.subs[i]):
            guard += 1
            _, dedup, _ = sol.ops(i)
            if not dedup:
                break
            h = [k * sol.cost[i] + 0.5 * (sol.counts[u] + sol.counts[v]) for u, v in dedup]
            u, v = dedup[int(np.argmin(h))]
            sol.apply(i, remove=(u, v))


def local_search(sol: Solution, rng, M: int, S: int, gamma: float, deadline=None) -> tuple[Solution, dict]:
    k = len(sol.subs)
    alpha = math.exp(math.log(0.2) / max(M, 1))
    t = 1.0
    p = np.ones(3)
    best = sol.copy()
    forced_deduplication(sol, k)
    if sol.tau < best.tau:
        best = sol.copy()
    cache = {i: sol.ops(i) for i in range(k)}
    stats = {"iterations": 0, "accepted": 0, "improved": 0}
    for it in range(1, M + 1):
        if deadline is not None and deadline():
            break
        stats["iterations"] = it
        mean = float(np.mean(sol.cost))
        light = [i for i in range(k) if sol.cost[i] <= mean]
        heavy = [i for i in range(k) if sol.cost[i] > mean]
        pools = [[(i, u, v) for i in light for u, v in cache[i][0]],
                 [(i, u, v) for i in heavy for u, v in cache[i][1]],
                 [(i, j, u, v) for i in light for j, u, v in cache[i][2]]]
        avail = [q for q in range(3) if pools[q]]
        if not avail:
            break
        qsel = avail[_softmax_pick(p[avail], rng)]
        pool = pools[qsel]
        if qsel == 0:
            h = [-k * sol.cost[i] - 0.5 * (sol.counts[u] + sol.counts[v]) for i, u, v in pool]
        elif qsel == 1:
            h = [k * sol.cost[i] + 0.5 * (sol.counts[u] + sol.counts[v]) for i, u, v in pool]
        else:
            h = [sol.cost[j] - sol.cost[i] for i, j, u, v in pool]
        op = pool[_softmax_pick(h, rng)]
        new = sol.copy()
        if qsel == 0:
            i, u, v = op
            new.apply(i, add=(u, v))
            touched = {i}
        elif qsel == 1:
            i, u, v = op
            new.apply(i, remove=(u, v))
            touched = {i}
        else:
            i, j, u, v = op
            new.apply(i, add=(u, v))
            new.apply(j, remove=(u, v))
            touched = {i, j}
        dtau = new.tau - sol.tau
        p[qsel] = (1 - gamma) * p[qsel] + gamma * max(-dtau, 0.0)
        if dtau < 0 or rng.random() < math.exp(-dtau / t):
            sol = new
            stats["accepted"] += 1
            # operators of subgraphs whose cells or duplication changed
            changed = set()
            for x in op[-2:]:
                for j2 in range(k):
                    if x in sol.subs[j2] or any(y in sol.subs[j2] for y in sol.D.nbr[x]):
                        changed.add(j2)
            for j2 in touched | changed:
                cache[j2] = sol.ops(j2)
        if it % S == 0 or dtau < 0:
            forced_deduplication(sol, k)
            cache = {i: sol.ops(i) for i in range(k)}
        if sol.tau < best.tau:
            best = sol.copy()
            stats["improved"] += 1
        t *= alpha
    return best, stats


# ------------------------------------------------------------------ initial solution
def initial_solution(rep, D: Decomposed) -> list[set]:
    """MFC tree cover of the complete coarse cells, extended to every sub-cell of D."""
    k = len(rep.team)
    nx2 = D.nx2
    complete = sorted(c for c, s in D.coarse_cells.items() if len(s) == 4
                      and all(y in D.nbr[x] for x in s for y in s if D._four(x, y)))
    subs = [set() for _ in range(k)]
    owner = {}
    if complete:
        idx = {c: t for t, c in enumerate(complete)}
        E = []
        for c in complete:
            for d in (0, 1):
                nb = D.neighbour(c, d)
                if nb in idx:
                    # both pairs of sub-cells across the border must be adjacent
                    a1, b1 = (D.sub(c, SE), D.sub(nb, SW)) if d == 0 else (D.sub(c, NW), D.sub(nb, SW))
                    a2, b2 = (D.sub(c, NE), D.sub(nb, NW)) if d == 0 else (D.sub(c, NE), D.sub(nb, SE))
                    if b1 in D.nbr[a1] and b2 in D.nbr[a2]:
                        E.append((idx[c], idx[nb]))
        E = np.array(E, dtype=int).reshape(-1, 2)
        # root: nearest complete coarse cell of every start (graph distance in D)
        roots, paths = [], []
        for s in D.starts:
            path = _nearest(D, s, lambda x: D.coarse(x) in idx)
            roots.append(idx[D.coarse(path[-1])])
            paths.append(path)
        # components of the complete-cell graph containing roots
        ds = DisjointSet(len(complete))
        for a, b in E:
            ds.union(int(a), int(b))
        groups = defaultdict(list)
        for r_i, r in enumerate(roots):
            groups[ds.find(r)].append(r_i)
        trees = [None] * k
        for comp, members in groups.items():
            verts = [v for v in range(len(complete)) if ds.find(v) == comp]
            loc = {v: t for t, v in enumerate(verts)}
            mask = np.array([ds.find(int(a)) == comp for a, _ in E], dtype=bool)
            El = np.array([[loc[int(a)], loc[int(b)]] for a, b in E[mask]], dtype=int).reshape(-1, 2)
            res, _, _ = rtc_min_max(len(verts), El, np.ones(len(El)), [loc[roots[m]] for m in members])
            for m, (tv, _) in zip(members, res):
                trees[m] = {complete[verts[v]] for v in tv}
        for i in range(k):
            for c in trees[i] or ():
                subs[i] |= D.coarse_cells[c]
            subs[i] |= set(paths[i])
    else:
        for i, s in enumerate(D.starts):
            subs[i].add(s)
    for i, s in enumerate(subs):
        for x in s:
            owner.setdefault(x, i)
    # remaining sub-cells (incomplete coarse cells): multi-source growth from the subgraphs
    dq = deque(owner.keys())
    while dq:
        x = dq.popleft()
        for y in D.nbr[x]:
            if y not in owner:
                owner[y] = owner[x]
                subs[owner[x]].add(y)
                dq.append(y)
    return subs


def _nearest(D: Decomposed, s: int, goal) -> list[int]:
    prev = {s: None}
    dq = deque([s])
    while dq:
        u = dq.popleft()
        if goal(u):
            out = [u]
            while prev[out[-1]] is not None:
                out.append(prev[out[-1]])
            return out[::-1]
        for v in D.nbr[u]:
            if v not in prev:
                prev[v] = u
                dq.append(v)
    return [s]


# ------------------------------------------------------------------ stages
class DecomposedGraph(Stage):
    kind = "environment_model"
    name = "decomposed_graph"
    consumes = ("representation",)
    produces = "model"

    def run(self, data, rep, ctx, params):
        return {"D": Decomposed(rep)}

    def summary(self, out):
        D = out["D"]
        return {"sub_cells": len(D.V), "coarse_cells": len(D.coarse_cells),
                "incomplete_coarse_cells": sum(1 for s in D.coarse_cells.values() if len(s) < 4)}


class InitialCover(Stage):
    kind = "work_distribution"
    name = "mfc_initial"
    consumes = ("model",)
    produces = "initial"

    def run(self, data, rep, ctx, params):
        D = data["model"]["D"]
        subs = initial_solution(rep, D)
        sol = Solution(D, subs, D.starts)
        return {"solution": sol, "tau": sol.tau}

    def metrics(self, out, rep):
        return {"initial_makespan_steps": float(out["tau"])}


class LocalSearch(Stage):
    kind = "optimization"
    name = "local_search"
    consumes = ("initial",)
    produces = "routes"

    def run(self, data, rep, ctx, params):
        sol = data["initial"]["solution"]
        best, stats = local_search(sol, ctx.rng, int(params["max_iterations"]), int(params["dedup_step"]),
                                   float(params["pool_weight_decay"]), deadline=lambda: ctx.out_of_time(0.85))
        walks = {r.id: best.walks[i] for i, r in enumerate(rep.team)}
        if not ctx.return_to_depot:
            walks = {rid: (w[:-1] if len(w) > 1 and w[0] == w[-1] else w) for rid, w in walks.items()}
        routes = {rid: [(c, SERVICE) for c in w[1:]] for rid, w in walks.items()}
        # coverage time in the published unit: sum of the visited decomposed-vertex weights
        # (a quarter of the coarse cell's terrain weight) along every closed walk
        wts = np.asarray(rep.coarse.get("weights", np.ones(rep.coarse["cells"].shape))).ravel()
        ct = {r.id: float(sum(wts[best.D.coarse(c)] / 4.0 for c in best.walks[i][:-1]))
              for i, r in enumerate(rep.team)}
        return {"routes": routes, "tau": best.tau, "costs": dict(zip([r.id for r in rep.team], best.cost)),
                "stats": stats, "subgraphs": [sorted(s) for s in best.subs], "coverage_time": ct}

    def metrics(self, out, rep):
        return {"makespan_steps": float(out["tau"]), **{k: float(v) for k, v in out["stats"].items()}}


class LSMCPP(StagedPlanner):
    manifest = Manifest(
        name="ls_mcpp",
        version="1.0",
        reference="Tang & Ma (2024), Large-scale multi-robot coverage path planning via local search (AAAI)",
        family="grid",
        adapter="grid",
        adapter_params={"coarse": "partial", "connectivity": 4},
        constraints={"multi_robot", "return_to_depot"},
        stochastic=True,
        mode="staged",
        stages=["decomposed_graph", "mfc_initial", "local_search"],
        hyperparameters=[
            HyperParam("max_iterations", "int", 3000, low=300, high=30000, log=True, doc="M, maximum iterations"),
            HyperParam("dedup_step", "int", 100, low=10, high=1000, log=True, doc="S, forced deduplication step"),
            HyperParam("pool_weight_decay", "float", 0.01, low=0.001, high=0.2, log=True, doc="gamma"),
        ],
        core_modules=["uninbench/planners/ls_mcpp.py", "uninbench/planners/mfc.py",
                      "uninbench/planners/tree_cover.py"],
        description="Local search with grow, deduplicate and exchange operators on sub-cell subgraphs; ESTC paths.",
    )
    stages = [DecomposedGraph(), InitialCover(), LocalSearch()]

    def finish(self, data, rep, ctx) -> PlanResult:
        routes = data["routes"]["routes"]
        ts = cell_routes_to_trajectories(rep, routes, self.manifest, ctx)
        g = rep.grid
        lab = np.full(g.shape, -1, dtype=np.int16)
        for i, sub in enumerate(data["routes"]["subgraphs"]):
            lab.ravel()[np.asarray(sub, dtype=int)] = i
        from ..util import pack_array

        return PlanResult(ts, native={rid: [c for c, _ in rt] for rid, rt in routes.items()},
                          layers={"subgraphs": {"type": "labels", "grid": g.to_dict(), "labels": pack_array(lab)}},
                          info={"makespan_steps": data["routes"]["tau"], "costs_steps": data["routes"]["costs"],
                                "coverage_time": data["routes"]["coverage_time"],
                                "local_search": data["routes"]["stats"]})

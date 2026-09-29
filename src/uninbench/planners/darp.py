"""DARP (Kapoutsis, Chatzichristofis & Kosmatopoulos, 2017) and compositions.

DARP divides the free coarse cells into m connected regions of (nearly) equal
size, each containing its robot's initial cell.  Every iteration assigns cells
to the robot with the smallest weighted distance, corrects the per-robot
weights towards the fair share, and biases disconnected regions towards the
component containing the robot (connectivity matrices).  A small random
perturbation makes the method stochastic.

Two staged planners use the DARP work distribution:

* ``darp_stc``  -- DARP + STC routes per region (the original method);
* ``darp_bf``   -- DARP + back-and-forth (boustrophedon) routes per region, a
  *composition* that is reported as such and never as an original algorithm.
"""

from __future__ import annotations

import math

import numpy as np
from scipy.sparse.csgraph import connected_components, dijkstra
from scipy.spatial import cKDTree

from ..trajectory import SERVICE, TRANSIT
from .base import HyperParam, Manifest, PlanResult, Stage, StagedPlanner
from .grid_common import (DIRECTION_PARAM, TREE_PARAM, cell_routes_to_trajectories, coarse_component,
                          coarse_matrix, connect_cells, partition_layer, partition_metrics, spanning_tree,
                          circumnavigation, walk_cycle, coarse_adjacency, tree_layer)

DARP_PARAMS = [
    HyperParam("max_iter", "int", 10000, low=500, high=80000, log=True, doc="iterations before relaxing the threshold"),
    HyperParam("cc_variation", "float", 0.01, low=1e-3, high=0.2, log=True, doc="connectivity bias range"),
    HyperParam("random_level", "float", 1e-4, low=1e-6, high=1e-2, log=True, doc="random perturbation"),
    HyperParam("dcells", "int", 2, low=0, high=20, doc="largest accepted cell imbalance threshold"),
    HyperParam("metric", "categorical", "euclidean", choices=["euclidean", "geodesic"],
               doc="distance from the robots (the original uses Euclidean distance)"),
]


def darp_partition(adj, start_nodes: list[int], rng: np.random.Generator, centers: np.ndarray | None = None,
                   max_iter: int = 10000, cc_variation: float = 0.01, random_level: float = 1e-4, dcells: int = 2,
                   metric: str = "euclidean", deadline=None):
    """DARP on a graph given by the sparse adjacency ``adj`` (N x N).

    Follows the reference implementation: cumulative multiplicative updates of
    the metric matrices by a fair-share correction, a random matrix and a
    connectivity multiplier; the imbalance threshold is relaxed (and the
    iteration budget halved) whenever an attempt does not converge.

    Returns ``(labels (N,), info)``.  ``start_nodes`` must be distinct.
    """
    N = adj.shape[0]
    m = len(start_nodes)
    starts = np.asarray(start_nodes, dtype=int)
    if metric == "euclidean" and centers is not None:
        D0 = np.hypot(centers[None, :, 0] - centers[starts, 0][:, None], centers[None, :, 1] - centers[starts, 1][:, None])
    else:
        D0 = dijkstra(adj, directed=False, indices=starts, unweighted=True)
        fin = np.isfinite(D0)
        D0 = np.where(fin, D0, (D0[fin].max() if fin.any() else 1.0) * 10)
    D0 = D0 + 1e-9
    tree = cKDTree(centers) if centers is not None else None
    f = N / m
    info = {"converged": False, "iterations": 0, "threshold": 0}
    best = None
    thr = 0
    budget = int(max_iter)
    A = np.argmin(D0, axis=0)
    while thr <= dcells:
        M = D0.copy()
        down = (N - thr * (m - 1)) / (N * m)
        up = (N + thr) / (N * m)
        for _ in range(max(1, budget)):
            info["iterations"] += 1
            A = np.argmin(M, axis=0)
            A[starts] = np.arange(m)
            counts = np.bincount(A, minlength=m)
            CC = np.ones((m, N))
            connected = np.ones(m, dtype=bool)
            for r in range(m):
                idx = np.flatnonzero(A == r)
                ncomp, lab = connected_components(adj[idx][:, idx], directed=False)
                if ncomp <= 1:
                    continue
                connected[r] = False
                own = lab[np.searchsorted(idx, starts[r])]
                d1 = _normalized_distance(idx[lab == own], N, tree, adj)
                d2 = _normalized_distance(idx[lab != own], N, tree, adj)
                d = d1 - d2
                lo, hi = d.min(), d.max()
                if hi > lo:
                    CC[r] = (d - lo) * (2 * cc_variation / (hi - lo)) + (1 - cc_variation)
            if connected.all():
                imb = float(np.abs(counts - f).max())
                if best is None or imb < best[0]:
                    best = (imb, A.copy())
                if np.all(counts >= np.floor(f) - thr) and np.all(counts <= np.ceil(f) + thr):
                    info["converged"] = True
                    break
            plain = counts / N
            div = np.where(plain < down, down - plain, np.where(plain > up, up - plain, 0.0))
            neg = div < 0
            tot_neg_perc = float(np.abs(div[neg]).sum())
            tot_neg_plain = float(plain[neg].sum())
            corr = np.ones(m)
            if tot_neg_plain != 0:
                corr = np.where(neg, 1 + plain / tot_neg_plain * tot_neg_perc / 2,
                                1 - plain / tot_neg_plain * tot_neg_perc / 2)
            RM = 2 * random_level * rng.random((m, N)) + (1 - random_level)
            M = M * corr[:, None] * RM * CC
            M /= M.max()  # a common rescaling does not change the argmin
            if deadline is not None and deadline():
                break
        info["threshold"] = thr
        if info["converged"] or (deadline is not None and deadline()):
            break
        thr += 1
        budget = max(1, budget // 2)
    if info["converged"]:
        pass
    elif best is not None:
        A = best[1]
    else:
        A = _repair(adj, A, starts)
        info["repaired"] = True
    info["imbalance_cells"] = float(np.abs(np.bincount(A, minlength=m) - f).max())
    return A, info


def _normalized_distance(sources: np.ndarray, N: int, tree, adj) -> np.ndarray:
    """Distance of every node to the nearest source node, normalised to [0, 1]."""
    if tree is not None:
        d, _ = cKDTree(tree.data[sources]).query(tree.data)
    else:
        d = dijkstra(adj, directed=False, indices=sources, unweighted=True, min_only=True)
        fin = np.isfinite(d)
        d = np.where(fin, d, d[fin].max() if fin.any() else 0.0)
    mx = d.max()
    return d / mx if mx > 0 else d


def _repair(adj, A, starts):
    """Make every region connected: keep the component of each start, re-grow the rest."""
    m = len(starts)
    out = np.full(len(A), -1)
    for r in range(m):
        idx = np.flatnonzero(A == r)
        _, lab = connected_components(adj[idx][:, idx], directed=False)
        own = lab[np.searchsorted(idx, starts[r])]
        out[idx[lab == own]] = r
    changed = True
    while changed and np.any(out < 0):
        changed = False
        for u in np.flatnonzero(out < 0):
            nb = adj.indices[adj.indptr[u]:adj.indptr[u + 1]]
            lab = out[nb]
            lab = lab[lab >= 0]
            if len(lab):
                out[u] = np.bincount(lab).argmax()
                changed = True
    return out


# ------------------------------------------------------------------ stages
class CoarseRegions(Stage):
    kind = "environment_model"
    name = "coarse_regions"
    consumes = ("representation",)
    produces = "model"

    def run(self, data, rep, ctx, params):
        nx2 = rep.coarse["grid"].nx
        roots = []
        for r in rep.team:
            I, J = rep.coarse_of(rep.starts[r.id]["cell"])
            roots.append(int(I) * nx2 + int(J))
        nodes = set()
        for root in roots:
            if root not in nodes:
                nodes |= coarse_component(rep, root)
        nodes = np.array(sorted(nodes), dtype=int)
        return {"nodes": nodes, "roots": roots, "adj": coarse_matrix(rep, nodes)}

    def summary(self, out):
        return {"coarse_cells": int(len(out["nodes"]))}


class DARPDistribution(Stage):
    kind = "work_distribution"
    name = "darp"
    consumes = ("model",)
    produces = "partition"
    hyperparameters = DARP_PARAMS

    def run(self, data, rep, ctx, params):
        model = data["model"]
        nodes, adj = model["nodes"], model["adj"]
        pos = {int(n): k for k, n in enumerate(nodes)}
        seeds = []
        used = set()
        seed_cells = {}
        for r, root in zip(rep.team, model["roots"]):
            k = pos[root]
            if k in used:  # two robots in the same coarse cell: nearest free cell as DARP seed
                dist = dijkstra(adj, directed=False, indices=k, unweighted=True)
                for cand in np.argsort(dist, kind="stable"):
                    if np.isfinite(dist[cand]) and int(cand) not in used:
                        k = int(cand)
                        break
            used.add(k)
            seeds.append(k)
            seed_cells[r.id] = int(nodes[k])
        g = rep.coarse["grid"]
        ci, cj = np.divmod(nodes, g.nx)
        centers = np.stack(g.center(ci, cj), axis=1)
        # DARP assumes a connected area: every connected component is divided among
        # the robots that start in it
        _, comp = connected_components(adj, directed=False)
        labels = np.full(len(nodes), -1, dtype=int)
        info = {"converged": True, "iterations": 0, "components": int(len(set(comp[seeds])))}
        for c in sorted(set(comp[seeds])):
            idx = np.flatnonzero(comp == c)
            robots = [k for k, sd in enumerate(seeds) if comp[sd] == c]
            if len(robots) == 1:
                labels[idx] = robots[0]
                continue
            local = {int(n): k for k, n in enumerate(idx)}
            lab, inf = darp_partition(
                adj[idx][:, idx], [local[seeds[k]] for k in robots], ctx.rng, centers=centers[idx],
                max_iter=int(params["max_iter"]), cc_variation=float(params["cc_variation"]),
                random_level=float(params["random_level"]), dcells=int(params["dcells"]), metric=params["metric"],
                deadline=lambda: ctx.out_of_time(0.8))
            labels[idx] = np.asarray(robots)[lab]
            info["converged"] &= inf["converged"]
            info["iterations"] += inf["iterations"]
            info["imbalance_cells"] = max(info.get("imbalance_cells", 0.0), inf["imbalance_cells"])
            info["repaired"] = info.get("repaired", False) or inf.get("repaired", False)
        return {"labels": labels, "nodes": nodes, "seed_cells": seed_cells, "info": info, "adj": adj}

    def metrics(self, out, rep):
        m = partition_metrics(out["labels"], len(rep.team), out["adj"])
        m.update({k: v for k, v in out["info"].items()})
        return m


class STCPerRegion(Stage):
    kind = "route_construction"
    name = "stc_per_region"
    consumes = ("partition",)
    produces = "routes"

    def run(self, data, rep, ctx, params):
        part = data["partition"]
        nodes, labels = part["nodes"], part["labels"]
        adj = coarse_adjacency(rep)
        nx2 = rep.coarse["grid"].nx
        routes, trees = {}, []
        for k, r in enumerate(rep.team):
            region = set(int(n) for n in nodes[labels == k])
            seed = part["seed_cells"][r.id]
            start = rep.starts[r.id]["cell"]
            I, J = rep.coarse_of(start)
            own_root = int(I) * nx2 + int(J)
            if not region:
                routes[r.id] = []
                continue
            tree = spanning_tree(region, adj, seed, params["tree"], nx2, ctx.rng)
            trees += tree
            cyc_adj = circumnavigation(rep, region, tree)
            if own_root == seed:
                entry = start
            else:  # robot starts outside its region: enter at the seed cell's nearest sub-cell
                subs = rep.sub_cells(*divmod(seed, nx2))
                d = rep.distances(start)
                entry = int(min(subs, key=lambda c: d[c]))
            tour = walk_cycle(cyc_adj, entry, params["direction"])
            route = []
            if entry != start:
                lead = connect_cells(rep, start, entry)
                route += [(c, TRANSIT) for c in lead[1:-1]] + [(entry, SERVICE)]
            route += [(c, SERVICE) for c in tour[1:]]
            routes[r.id] = route
        return {"routes": routes, "trees": trees}


class BoustrophedonPerRegion(Stage):
    """Back-and-forth sweeps over the sub-cells of each region (row runs, greedy order)."""

    kind = "route_construction"
    name = "bf_per_region"
    consumes = ("partition",)
    produces = "routes"

    def run(self, data, rep, ctx, params):
        part = data["partition"]
        nodes, labels = part["nodes"], part["labels"]
        nx2 = rep.coarse["grid"].nx
        routes = {}
        for k, r in enumerate(rep.team):
            region = nodes[labels == k]
            subs = set()
            for c in region:
                subs.update(int(s) for s in rep.sub_cells(*divmod(int(c), nx2)))
            routes[r.id] = _row_runs_route(rep, subs, rep.starts[r.id]["cell"], params.get("sweep", "rows"))
        return {"routes": routes, "trees": []}


def _row_runs_route(rep, cells: set[int], start: int, sweep: str = "rows"):
    """Greedy boustrophedon over maximal runs of connected cells along rows (or columns)."""
    if not cells:
        return []
    arr = np.array(sorted(cells))
    i, j = rep.rc(arr)
    major, minor = (i, j) if sweep == "rows" else (j, i)
    runs = []
    order = np.lexsort((minor, major))
    cur = [int(arr[order[0]])]
    for a, b in zip(order[:-1], order[1:]):
        same_line = major[a] == major[b] and minor[b] == minor[a] + 1
        if same_line and int(arr[b]) in rep.neighbors(int(arr[a])):
            cur.append(int(arr[b]))
        else:
            runs.append(cur)
            cur = [int(arr[b])]
    runs.append(cur)
    remaining = list(range(len(runs)))
    route = []
    pos = start
    while remaining:
        d = rep.distances(pos)
        best, best_d, best_rev = None, math.inf, False
        for ridx in remaining:
            run = runs[ridx]
            for rev, end in ((False, run[0]), (True, run[-1])):
                if d[end] < best_d:
                    best, best_d, best_rev = ridx, d[end], rev
        if best is None:
            break
        remaining.remove(best)
        run = runs[best][::-1] if best_rev else runs[best]
        if run[0] != pos:
            lead = connect_cells(rep, pos, run[0])
            route += [(c, TRANSIT) for c in lead[1:-1]] + [(run[0], SERVICE)]
        route += [(c, SERVICE) for c in run[1:]]
        pos = run[-1]
    return route


class _DARPBase(StagedPlanner):
    def finish(self, data, rep, ctx) -> PlanResult:
        routes = data["routes"]["routes"]
        ts = cell_routes_to_trajectories(rep, routes, self.manifest, ctx)
        part = data["partition"]
        layers = {"partition": partition_layer(rep, part["nodes"], part["labels"])}
        if data["routes"].get("trees"):
            layers["spanning_tree"] = tree_layer(rep, data["routes"]["trees"])
        return PlanResult(ts, native={rid: [c for c, _ in rt] for rid, rt in routes.items()}, layers=layers,
                          info={"darp": part["info"]})


class DARPSTC(_DARPBase):
    manifest = Manifest(
        name="darp_stc",
        version="1.0",
        reference="Kapoutsis, Chatzichristofis & Kosmatopoulos (2017), DARP: Divide areas algorithm for optimal "
                  "multi-robot coverage path planning",
        family="grid",
        adapter="grid",
        adapter_params={"coarse": True, "connectivity": 4},
        constraints={"multi_robot", "deconfliction"},
        stochastic=True,
        mode="staged",
        stages=["coarse_regions", "darp", "stc_per_region"],
        hyperparameters=DARP_PARAMS + [TREE_PARAM, DIRECTION_PARAM],
        core_modules=["uninbench/planners/darp.py", "uninbench/planners/grid_common.py"],
        description="DARP partition of coarse cells followed by STC inside every region.",
    )
    stages = [CoarseRegions(), DARPDistribution(), STCPerRegion()]


class DARPBF(_DARPBase):
    manifest = Manifest(
        name="darp_bf",
        version="1.0",
        reference="Composition: DARP partition (Kapoutsis et al., 2017) + back-and-forth sweeps",
        family="grid",
        adapter="grid",
        adapter_params={"coarse": True, "connectivity": 4},
        constraints={"multi_robot", "deconfliction"},
        stochastic=True,
        mode="staged",
        stages=["coarse_regions", "darp", "bf_per_region"],
        composition_of=["darp", "boustrophedon"],
        hyperparameters=DARP_PARAMS + [HyperParam("sweep", "categorical", "rows", choices=["rows", "cols"],
                                                     doc="back-and-forth along rows or columns of sub-cells")],
        core_modules=["uninbench/planners/darp.py", "uninbench/planners/grid_common.py"],
        description="Staged composition: DARP work distribution with back-and-forth route construction.",
    )
    stages = [CoarseRegions(), DARPDistribution(), BoustrophedonPerRegion()]

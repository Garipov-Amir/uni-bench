"""Coverage with Route Clustering, CRC (Karapetyan, Benson, McKinney, Taslakian & Rekleitis, 2017).

Multi-robot extension of Efficient Complete Coverage (Xu et al., 2014) --
geometric family, staged mode:

1. **Reeb graph** of the boustrophedon cellular decomposition (Choset, 2000):
   vertices are the critical points where cells meet, every cell is an edge
   between its two ends along the sweep direction.  The coverage weight of an
   edge is the cell's boustrophedon path length (area / working width), its
   traversal weight the distance between its end vertices.
2. **Chinese postman**: odd-degree vertices are paired by a minimum-weight
   perfect matching on traversal shortest-path distances and the matched paths
   are duplicated; an Eulerian tour from the start vertex visits every cell,
   covering it the first time and traversing it afterwards.
3. **k-postman split (Frederickson, Hecht & Kim, 1978)**: with
   ``s_max = max_j s(v1, v_j) + c(e_j) + s(v_{j+1}, v1)`` and total tour cost
   ``C``, the j-th split point is the last tour vertex whose prefix cost does
   not exceed ``L_j = j (C - 2 s_max) / k + s_max``, moved to the next vertex
   when finishing the edge is cheaper; every sub-tour is travelled from and
   back to the start (traversal shortest paths, the paper's modification).

Cells are covered by back-and-forth passes spaced by the working width,
progressing from the cell end at which the tour enters to the other end.
The paper assumes a single start point for all robots; with distinct starts,
the tour starts at the first robot's start vertex and sub-tours are assigned
to robots by a bottleneck assignment of their travel times.
"""

from __future__ import annotations

import math
from collections import defaultdict

import networkx as nx
import numpy as np
from scipy.optimize import linear_sum_assignment
from shapely.geometry import LineString, Point

from ..geometry.decomposition import lines_of, rotate
from ..trajectory import TrajectorySet
from .base import Manifest, PlanResult, Stage, StagedPlanner
from .sweep_common import Pass, RouteBuilder


def reeb_graph(cells, adjacency, angle: float, tol: float = 1e-7):
    """Vertices (critical points) and cell edges of a boustrophedon decomposition.

    Returns (positions (V, 2) world frame, edges [(bottom vertex, top vertex)] per cell)."""
    rot = [rotate(c, angle) for c in cells]
    ext = [(c.bounds[1], c.bounds[3]) for c in rot]
    parent = {}

    def find(a):
        while parent.setdefault(a, a) != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    def union(a, b):
        parent[find(a)] = find(b)

    for a, b in adjacency:
        # the lower cell's top end meets the upper cell's bottom end
        if abs(ext[a][1] - ext[b][0]) <= max(tol, 1e-9 * (1 + abs(ext[a][1]))):
            union((a, 1), (b, 0))
        elif abs(ext[b][1] - ext[a][0]) <= max(tol, 1e-9 * (1 + abs(ext[b][1]))):
            union((b, 1), (a, 0))
        else:  # touching at other heights (numerical): join the nearer ends
            union((a, 1) if ext[a][1] < ext[b][1] else (a, 0), (b, 0) if ext[a][1] < ext[b][1] else (b, 1))
    ends = [(i, s) for i in range(len(cells)) for s in (0, 1)]
    roots = sorted({find(e) for e in ends}, key=str)
    vid = {r: k for k, r in enumerate(roots)}
    pts = defaultdict(list)
    for i, s in ends:
        g = rot[i]
        y = ext[i][s]
        seg = LineString([(g.bounds[0] - 1, y), (g.bounds[2] + 1, y)]).intersection(g.buffer(1e-9))
        c = seg.centroid if not seg.is_empty else g.centroid
        pts[vid[find((i, s))]].append((c.x, c.y))
    pos_rot = np.array([np.mean(pts[k], axis=0) for k in range(len(roots))])
    pos = np.array([rotate(Point(x, y), angle, inverse=True).coords[0] for x, y in pos_rot])
    edges = [(vid[find((i, 0))], vid[find((i, 1))]) for i in range(len(cells))]
    return pos, edges


def cell_passes(cell, cspace, angle: float, w: float, lo: float, hi: float) -> list[np.ndarray]:
    """Back-and-forth passes of one cell (bottom to top in the rotated frame)."""
    crot = rotate(cell, angle)
    region = crot.intersection(rotate(cspace, angle))
    if region.is_empty:
        return []
    n = int(np.ceil((hi - lo) / w - 1e-6)) + 1
    offsets = np.linspace(lo, hi, n) if n > 1 else np.array([(lo + hi) / 2])
    y0, y1 = crot.bounds[1], crot.bounds[3]
    mine = offsets[(offsets >= y0) & ((offsets < y1) | (offsets >= hi - 1e-12))]
    if len(mine) == 0:
        mine = np.array([(max(y0, lo) + min(y1, hi)) / 2])
    x0, x1 = crot.bounds[0] - 1, crot.bounds[2] + 1
    out = []
    for y in mine:
        for piece in lines_of(LineString([(x0, y), (x1, y)]).intersection(region)):
            if piece.length > 1e-9:
                out.append(np.asarray(rotate(piece, angle, inverse=True).coords)[[0, -1]])
    return out


class ReebGraphModel(Stage):
    kind = "environment_model"
    name = "reeb_graph"
    consumes = ("representation",)
    produces = "model"

    def run(self, data, rep, ctx, params):
        robot = rep.team[0]
        cs = rep.cspace(robot)
        w = min(r.working_width for r in rep.team)
        angle = rep.angle or 0.0
        cells = rep.cells or [rep.free]
        adj = rep.adjacency or []
        pos, edges = reeb_graph(cells, adj, angle)
        _, lo, _, hi = rotate(cs, angle).bounds
        passes = [cell_passes(c, cs, angle, w, lo, hi) for c in cells]
        tp = rep.transit(robot)
        # traversal weight: distance between the two end vertices; coverage weight: path length in the cell
        t_w = [max(tp.length(pos[a], pos[b]), 1e-9) for a, b in edges]
        c_w = [max(float(c.area) / w, t) for c, t in zip(cells, t_w)]
        return {"cells": cells, "pos": pos, "edges": edges, "passes": passes, "tw": t_w, "cw": c_w}

    def summary(self, out):
        return {"cells": len(out["cells"]), "reeb_vertices": int(len(out["pos"]))}


def eulerian_tour(n, edges, tw, cw, start: int):
    """Chinese-postman tour of the Reeb graph: [(edge index, from, to, covers)]."""
    G = nx.Graph()
    G.add_nodes_from(range(n))
    for e, (a, b) in enumerate(edges):
        if a == b:
            continue
        if G.has_edge(a, b) and G[a][b]["weight"] <= tw[e]:
            continue
        G.add_edge(a, b, weight=tw[e], edge=e)
    M = nx.MultiGraph()
    M.add_nodes_from(range(n))
    for e, (a, b) in enumerate(edges):
        M.add_edge(a, b, edge=e)
    odd = [v for v in M.nodes if M.degree(v) % 2 == 1]
    if odd:
        dist, paths = {}, {}
        for u in odd:
            d, p = nx.single_source_dijkstra(G, u, weight="weight")
            dist[u], paths[u] = d, p
        K = nx.Graph()
        for i, u in enumerate(odd):
            for v in odd[i + 1:]:
                K.add_edge(u, v, weight=dist[u].get(v, math.inf))
        for u, v in nx.min_weight_matching(K):
            p = paths[u][v]
            for x, y in zip(p[:-1], p[1:]):
                M.add_edge(x, y, edge=G[x][y]["edge"])
    if not nx.is_connected(M):
        comp = nx.node_connected_component(M, start)
        M = M.subgraph(comp).copy()
    circuit = list(nx.eulerian_circuit(M, source=start, keys=True)) if M.number_of_edges() else []
    seen = set()
    out = []
    for a, b, k in circuit:
        e = M[a][b][k]["edge"]
        out.append((e, a, b, e not in seen))
        seen.add(e)
    return out


def fhk_split(tour, cost, s_home, k: int):
    """Frederickson-Hecht-Kim split of a tour into k sub-tours (lists of tour positions)."""
    m = len(tour)
    if k <= 1 or m == 0:
        return [list(range(m))] + [[] for _ in range(k - 1)]
    c = np.array([cost(t) for t in tour])
    R = np.r_[0.0, np.cumsum(c)]  # prefix cost at tour vertex j (before edge j)
    verts = [t[1] for t in tour] + [tour[-1][2]]
    s = np.array([s_home(v) for v in verts])
    s_max = max(s[j] + c[j] + s[j + 1] for j in range(m))
    C = R[-1]
    cuts = [0]
    for j in range(1, k):
        L = j * (C - 2 * s_max) / k + s_max
        p = int(np.searchsorted(R, L, side="right") - 1)
        p = min(max(p, cuts[-1]), m)
        if p < m:
            r = L - R[p]
            if r + s[p] > c[p] - r + s[p + 1]:  # finishing the edge is cheaper
                p += 1
        cuts.append(min(p, m))
    cuts.append(m)
    return [list(range(cuts[i], cuts[i + 1])) for i in range(k)]


class EulerianTour(Stage):
    kind = "route_construction"
    name = "eulerian_tour"
    consumes = ("model",)
    produces = "tour"

    def run(self, data, rep, ctx, params):
        md = data["model"]
        pos = md["pos"]
        r0 = rep.team[0]
        start = int(np.argmin(np.hypot(*(pos - np.asarray(r0.start[:2])).T)))
        tour = eulerian_tour(len(pos), md["edges"], md["tw"], md["cw"], start)
        return {"tour": tour, "start": start}

    def summary(self, out):
        return {"tour_edges": len(out["tour"]), "covered_cells": int(sum(t[3] for t in out["tour"]))}


class KPostmanSplit(Stage):
    kind = "work_distribution"
    name = "fhk_split"
    consumes = ("tour", "model")
    produces = "routes"

    def run(self, data, rep, ctx, params):
        md, td = data["model"], data["tour"]
        tour, start = td["tour"], td["start"]
        pos, tw, cw = md["pos"], md["tw"], md["cw"]
        k = len(rep.team)
        G = nx.Graph()
        for e, (a, b) in enumerate(md["edges"]):
            if a != b and (not G.has_edge(a, b) or G[a][b]["weight"] > tw[e]):
                G.add_edge(a, b, weight=tw[e])
        G.add_nodes_from(range(len(pos)))
        home = nx.single_source_dijkstra_path_length(G, start, weight="weight")
        cost = (lambda t: cw[t[0]] if t[3] else tw[t[0]])  # noqa: E731
        parts = fhk_split(tour, cost, lambda v: home.get(v, 0.0), k)
        # sub-tour -> robot (bottleneck assignment of the robots' own travel times)
        tp = rep.transit(rep.team[0])
        C = np.zeros((k, k))
        for j, part in enumerate(parts):
            body = sum(cost(tour[t]) for t in part)
            if not part:
                continue
            a, b = pos[tour[part[0]][1]], pos[tour[part[-1]][2]]
            for i, r in enumerate(rep.team):
                C[i, j] = (tp.length(r.start[:2], a) + body + tp.length(b, r.start[:2])) / r.speed
        ri, ci = linear_sum_assignment((C / max(C.max(), 1e-9)) ** 4)
        assign = dict(zip(ri, ci))
        jobs, flat, owner = {}, [], {}
        for i, r in enumerate(rep.team):
            seq = []
            for t in parts[assign[i]]:
                e, a, b, covers = tour[t]
                if not covers:
                    continue
                lo, hi = md["edges"][e]
                ps = md["passes"][e] if a == lo else md["passes"][e][::-1]
                rev = False
                for pts in ps:
                    flat.append(Pass(pts, False, e))
                    seq.append((len(flat) - 1, rev))
                    owner[len(flat) - 1] = i
                    rev = not rev
            jobs[r.id] = seq
        return {"jobs": jobs, "passes": flat, "parts": parts, "estimated": C[ri, ci].tolist()}

    def metrics(self, out, rep):
        est = np.array(out["estimated"], dtype=float)
        return {"estimated_makespan": float(est.max()) if len(est) else 0.0,
                "cells_per_robot": [len(p) for p in out["parts"]]}


class KarapetyanCRC(StagedPlanner):
    manifest = Manifest(
        name="crc",
        version="1.0",
        reference="Karapetyan, Benson, McKinney, Taslakian & Rekleitis (2017), Efficient multi-robot coverage of "
                  "a known environment (Coverage with Route Clustering)",
        family="geometric",
        adapter="geometric",
        adapter_params={"mode": "boustrophedon", "angle": "auto"},
        constraints={"multi_robot", "return_to_depot"},
        mode="staged",
        stages=["reeb_graph", "eulerian_tour", "fhk_split"],
        hyperparameters=[],
        core_modules=["uninbench/planners/karapetyan.py", "uninbench/planners/sweep_common.py",
                      "uninbench/geometry/decomposition.py"],
        external_dependencies=[],
        description="Boustrophedon cells, Chinese-postman tour of the Reeb graph, Frederickson k-postman split.",
    )
    stages = [ReebGraphModel(), EulerianTour(), KPostmanSplit()]

    def finish(self, data, rep, ctx) -> PlanResult:
        passes = data["routes"]["passes"]
        ts = TrajectorySet(metadata={"planner": self.manifest.name})
        for r in rep.team:
            rb = RouteBuilder(rep, r, self.manifest, ctx, service_links=False)
            ts.add(rb.run(data["routes"]["jobs"].get(r.id, []), passes, return_to_depot=ctx.return_to_depot))
        layers = rep.layers()
        pos = data["model"]["pos"]
        layers["reeb_graph"] = {"type": "segments", "coords": [[np.round(pos[a], 4).tolist(), np.round(pos[b], 4).tolist()]
                                                               for a, b in data["model"]["edges"] if a != b]}
        covered = sorted({int(passes[i].group) for jobs in data["routes"]["jobs"].values() for i, _ in jobs})
        with_passes = [k for k, ps in enumerate(data["model"]["passes"]) if ps]
        return PlanResult(ts, native=data["routes"]["jobs"], layers=layers,
                          info={"estimated_times": data["routes"]["estimated"], "covered_cells": covered,
                                "cells_with_passes": with_passes})

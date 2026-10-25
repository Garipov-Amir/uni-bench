"""SCoPP -- Scalable Coverage Path Planning (Collins, Ghassemi, Esfahani, Doermann, Dantu & Chowdhury, 2021).

Geometric family (the planner receives the polygon with holes and decomposes it
itself), staged mode, stochastic:

1. **Discretisation (Step 2)**: the area is covered by square cells of width
   ``W`` such that the footprint at the cell centre covers the cell (for a disc
   tool ``W = w / sqrt 2``); cells whose centre is reachable are kept, and the
   perimeter of every cell is represented by evenly spaced points.
   For ground robots, cells whose centre the body cannot reach are surveyed
   from the nearest reachable point.
2. **Partitioning (Step 3)**: Lloyd's algorithm clusters all perimeter points
   into one cluster per robot (at most ten iterations, tolerance ``W / 8``);
   clusters are matched to robots by their distance to the robots' initial
   positions.
3. **Conflict auction (Step 4, Algorithm 1)**: cells whose perimeter points lie
   in several clusters are auctioned: a cell goes to the robot with the lowest
   cell count plus distance bias ``d_B = B * d_0`` (``d_0``: distance from the
   robot's initial position to its closest assigned cell, in cell widths).
4. **Path planning (Step 5, Algorithm 2)**: every robot visits its cell centres
   in nearest-neighbour order (k-d tree) from its initial position.

Like the original, SCoPP does not distinguish service from transit (all
segments are coverage).  The paper plans for UAVs over convex areas; for ground
robots, waypoints that are not mutually visible are joined by the shortest
collision-free path.
"""

from __future__ import annotations

import math

import numpy as np
import shapely
import shapely.ops
from scipy.optimize import linear_sum_assignment
from scipy.spatial import cKDTree

from ..trajectory import SERVICE, TrajectoryBuilder, TrajectorySet
from .base import HyperParam, Manifest, PlanResult, Stage, StagedPlanner


def cell_width(robot) -> float:
    tool = robot.tool
    if tool.kind == "disc":
        return tool.working_width / math.sqrt(2.0)
    return min(tool.working_width, tool.along)


class Discretisation(Stage):
    kind = "environment_model"
    name = "square_cells"
    consumes = ("representation",)
    produces = "model"

    def run(self, data, rep, ctx, params):
        robot = rep.team[0]
        W = cell_width(robot) * 0.999
        cs = rep.cspace(robot)
        minx, miny, maxx, maxy = rep.free.bounds
        xs = np.arange(minx + W / 2, maxx + W / 2, W)
        ys = np.arange(miny + W / 2, maxy + W / 2, W)
        X, Y = np.meshgrid(xs, ys)
        C = np.c_[X.ravel(), Y.ravel()]
        shapely.prepare(cs)
        free = rep.free
        shapely.prepare(free)
        inside = shapely.contains_xy(cs, C[:, 0], C[:, 1]) if len(C) else np.zeros(0, dtype=bool)
        # ground robots: a cell of the area whose centre the body cannot reach is surveyed from the
        # nearest reachable point (UAVs in the paper reach every cell centre)
        edge = C[~inside & shapely.contains_xy(free, C[:, 0], C[:, 1])] if len(C) else C
        moved = []
        inner = cs.buffer(-1e-6 * max(1.0, W))
        for q in edge:
            nq = shapely.ops.nearest_points(inner, shapely.Point(q))[0]
            moved.append((nq.x, nq.y))
        C = C[inside]
        if moved:
            M = np.asarray(moved)
            if len(C):
                d, _ = cKDTree(C).query(M)
                M = M[d > 0.5 * W]
            if len(M):
                keep = [0]
                for t in range(1, len(M)):
                    if np.min(np.hypot(*(M[keep] - M[t]).T)) > 0.5 * W:
                        keep.append(t)
                C = np.r_[C, M[keep]] if len(C) else M[keep]
        # the perimeter of every cell is a series of evenly spaced points
        per_side = max(1, int(params["perimeter_points"]) // 4)
        t = np.linspace(-0.5, 0.5, per_side, endpoint=False) * W
        h = W / 2
        ring = np.r_[np.c_[t, np.full_like(t, -h)], np.c_[np.full_like(t, h), t],
                     np.c_[-t, np.full_like(t, h)], np.c_[np.full_like(t, -h), -t]]
        pts = (C[:, None, :] + ring[None, :, :]).reshape(-1, 2)
        owner = np.repeat(np.arange(len(C)), len(ring))
        return {"centers": C, "points": pts, "owner": owner, "W": W}

    def summary(self, out):
        return {"cells": int(len(out["centers"])), "points": int(len(out["points"]))}


def lloyd(points: np.ndarray, k: int, rng, max_iter: int, tol: float):
    """Lloyd's algorithm with k-means++ seeding; returns (labels, centres)."""
    n = len(points)
    k = min(k, n)
    centres = [points[rng.integers(n)]]
    for _ in range(1, k):
        d2 = np.min(((points[:, None, :] - np.array(centres)[None]) ** 2).sum(-1), axis=1)
        centres.append(points[rng.choice(n, p=d2 / d2.sum())] if d2.sum() > 0 else points[rng.integers(n)])
    centres = np.array(centres, dtype=float)
    lab = np.zeros(n, dtype=int)
    for _ in range(max_iter):
        lab = cKDTree(centres).query(points)[1]
        new = np.array([points[lab == j].mean(axis=0) if np.any(lab == j) else centres[j] for j in range(k)])
        shift = float(np.max(np.hypot(*(new - centres).T)))
        centres = new
        if shift <= tol:
            break
    lab = cKDTree(centres).query(points)[1]
    return lab, centres


class LloydPartition(Stage):
    kind = "work_distribution"
    name = "lloyd_auction"
    consumes = ("model",)
    produces = "assignment"

    def run(self, data, rep, ctx, params):
        md = data["model"]
        C, P, owner, W = md["centers"], md["points"], md["owner"], md["W"]
        robots = rep.team
        k = len(robots)
        if len(C) == 0:
            return {"cells": {r.id: np.zeros((0, 2)) for r in robots}, "conflicts": 0}
        lab, centres = lloyd(P, k, ctx.rng, int(params["max_iter"]), W / 8)
        # clusters -> robots by distance of the robots' initial positions to the cluster centres
        starts = np.array([r.start[:2] for r in robots])
        D = np.hypot(starts[:, None, 0] - centres[None, :, 0], starts[:, None, 1] - centres[None, :, 1])
        ri, ci = linear_sum_assignment(D)
        robot_of_cluster = {int(c): int(r) for r, c in zip(ri, ci)}
        n = len(C)
        per_cell = [set() for _ in range(n)]
        for p, c in zip(owner, lab):
            per_cell[p].add(robot_of_cluster.get(int(c), 0))
        assign = np.full(n, -1)
        conflicts = []
        for c in range(n):
            if len(per_cell[c]) == 1:
                assign[c] = next(iter(per_cell[c]))
            else:
                conflicts.append(c)
        counts = np.bincount(assign[assign >= 0], minlength=k).astype(float)
        B = float(params["bias"])
        for c in conflicts:  # Algorithm 1 (conflict resolution), as published
            bias = np.zeros(k)
            for r in range(k):
                mine = C[assign == r]
                d0 = float(np.min(np.hypot(*(mine - starts[r]).T))) if len(mine) else 0.0
                bias[r] = B * d0 / W
            lowest, who = math.inf, None
            for r in range(k):
                if counts[r] + bias[r] < lowest:
                    lowest = counts[r]
                    who = r
            who = 0 if who is None else who
            assign[c] = who
            counts[who] += 1
        return {"cells": {r.id: C[assign == i] for i, r in enumerate(robots)}, "conflicts": len(conflicts),
                "counts": counts.astype(int).tolist()}

    def metrics(self, out, rep):
        c = np.array(out.get("counts", [0]), dtype=float)
        return {"conflict_cells": float(out["conflicts"]),
                "imbalance": float(c.max() / c.mean()) if c.mean() > 0 else None}


class NearestNeighbour(Stage):
    kind = "route_construction"
    name = "nn_path"
    consumes = ("assignment",)
    produces = "routes"

    def run(self, data, rep, ctx, params):
        out = {}
        for r in rep.team:
            cells = data["assignment"]["cells"][r.id]
            pos = np.asarray(r.start[:2], dtype=float)
            order = []
            if len(cells):
                left = np.ones(len(cells), dtype=bool)
                tree = cKDTree(cells, leafsize=10)
                for _ in range(len(cells)):
                    kq = min(len(cells), 16)
                    while True:
                        _, idx = tree.query(pos, k=kq)
                        idx = np.atleast_1d(idx)
                        ok = [int(i) for i in idx if i < len(cells) and left[i]]
                        if ok or kq >= len(cells):
                            break
                        kq = min(len(cells), kq * 4)
                    j = ok[0] if ok else int(np.flatnonzero(left)[0])
                    order.append(j)
                    left[j] = False
                    pos = cells[j]
            out[r.id] = cells[order] if len(cells) else cells
        return {"waypoints": out}


class SCoPP(StagedPlanner):
    manifest = Manifest(
        name="scopp",
        version="1.0",
        reference="Collins, Ghassemi, Esfahani, Doermann, Dantu & Chowdhury (2021), Scalable coverage path "
                  "planning of multi-robot teams for monitoring non-convex areas",
        family="geometric",
        adapter="geometric",
        adapter_params={"mode": "polygon"},
        constraints={"multi_robot", "return_to_depot"},
        stochastic=True,
        distinguishes_service_transit=False,
        mode="staged",
        stages=["square_cells", "lloyd_auction", "nn_path"],
        hyperparameters=[
            HyperParam("bias", "float", 0.5, low=0.0, high=2.0, doc="B, distance bias of the conflict auction"),
            HyperParam("max_iter", "int", 10, low=2, high=100, log=True, doc="Lloyd iterations"),
            HyperParam("perimeter_points", "int", 8, low=4, high=32, doc="points on the perimeter of every cell"),
        ],
        core_modules=["uninbench/planners/scopp.py"],
        description="Square cells, Lloyd clustering of cell perimeters, conflict auction, nearest-neighbour paths.",
    )
    stages = [Discretisation(), LloydPartition(), NearestNeighbour()]

    def finish(self, data, rep, ctx) -> PlanResult:
        ts = TrajectorySet(metadata={"planner": self.manifest.name})
        cs = rep.cspace(rep.team[0]).buffer(1e-6 * max(1.0, rep.team[0].working_width))
        shapely.prepare(cs)
        for r in rep.team:
            b = TrajectoryBuilder(r.id, *r.start, tool_active_in_turns=self.manifest.tool_active_in_turns)
            wps = data["routes"]["waypoints"][r.id]
            tp = None
            for q in wps:
                if cs.covers(shapely.LineString([b.pos, tuple(q)])):
                    b.move_to(q[0], q[1], SERVICE)
                else:
                    tp = tp or rep.transit(r)
                    b.follow(tp.path(b.pos, q), SERVICE)
            if ctx.return_to_depot:
                depot = r.depot if r.depot is not None else r.start[:2]
                tp = tp or rep.transit(r)
                b.follow(tp.path(b.pos, depot), SERVICE)
            ts.add(b.build())
        layers = rep.layers()
        C = data["model"]["centers"]
        lab = np.full(len(C), -1)
        idx = {tuple(np.round(c, 9)): k for k, c in enumerate(C)}
        for i, r in enumerate(rep.team):
            for c in data["assignment"]["cells"][r.id]:
                lab[idx[tuple(np.round(c, 9))]] = i
        layers["cells"] = {"type": "point_labels", "coords": np.round(C, 4).tolist(), "labels": lab.tolist()}
        return PlanResult(ts, native={rid: w.tolist() for rid, w in data["routes"]["waypoints"].items()},
                          layers=layers, info={"conflict_cells": data["assignment"]["conflicts"]})

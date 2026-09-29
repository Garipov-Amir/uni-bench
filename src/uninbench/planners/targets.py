"""Target-based planner: balanced k-means clustering + TSP tours (black-box, stochastic).

Coverage targets are clustered into m balanced clusters (seeded k-means++),
clusters are assigned to robots, and every robot visits its targets in a
nearest-neighbour order improved by 2-opt.  The planner does not distinguish
service from transit, so all its segments are labelled service (treated
conservatively by the evaluator).
"""

from __future__ import annotations

import numpy as np
import shapely
from scipy.optimize import linear_sum_assignment
from scipy.spatial import cKDTree

from ..trajectory import SERVICE, TRANSIT, TrajectoryBuilder, TrajectorySet
from .base import HyperParam, Manifest, PlanContext, Planner, PlanResult
from .deconflict import maybe_deconflict


def balanced_kmeans(pts: np.ndarray, m: int, rng, iters: int = 50, balance: float = 0.5) -> np.ndarray:
    n = len(pts)
    if n == 0:
        return np.zeros(0, dtype=int)
    m_eff = min(m, n)
    centers = [pts[rng.integers(n)]]
    for _ in range(1, m_eff):
        d2 = np.min(((pts[:, None, :] - np.array(centers)[None]) ** 2).sum(-1), axis=1)
        p = d2 / d2.sum() if d2.sum() > 0 else None
        centers.append(pts[rng.choice(n, p=p)])
    centers = np.array(centers)
    w = np.ones(m_eff)
    lab = np.zeros(n, dtype=int)
    for _ in range(iters):
        D = np.hypot(pts[:, None, 0] - centers[None, :, 0], pts[:, None, 1] - centers[None, :, 1]) * w[None]
        lab = np.argmin(D, axis=1)
        sizes = np.bincount(lab, minlength=m_eff).astype(float)
        for k in range(m_eff):
            if sizes[k] > 0:
                centers[k] = pts[lab == k].mean(axis=0)
        w *= np.maximum(sizes / (n / m_eff), 0.1) ** (0.5 * balance)
        w /= w.mean()
    return lab


def nn_tour(start: np.ndarray, pts: np.ndarray) -> np.ndarray:
    n = len(pts)
    if n == 0:
        return np.zeros(0, dtype=int)
    tree = cKDTree(pts)
    left = np.ones(n, dtype=bool)
    order = []
    pos = start
    k = 8
    for _ in range(n):
        while True:
            _, idx = tree.query(pos, k=min(k, n))
            idx = np.atleast_1d(idx)
            ok = idx[left[idx]]
            if len(ok):
                j = int(ok[0])
                break
            if k >= n:
                j = int(np.flatnonzero(left)[np.argmin(np.hypot(*(pts[left] - pos).T))])
                break
            k *= 4
        order.append(j)
        left[j] = False
        pos = pts[j]
    return np.array(order)


def two_opt_path(P: np.ndarray, rounds: int, deadline=None) -> np.ndarray:
    """2-opt for an open path P[0] -> ... -> P[-1] with fixed first node."""
    n = len(P)
    order = np.arange(n)
    if n < 4:
        return order
    for _ in range(rounds):
        improved = False
        for i in range(1, n - 1):
            Q = P[order]
            a, b = Q[i - 1], Q[i]
            js = np.arange(i + 1, n)
            c = Q[js]
            nxt = np.r_[js[:-1] + 1, -1]
            d_ab = np.hypot(*(b - a))
            d_ac = np.hypot(*(c - a).T)
            has_next = nxt >= 0
            dnext = np.zeros(len(js))
            d_cd = np.zeros(len(js))
            d_bd = np.zeros(len(js))
            dd = Q[nxt[has_next]]
            d_cd[has_next] = np.hypot(*(dd - c[has_next]).T)
            d_bd[has_next] = np.hypot(*(dd - b).T)
            gain = d_ab + d_cd - d_ac - d_bd - dnext
            k = int(np.argmax(gain))
            if gain[k] > 1e-9:
                j = js[k]
                order[i:j + 1] = order[i:j + 1][::-1]
                improved = True
            if deadline is not None and deadline():
                return order
        if not improved:
            break
    return order


class KMeansTSP(Planner):
    manifest = Manifest(
        name="kmeans_tsp",
        version="1.0",
        reference="Target-based baseline: balanced k-means partition + TSP (NN + 2-opt) per robot",
        family="target",
        adapter="target",
        adapter_params={"lattice": "square"},
        constraints={"multi_robot", "return_to_depot", "deconfliction"},
        stochastic=True,
        distinguishes_service_transit=False,
        hyperparameters=[
            HyperParam("kmeans_iters", "int", 50, low=5, high=300, log=True, doc="balanced k-means iterations"),
            HyperParam("balance", "float", 0.5, low=0.0, high=2.0, doc="strength of the cluster-size balancing"),
            HyperParam("two_opt_rounds", "int", 5, low=0, high=30, doc="2-opt rounds per tour (0 disables 2-opt)"),
        ],
        core_modules=["uninbench/planners/targets.py"],
        description="Balanced k-means over coverage targets and a 2-opt improved tour per cluster.",
    )

    def plan(self, rep, ctx: PlanContext) -> PlanResult:
        pts = rep.targets
        robots = rep.team
        m = len(robots)
        lab = balanced_kmeans(pts, m, ctx.rng, int(self.params["kmeans_iters"]), float(self.params["balance"]))
        k_eff = int(lab.max()) + 1 if len(lab) else 0
        starts = np.array([r.start[:2] for r in robots])
        cost = np.zeros((m, max(k_eff, 1)))
        for k in range(k_eff):
            c = pts[lab == k].mean(axis=0)
            cost[:, k] = np.hypot(*(starts - c).T)
        ri, ci = linear_sum_assignment(cost) if k_eff else ([], [])
        assign = dict(zip(ri, ci))
        ts = TrajectorySet(metadata={"planner": self.manifest.name})
        cs_tol = rep.cspace(robots[0]).buffer(1e-6 * max(1.0, robots[0].working_width))
        shapely.prepare(cs_tol)
        tours = {}
        for k, r in enumerate(robots):
            b = TrajectoryBuilder(r.id, *r.start, tool_active_in_turns=self.manifest.tool_active_in_turns)
            if k in assign:
                sub = pts[lab == assign[k]]
                order = nn_tour(np.asarray(r.start[:2]), sub)
                P = np.r_[[r.start[:2]], sub[order]]
                opt = two_opt_path(P, int(self.params["two_opt_rounds"]), lambda: ctx.out_of_time(0.8))
                seq = P[opt]
                tours[r.id] = seq[1:]
                lines = shapely.linestrings(np.stack([seq[:-1], seq[1:]], 1))
                free = shapely.covers(cs_tol, lines)
                tp = rep.transit(r)
                for q, ok in zip(seq[1:], free):
                    if ok:
                        b.move_to(q[0], q[1], SERVICE)
                    else:
                        b.follow(tp.path(b.pos, q), SERVICE)
            if ctx.return_to_depot:
                depot = r.depot if r.depot is not None else r.start[:2]
                b.follow(rep.transit(r).path(b.pos, depot), SERVICE if not self.manifest.distinguishes_service_transit
                         else TRANSIT)
            ts.add(b.build())
        ts, dinfo = maybe_deconflict(ts, rep, ctx)
        layers = rep.layers()
        layers["clusters"] = {"type": "point_labels", "coords": np.round(pts, 4).tolist(), "labels": lab.tolist()}
        return PlanResult(ts, native={rid: t.tolist() for rid, t in tours.items()}, layers=layers,
                          info={"clusters": k_eff, **dinfo})

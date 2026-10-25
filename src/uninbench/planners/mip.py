"""MIP for time-optimal MCPP (Tang & Ma, 2023) -- staged mode.

The Rooted Min-Max Tree Cover problem on the terrain graph is solved with the
mixed-integer programme of the paper: binary edge and vertex variables per
robot, a makespan variable, cover and rooted constraints, and the flow-based
tree constraints (Eqns. 2-8); every robot then circumnavigates its tree with STC.

Model reduction heuristics (Sec. IV): the Parabolic Removal Heuristic (PRH,
parameter alpha) and the Subgraph Removal Heuristic (SRH, parameter beta)
remove, for every robot, the variables of an inferior graph that its tree is
unlikely to cover; a connectivity check keeps the residual graph of every root
connected.  With ``heuristic = "auto"`` the planner follows the paper's
procedure: the original model for small instances, otherwise a short search
over alpha, beta in {0.3, 0.6, 0.9} (2 % of the time limit each) and the model
with the smallest lower bound is solved with the remaining time.

Solver: HiGHS as bundled with SciPy (the paper uses Gurobi).  As in the
paper, the solver is warm-started: the full model with the MFC tree cover
(modified Rooted-Tree-Cover), the reduced models with the minimum spanning
trees of the robots' residual graphs.  Should the bundled HiGHS interface be
unavailable, :func:`scipy.optimize.milp` is used without a start solution and
the MFC cover is returned when the solver finds nothing better in time.
"""

from __future__ import annotations

import heapq
import math
import time

import numpy as np
from scipy.optimize import Bounds, LinearConstraint, milp
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components, dijkstra

from .base import HyperParam, Manifest, Stage
from .mfc import STCPerTree, _TreeCoverPlanner, rtc_min_max, tree_cover_by_component
from .mstc_star import TerrainGraph
from .tree_cover import kruskal

GRID = (0.3, 0.6, 0.9)


def _logistic(x: float) -> float:
    return 1.0 / (1.0 + math.exp(-x))


# ------------------------------------------------------------------ model
class RMMTCModel:
    """MIP of the rooted min-max tree cover restricted to ``allowed[i]`` vertices per robot.

    The cost of a tree is the sum of its edge weights ``w`` plus, optionally, the sum of its
    vertex weights ``wv`` (MCFS, where an isovertex weighs the length of its isoline)."""

    def __init__(self, n, edges, w, roots, allowed=None, wv=None):
        self.n, self.edges, self.w, self.roots = n, edges, w, roots
        self.wv = None if wv is None else np.asarray(wv, dtype=float)
        k = len(roots)
        self.allowed = allowed or [np.ones(n, dtype=bool) for _ in range(k)]
        self.V, self.E = [], []
        for i in range(k):
            a = self.allowed[i].copy()
            a[roots[i]] = True
            self.V.append(np.flatnonzero(a))
            self.E.append(np.flatnonzero(a[edges[:, 0]] & a[edges[:, 1]]))
        # variable layout: [x_i | y_i | fu_i | fv_i] per robot, then tau
        self.off = []
        o = 0
        for i in range(k):
            ne, nv = len(self.E[i]), len(self.V[i])
            self.off.append((o, o + ne, o + ne + nv, o + 2 * ne + nv))
            o += 3 * ne + nv
        self.tau = o
        self.nvar = o + 1

    def build(self):
        k, n = len(self.roots), self.n
        rows, cols, vals, lb, ub = [], [], [], [], []
        r = 0

        def add(cs, vs, lo, hi):
            nonlocal r
            rows.extend([r] * len(cs))
            cols.extend(cs)
            vals.extend(vs)
            lb.append(lo)
            ub.append(hi)
            r += 1

        vpos = []
        for i in range(k):
            ox, oy, ofu, ofv = self.off[i]
            Ei, Vi = self.E[i], self.V[i]
            pos = np.full(n, -1)
            pos[Vi] = np.arange(len(Vi))
            vpos.append(pos)
            # makespan: sum_e w_e x_e (+ sum_v w_v y_v) - tau <= 0
            cols_m = list(ox + np.arange(len(Ei)))
            vals_m = list(self.w[Ei])
            if self.wv is not None:
                cols_m += list(oy + np.arange(len(Vi)))
                vals_m += list(self.wv[Vi])
            add(cols_m + [self.tau], vals_m + [-1.0], -np.inf, 0.0)
            # tree: sum_v y_v - sum_e x_e = 1
            add(list(oy + np.arange(len(Vi))) + list(ox + np.arange(len(Ei))),
                [1.0] * len(Vi) + [-1.0] * len(Ei), 1.0, 1.0)
        # cover: sum_i y_v^i >= 1
        for v in range(n):
            cs = [self.off[i][1] + vpos[i][v] for i in range(k) if vpos[i][v] >= 0]
            add(cs, [1.0] * len(cs), 1.0, np.inf)
        for i in range(k):
            ox, oy, ofu, ofv = self.off[i]
            Ei = self.E[i]
            pos = vpos[i]
            eu = pos[self.edges[Ei, 0]]
            ev = pos[self.edges[Ei, 1]]
            inc = [[] for _ in range(len(self.V[i]))]  # flows into every vertex
            for t in range(len(Ei)):
                add([ofu + t, ofv + t, ox + t], [1.0, 1.0, -1.0], 0.0, 0.0)  # f_eu + f_ev = x_e
                add([ox + t, oy + eu[t]], [1.0, -1.0], -np.inf, 0.0)  # x_e <= y_u
                add([ox + t, oy + ev[t]], [1.0, -1.0], -np.inf, 0.0)  # x_e <= y_v
                inc[eu[t]].append(ofu + t)
                inc[ev[t]].append(ofv + t)
            for cs in inc:  # sum of flows into v <= 1 - 1/n
                if cs:
                    add(cs, [1.0] * len(cs), -np.inf, 1.0 - 1.0 / n)
        A = coo_matrix((vals, (rows, cols)), shape=(r, self.nvar)).tocsr()
        integrality = np.zeros(self.nvar)
        lo = np.zeros(self.nvar)
        hi = np.ones(self.nvar)
        for i in range(k):
            ox, oy, ofu, ofv = self.off[i]
            integrality[ox:ofu] = 1  # x and y binary
            lo[oy + vpos[i][self.roots[i]]] = 1.0  # rooted
        hi[self.tau] = np.inf
        c = np.zeros(self.nvar)
        c[self.tau] = 1.0
        return c, LinearConstraint(A, np.array(lb), np.array(ub)), integrality, Bounds(lo, hi)

    def start_vector(self, trees) -> np.ndarray | None:
        """Variable values of a tree cover (None if it does not fit the model)."""
        x = np.zeros(self.nvar)
        n = self.n
        tau = 0.0
        for i, (verts, tedges) in enumerate(trees):
            ox, oy, ofu, ofv = self.off[i]
            epos = {int(e): t for t, e in enumerate(self.E[i])}
            vpos = {int(v): t for t, v in enumerate(self.V[i])}
            verts = set(int(v) for v in verts) | {self.roots[i]}
            if any(v not in vpos for v in verts) or any(int(e) not in epos for e in tedges):
                return None
            for v in verts:
                x[oy + vpos[v]] = 1.0
            adj = {}
            for e in tedges:
                a, b = int(self.edges[e, 0]), int(self.edges[e, 1])
                x[ox + epos[int(e)]] = 1.0
                adj.setdefault(a, []).append((b, int(e)))
                adj.setdefault(b, []).append((a, int(e)))
            # flows by leaf peeling from the root: a vertex with a subtree of s vertices receives
            # 1 - s/n from the edge to its parent, the parent s/n (feasible for trees with <= n vertices)
            parent, order = {self.roots[i]: (None, None)}, [self.roots[i]]
            for u in order:
                for v, e in adj.get(u, []):
                    if v not in parent:
                        parent[v] = (u, e)
                        order.append(v)
            size = {v: 1 for v in order}
            for v in reversed(order[1:]):
                size[parent[v][0]] += size[v]
            for v in order[1:]:
                p_, e = parent[v]
                t = epos[e]
                into_v = 1.0 - size[v] / n
                a = int(self.edges[e, 0])
                if a == v:
                    x[ofu + t], x[ofv + t] = into_v, 1.0 - into_v
                else:
                    x[ofu + t], x[ofv + t] = 1.0 - into_v, into_v
            cost = float(np.sum(self.w[list(tedges)])) if len(tedges) else 0.0
            if self.wv is not None:
                cost += float(np.sum(self.wv[list(verts)]))
            tau = max(tau, cost)
        x[self.tau] = tau
        return x

    def residual_msts(self):
        """Minimum spanning tree of every robot's residual graph (warm start of reduced models)."""
        out = []
        for i in range(len(self.roots)):
            Ei = self.E[i]
            idx = kruskal(self.n, self.edges[Ei], self.w[Ei]) if len(Ei) else []
            te = [int(Ei[j]) for j in idx]
            comp = _component(self.n, self.edges[te], self.roots[i])
            out.append((comp, [e for e in te if int(self.edges[e, 0]) in comp]))
        return out

    def solve(self, time_limit: float, start=None):
        """Solve with an optional start cover; returns (trees, lower bound, info)."""
        c, cons, integ, bnds = self.build()
        x0 = self.start_vector(start) if start is not None else None
        if x0 is None and start is not None:
            x0 = self.start_vector(self.residual_msts())
        res = _highs(c, cons, integ, bnds, x0, time_limit)
        if res is None:  # bundled HiGHS interface unavailable
            m = milp(c, constraints=cons, integrality=integ, bounds=bnds,
                     options={"time_limit": max(0.1, float(time_limit)), "disp": False, "mip_rel_gap": 1e-6})
            res = {"x": m.x, "bound": getattr(m, "mip_dual_bound", None), "status": str(m.message),
                   "warm_start": False}
        if res["x"] is None:
            return None, res["bound"], res
        return self.trees(np.asarray(res["x"])), res["bound"], res

    def trees(self, x):
        out = []
        for i in range(len(self.roots)):
            ox, oy, ofu, ofv = self.off[i]
            Ei, Vi = self.E[i], self.V[i]
            e_sel = Ei[x[ox:ox + len(Ei)] > 0.5]
            v_sel = set(Vi[x[oy:oy + len(Vi)] > 0.5].tolist()) | {self.roots[i]}
            # keep the component of the root (numerically clean tree)
            idx = kruskal(self.n, self.edges[e_sel], self.w[e_sel]) if len(e_sel) else []
            e_sel = e_sel[idx] if len(idx) else np.zeros(0, dtype=int)
            comp = _component(self.n, self.edges[e_sel], self.roots[i])
            e_keep = [int(e) for e in e_sel if int(self.edges[e, 0]) in comp]
            out.append(({v for v in v_sel if v in comp}, e_keep))
        return out


def _highs(c, cons, integ, bnds, x0, time_limit):
    """Solve with the HiGHS build bundled with SciPy (supports a start solution)."""
    try:
        from scipy.optimize._highspy import _core
    except ImportError:  # pragma: no cover
        return None
    try:
        A = cons.A.tocsc()
        h = _core._Highs()
        h.setOptionValue("output_flag", False)
        h.setOptionValue("time_limit", max(0.1, float(time_limit)))
        h.setOptionValue("mip_rel_gap", 1e-6)
        h.setOptionValue("threads", 1)
        lp = _core.HighsLp()
        lp.num_col_, lp.num_row_ = A.shape[1], A.shape[0]
        lp.col_cost_ = np.asarray(c, dtype=float)
        lp.col_lower_ = np.asarray(bnds.lb, dtype=float)
        lp.col_upper_ = np.where(np.isfinite(bnds.ub), bnds.ub, _core.kHighsInf).astype(float)
        lp.row_lower_ = np.where(np.isfinite(cons.lb), cons.lb, -_core.kHighsInf).astype(float)
        lp.row_upper_ = np.where(np.isfinite(cons.ub), cons.ub, _core.kHighsInf).astype(float)
        lp.a_matrix_.format_ = _core.MatrixFormat.kColwise
        lp.a_matrix_.start_ = A.indptr
        lp.a_matrix_.index_ = A.indices
        lp.a_matrix_.value_ = A.data
        lp.integrality_ = [_core.HighsVarType.kInteger if v else _core.HighsVarType.kContinuous for v in integ]
        h.passModel(lp)
        warm = False
        if x0 is not None:
            sol = _core.HighsSolution()
            sol.col_value = list(map(float, x0))
            sol.value_valid = True
            warm = h.setSolution(sol) == _core.HighsStatus.kOk
        h.run()
        info = h.getInfo()
        ok = info.primal_solution_status == 2  # kSolutionStatusFeasible
        x = np.asarray(h.getSolution().col_value) if ok else None
        bound = float(info.mip_dual_bound) if np.isfinite(info.mip_dual_bound) else None
        return {"x": x, "bound": bound, "status": h.modelStatusToString(h.getModelStatus()), "warm_start": warm}
    except (AttributeError, TypeError):  # pragma: no cover - private interface changed
        return None


def _component(n, edges, root) -> set:
    adj = {}
    for u, v in edges:
        adj.setdefault(int(u), []).append(int(v))
        adj.setdefault(int(v), []).append(int(u))
    seen, stack = {root}, [root]
    while stack:
        x = stack.pop()
        for y in adj.get(x, []):
            if y not in seen:
                seen.add(y)
                stack.append(y)
    return seen


# ------------------------------------------------------------------ heuristics
def _graph(n, edges, w):
    u, v = edges[:, 0], edges[:, 1]
    return coo_matrix((np.r_[w, w] + 1e-300, (np.r_[u, v], np.r_[v, u])), shape=(n, n)).tocsr()


def _connectivity_check(n, edges, w, root, removed: np.ndarray) -> np.ndarray:
    """Give back vertices so that the residual graph of the root stays connected."""
    removed = removed.copy()
    removed[root] = False
    M = _graph(n, edges, w)
    for _ in range(n):
        keep = ~removed
        sub = np.flatnonzero(keep)
        mask = keep[edges[:, 0]] & keep[edges[:, 1]]
        pos = np.full(n, -1)
        pos[sub] = np.arange(len(sub))
        E = edges[mask]
        S = coo_matrix((np.ones(len(E)), (pos[E[:, 0]], pos[E[:, 1]])), shape=(len(sub), len(sub)))
        ncomp, lab = connected_components(S, directed=False)
        if ncomp <= 1:
            break
        own = lab[pos[root]]
        other = sub[lab != own]
        # shortest path in G from the root to the nearest vertex of a detached component
        dist, pred = dijkstra(M, directed=False, indices=root, return_predecessors=True)
        t = int(other[np.argmin(dist[other])])
        while t != root and t >= 0:
            removed[t] = False
            t = int(pred[t])
    return removed


def _boundary(n, edges) -> np.ndarray:
    deg = np.bincount(edges.ravel(), minlength=n)
    return np.flatnonzero(deg < 4)


def srh(n, edges, w, roots, beta: float) -> list[np.ndarray]:
    """Subgraph Removal Heuristic: allowed vertices per robot."""
    D = dijkstra(_graph(n, edges, w), directed=False, indices=list(roots))
    B = _boundary(n, edges)
    adj = [[] for _ in range(n)]
    for u, v in edges:
        adj[int(u)].append(int(v))
        adj[int(v)].append(int(u))
    allowed = []
    for i, ri in enumerate(roots):
        H = np.zeros(n, dtype=bool)
        for j, rj in enumerate(roots):
            if j == i or rj == ri:
                continue
            S = D[i] > D[j]
            if not S.any():
                continue
            c = int(B[np.argmax(D[i][B] - D[j][B])]) if len(B) else int(np.argmax(D[i] - D[j]))
            dij, djc = D[i][rj], D[j][c]
            b = int(math.ceil(beta * int(S.sum()) * _logistic(dij / djc if djc > 0 else 1e3)))
            if not S[c] or b <= 0:
                continue
            # farthest-first search from c inside S (largest distance to r_i first)
            seen, heap, tree = {c}, [(-D[i][c], c)], []
            while heap and len(tree) <= b:
                _, x = heapq.heappop(heap)
                tree.append(x)
                for y in adj[x]:
                    if S[y] and y not in seen:
                        seen.add(y)
                        heapq.heappush(heap, (-D[i][y], y))
            H[tree] = True
        allowed.append(~_connectivity_check(n, edges, w, ri, H))
    return _ensure_cover(allowed, D)


def prh(n, edges, w, roots, centers: np.ndarray, alpha: float) -> list[np.ndarray]:
    """Parabolic Removal Heuristic: allowed vertices per robot."""
    D = dijkstra(_graph(n, edges, w), directed=False, indices=list(roots))
    B = _boundary(n, edges)
    scale = float(np.median(np.hypot(*np.diff(centers[edges], axis=1)[:, 0].T))) if len(edges) else 1.0
    P = centers / max(scale, 1e-12)  # grid units
    allowed = []
    for i, ri in enumerate(roots):
        H = np.zeros(n, dtype=bool)
        for j, rj in enumerate(roots):
            if j == i or rj == ri:
                continue
            c = int(B[np.argmax(D[i][B] - D[j][B])]) if len(B) else int(np.argmax(D[i] - D[j]))
            dij = D[i][rj]
            a = alpha * _logistic(D[j][c] / dij if dij > 0 else 1e3)
            axis = P[rj] - P[ri]
            L = np.hypot(*axis)
            if L <= 0:
                continue
            ux, uy = axis / L
            rel = P - P[rj]
            yv = rel[:, 0] * ux + rel[:, 1] * uy  # along the ray r_i -> r_j, beyond r_j
            xv = -rel[:, 0] * uy + rel[:, 1] * ux
            inside = yv >= (a * xv) ** 2 if a > 0 else yv >= 0
            H |= inside & (np.arange(n) != rj)
        allowed.append(~_connectivity_check(n, edges, w, ri, H))
    return _ensure_cover(allowed, D)


def _ensure_cover(allowed, D) -> list[np.ndarray]:
    """Every vertex must stay available to some robot (completeness)."""
    A = np.array(allowed)
    lost = np.flatnonzero(~A.any(axis=0))
    for v in lost:
        A[int(np.argmin(D[:, v])), v] = True
    return [a for a in A]


# ------------------------------------------------------------------ planner
def mip_cover(n, edges, w, roots, centers, params, time_limit: float, deadline_t0: float):
    """Tree cover of one component; returns (trees, info)."""
    t_start = time.perf_counter()
    uniq = list(roots)
    warm, warm_tau, _ = rtc_min_max(n, edges, w, uniq)
    info = {"warm_start_tau": warm_tau}
    left = lambda: time_limit - (time.perf_counter() - deadline_t0)  # noqa: E731
    heuristic = params["heuristic"]
    full = RMMTCModel(n, edges, w, uniq)
    if heuristic == "auto":
        if full.nvar <= int(params["full_model_max_vars"]):
            heuristic, model = "none", full
        else:
            best = None
            probe = 0.02 * time_limit
            for kind in ("prh", "srh"):
                for p in GRID:
                    allowed = prh(n, edges, w, uniq, centers, p) if kind == "prh" else srh(n, edges, w, uniq, p)
                    m = RMMTCModel(n, edges, w, uniq, allowed)
                    tr, bound, res = m.solve(min(probe, max(0.1, left())), start=warm)
                    lb = bound if bound is not None and np.isfinite(bound) else math.inf
                    if best is None or lb < best[0]:
                        best = (lb, kind, p, m)
            _, heuristic, p, model = best
            info["parameter"] = p
    elif heuristic == "none":
        model = full
    elif heuristic == "prh":
        model = RMMTCModel(n, edges, w, uniq, prh(n, edges, w, uniq, centers, float(params["alpha"])))
    else:
        model = RMMTCModel(n, edges, w, uniq, srh(n, edges, w, uniq, float(params["beta"])))
    info.update(model=heuristic, variables=int(model.nvar))
    trees, bound, res = model.solve(max(0.1, 0.9 * left()), start=warm)
    info.update(status=res["status"], solver_warm_start=res.get("warm_start"), lower_bound=bound)
    if trees is None:
        info["used_warm_start"] = True
        return warm, info
    tau = max(float(np.sum(w[t[1]])) if t[1] else 0.0 for t in trees)
    if tau > warm_tau + 1e-9:
        info["used_warm_start"] = True
        return warm, info
    info.update(tau=tau, gap=(tau - bound) / tau if bound is not None and tau > 0 else None,
                used_warm_start=False, time=time.perf_counter() - t_start)
    return trees, info


class MIPTreeCover(Stage):
    kind = "work_distribution"
    name = "mip_tree_cover"
    consumes = ("model",)
    produces = "cover"
    hyperparameters = []

    def run(self, data, rep, ctx, params):
        cg = data["model"]["graph"]
        infos = []
        limit = ctx.time_limit if math.isfinite(ctx.time_limit) else 600.0

        def solver(n, E, w, roots, verts):
            trees, info = mip_cover(n, E, w, roots, cg.centers[verts], params, limit, ctx.start_time)
            infos.append(info)
            return trees

        out = tree_cover_by_component(cg, solver)
        out["mip"] = infos
        return out

    def metrics(self, out, rep):
        m = {"max_tree_weight": out["max_weight"]}
        if out.get("mip"):
            m["used_warm_start"] = float(any(i.get("used_warm_start") for i in out["mip"]))
            gaps = [i.get("gap") for i in out["mip"] if i.get("gap") is not None]
            if gaps:
                m["gap"] = float(max(gaps))
        return m


class MIP(_TreeCoverPlanner):
    manifest = Manifest(
        name="mip",
        version="1.0",
        reference="Tang & Ma (2023), Mixed integer programming for time-optimal multi-robot coverage path "
                  "planning with efficient heuristics",
        family="grid",
        adapter="grid",
        adapter_params={"coarse": True, "connectivity": 4},
        constraints={"multi_robot", "return_to_depot"},
        mode="staged",
        stages=["terrain_graph", "mip_tree_cover", "stc_per_tree"],
        hyperparameters=[
            HyperParam("heuristic", "categorical", "auto", choices=["auto", "none", "prh", "srh"],
                       doc="model reduction: the paper's automatic selection, none, PRH or SRH"),
            HyperParam("alpha", "float", 0.6, low=0.1, high=1.0, doc="PRH parabola width (heuristic = prh)"),
            HyperParam("beta", "float", 0.6, low=0.1, high=1.0, doc="SRH removal size (heuristic = srh)"),
            HyperParam("full_model_max_vars", "int", 1200, low=500, high=20000, log=True,
                       doc="largest model solved without reduction when heuristic = auto"),
        ],
        external_dependencies=["MILP solver (HiGHS via SciPy)"],
        core_modules=["uninbench/planners/mip.py", "uninbench/planners/mfc.py", "uninbench/planners/tree_cover.py",
                      "uninbench/planners/grid_common.py"],
        description="Rooted min-max tree cover by mixed-integer programming, STC around every tree.",
    )
    stages = [TerrainGraph(), MIPTreeCover(), STCPerTree()]

"""MCFS -- Multi-Robot Connected Fermat Spiral coverage (Tang & Ma, 2024b); swath/isoline family.

1. **Layered isolines** (swath adapter, ``mode="offset"``): closed offset curves of
   the reachable configuration space spaced by the working width ``l``; every
   isoline is oriented counter-clockwise and carries anchor points every ``l``
   along it (the paper's equidistant isoline points).
2. **Isograph**: one isovertex per isoline; isovertices of adjacent layers are
   joined when they have stitching tuples ``O_{u,v}`` -- pairs of mutually
   nearest anchors ``p in O_{u->v}``, ``q in O_{v->u}`` (Eqns. 3-4) whose
   connections ``p-q`` and ``B(p)-B(q)`` are collision-free.
3. **Isograph augmentation** (``delta`` >= 2): isovertices at graph distance
   ``2 <= k <= delta`` are joined when a chain of consecutive stitching tuples
   connects them, with edge weight ``l * k``.
4. **MMRTC** on the isograph (vertex weight = number of isoline points times
   ``l``, plus the weights of augmented edges) solved with the MIP of Tang &
   Ma (2023), warm-started with the rooted tree cover of MFC.
5. **Unified CFS** (Algorithm 1) on every tree: starting from the robot's entry
   point on its root isoline, the isolines of a depth-first traversal of the
   tree are stitched into one closed path (stitch ``p`` to ``q`` and ``B(p)`` to
   ``B(q)``) with the CFS stitching-tuple selector.

The paper's variants are MCFS (NONE), +AUG, +REF and +BOTH; this planner
implements NONE (``delta`` < 2) and +AUG.  The MMRTC solution refinement
(pairwise isovertex splitting, Sec. 4.3) is not implemented, so robots whose
trees share an isovertex both traverse its isoline.  Isolines start half a
working width from the obstacles (the tool edge reaches the boundary).
"""

from __future__ import annotations

import math
from collections import defaultdict, deque

import numpy as np
import shapely
from scipy.spatial import cKDTree
from shapely.geometry import LineString
from shapely.ops import substring

from ..trajectory import SERVICE, TRANSIT, TrajectoryBuilder, TrajectorySet
from .base import HyperParam, Manifest, PlanResult, Stage, StagedPlanner
from .mfc import rtc_min_max
from .mip import RMMTCModel


class Isoline:
    def __init__(self, line: LineString, layer: int, l: float):
        c = np.asarray(line.coords)
        if not np.allclose(c[0], c[-1]):
            c = np.r_[c, c[:1]]
        ring = shapely.LinearRing(c)
        if not ring.is_ccw:
            c = c[::-1]
        self.line = LineString(c)
        self.layer = layer
        L = self.line.length
        n = max(3, int(round(L / l)))
        self.s = np.linspace(0.0, L, n, endpoint=False)
        self.anchors = np.array([self.line.interpolate(x).coords[0] for x in self.s])
        self.n = n

    def prev(self, i: int) -> int:
        return (i - 1) % self.n

    def arc(self, i: int, j: int) -> np.ndarray:
        """Polyline from anchor i forward (counter-clockwise) to the next anchor j = i + 1."""
        a = self.s[i]
        b = self.s[j] if j != 0 else self.line.length
        return np.asarray(substring(self.line, a, b).coords)


def build_isograph(isolines: list[Isoline], cs_check, l: float):
    """Adjacency with stitching tuples: tuples[(u, v)] = [(p on u, q on v)] for u < v."""
    by_layer = defaultdict(list)
    for k, iso in enumerate(isolines):
        by_layer[iso.layer].append(k)
    trees = {}
    for L, ids in by_layer.items():
        pts = np.concatenate([isolines[k].anchors for k in ids])
        own = np.concatenate([np.full(isolines[k].n, k) for k in ids])
        idx = np.concatenate([np.arange(isolines[k].n) for k in ids])
        trees[L] = (cKDTree(pts), own, idx)
    per_iso = {k: cKDTree(iso.anchors) for k, iso in enumerate(isolines)}
    tuples = {}
    for L in sorted(by_layer):
        if L + 1 not in by_layer:
            continue
        for u in by_layer[L]:
            A = isolines[u].anchors
            t_up, own_up, idx_up = trees[L + 1]
            _, nn = t_up.query(A)
            for p, j in enumerate(nn):
                v = int(own_up[j])
                q = int(idx_up[j])
                # mutual: the anchor of u nearest to q is p, and q's nearest isoline in layer L is u
                _, p2 = per_iso[u].query(isolines[v].anchors[q])
                t_dn, own_dn, _ = trees[L]
                _, jj = t_dn.query(isolines[v].anchors[q])
                if int(p2) != p or int(own_dn[jj]) != u:
                    continue
                P, Q = A[p], isolines[v].anchors[q]
                Pb, Qb = A[isolines[u].prev(p)], isolines[v].anchors[isolines[v].prev(q)]
                if np.hypot(*(P - Q)) > 1.6 * l or np.hypot(*(Pb - Qb)) > 2.6 * l:
                    continue
                if not (cs_check(P, Q) and cs_check(Pb, Qb)):
                    continue
                tuples.setdefault((u, v), []).append((p, q))
    return tuples


def augment(isolines, tuples, cs_check, delta: int, l: float):
    """Augmented edges up to graph distance ``delta``: {(u, v): (tuples, k)}."""
    adj = defaultdict(set)
    for (u, v) in tuples:
        adj[u].add(v)
        adj[v].add(u)
    stitch = {}
    for (u, v), ts in tuples.items():
        stitch[(u, v)] = {p: q for p, q in ts}
        stitch[(v, u)] = {q: p for p, q in ts}
    out = {}
    for s in range(len(isolines)):
        prev = {s: None}
        dist = {s: 0}
        dq = deque([s])
        while dq:
            x = dq.popleft()
            if dist[x] >= delta:
                continue
            for y in sorted(adj[x]):
                if y not in dist:
                    dist[y] = dist[x] + 1
                    prev[y] = x
                    dq.append(y)
        for t, k in dist.items():
            if k < 2 or t <= s or (s, t) in tuples:
                continue
            path = [t]
            while path[-1] != s:
                path.append(prev[path[-1]])
            path = path[::-1]
            chains = []
            for p in range(isolines[s].n):
                cur, ok = p, True
                for a, b in zip(path[:-1], path[1:]):
                    nxt = stitch.get((a, b), {}).get(cur)
                    if nxt is None:
                        ok = False
                        break
                    cur = nxt
                if not ok:
                    continue
                P, Q = isolines[s].anchors[p], isolines[t].anchors[cur]
                Pb = isolines[s].anchors[isolines[s].prev(p)]
                Qb = isolines[t].anchors[isolines[t].prev(cur)]
                if cs_check(P, Q) and cs_check(Pb, Qb):
                    chains.append((p, cur))
            if chains:
                out[(s, t)] = (chains, k)
    return out


def cfs_path(isolines, tree_adj, root: int, entry: int, tuples_of, rng, selector: str = "cfs"):
    """Unified CFS (Algorithm 1): closed path as a cyclic list of (isoline, anchor) nodes and
    the set of stitch edges (frozensets) among its consecutive nodes."""
    nbr = defaultdict(list)

    def add_ring(v):
        n = isolines[v].n
        for i in range(n):
            a, b = (v, i), (v, (i + 1) % n)
            nbr[a].append(b)
            nbr[b].append(a)

    def unlink(a, b):
        nbr[a].remove(b)
        nbr[b].remove(a)

    def link(a, b):
        nbr[a].append(b)
        nbr[b].append(a)

    add_ring(root)
    used = set()
    jumps = set()
    last_q = {}
    seen = {root}
    stack = [root]
    order = []
    while stack:  # DFS traversal edges from the root
        u = stack.pop()
        for v in sorted(tree_adj[u], reverse=True):
            if v not in seen:
                seen.add(v)
                order.append((u, v))
                stack.append(v)
    for u, v in order:
        cands = [(p, q) for p, q in tuples_of(u, v)
                 if (u, p) not in used and (u, isolines[u].prev(p)) not in used and (v, q) not in used
                 and (v, isolines[v].prev(q)) not in used and (u, isolines[u].prev(p)) in nbr[(u, p)]]
        choice = None
        if cands:
            if selector == "random":
                choice = cands[int(rng.integers(len(cands)))]
            else:  # CFS selector: adjacent to the stitch that brought u into the path
                qprev = last_q.get(u)
                choice = next((c for c in cands if qprev is not None and isolines[u].prev(c[0]) == qprev), cands[0])
        add_ring(v)
        if choice is not None:
            p, q = choice
            a, b = (u, isolines[u].prev(p)), (u, p)
            c, d = (v, isolines[v].prev(q)), (v, q)
        else:  # no stitching tuple left: splice the isoline into the nearest path edge at u (transit jumps)
            B = isolines[v].anchors
            best = None
            for i in range(isolines[u].n):
                x = (u, i)
                P = isolines[u].anchors[i]
                dq_ = np.hypot(*(B - P).T)
                qq = int(np.argmin(dq_))
                for y in nbr[x]:
                    if best is None or dq_[qq] < best[0]:
                        best = (float(dq_[qq]), x, y, qq)
            _, a, b, q = best
            c, d = (v, isolines[v].prev(q)), (v, q)
            jumps |= {frozenset((a, c)), frozenset((d, b))}
        unlink(a, b)
        unlink(c, d)
        link(a, c)
        link(d, b)
        used |= {a, b, c, d}
        last_q[v] = q
    # walk the cycle from the entry anchor
    start = (root, entry)
    cyc = [start]
    prev, cur = None, start
    first = min(nbr[start], key=lambda x: (x[0] != root, x))
    prev, cur = start, first
    guard = 0
    while cur != start and guard < 10 ** 7:
        cyc.append(cur)
        a, b = nbr[cur]
        prev, cur = cur, (b if a == prev else a)
        guard += 1
    cyc.append(start)
    return cyc, jumps


class IsographModel(Stage):
    kind = "environment_model"
    name = "isograph"
    consumes = ("representation",)
    produces = "model"

    def run(self, data, rep, ctx, params):
        robot = rep.team[0]
        l = min(r.working_width for r in rep.team)
        cs = rep.cspace(robot)
        cs_tol = cs.buffer(1e-6 * max(1.0, l))
        shapely.prepare(cs_tol)

        def free(a, b):
            return bool(cs_tol.covers(LineString([tuple(a), tuple(b)])))

        isolines = [Isoline(s["line"], int(s.get("row", 0)), l) for s in rep.swaths
                    if s["line"].length > 1e-6]
        tuples = build_isograph(isolines, free, l)
        aug = augment(isolines, tuples, free, int(params["delta"]), l) if int(params["delta"]) >= 2 else {}
        return {"isolines": isolines, "tuples": tuples, "augmented": aug, "l": l, "free": free}

    def summary(self, out):
        return {"isovertices": len(out["isolines"]), "edges": len(out["tuples"]),
                "augmented_edges": len(out["augmented"])}


class IsographTreeCover(Stage):
    kind = "work_distribution"
    name = "isograph_mmrtc"
    consumes = ("model",)
    produces = "cover"

    def run(self, data, rep, ctx, params):
        md = data["model"]
        iso, l = md["isolines"], md["l"]
        n = len(iso)
        pairs = sorted(set(md["tuples"]) | set(md["augmented"]))
        edges = np.array(pairs, dtype=int).reshape(-1, 2)
        w_e = np.array([l * md["augmented"][e][1] if e in md["augmented"] else 0.0 for e in pairs])
        w_v = np.array([it.n * l for it in iso])
        allpts = np.concatenate([it.anchors for it in iso])
        own = np.concatenate([np.full(it.n, k) for k, it in enumerate(iso)])
        idx = np.concatenate([np.arange(it.n) for it in iso])
        tree = cKDTree(allpts)
        roots, entries = [], []
        for r in rep.team:
            _, j = tree.query(r.start[:2])
            roots.append(int(own[j]))
            entries.append(int(idx[j]))
        # components of the isograph with robots
        comp = np.arange(n)

        def find(a):
            while comp[a] != a:
                comp[a] = comp[comp[a]]
                a = comp[a]
            return a

        for u, v in edges:
            comp[find(int(u))] = find(int(v))
        labels = np.array([find(v) for v in range(n)])
        trees = [None] * len(rep.team)
        info = []
        for c in sorted({labels[r] for r in roots}):
            verts = np.flatnonzero(labels == c)
            loc = {int(v): t for t, v in enumerate(verts)}
            emask = np.array([labels[u] == c for u, _ in edges], dtype=bool) if len(edges) else np.zeros(0, bool)
            E = np.array([[loc[int(u)], loc[int(v)]] for u, v in edges[emask]], dtype=int).reshape(-1, 2)
            we = w_e[emask] if len(edges) else np.zeros(0)
            wv = w_v[verts]
            robots = [k for k, r in enumerate(roots) if labels[r] == c]
            R = [loc[roots[k]] for k in robots]
            proxy = we + 0.5 * (wv[E[:, 0]] + wv[E[:, 1]]) if len(E) else we
            warm, _, _ = rtc_min_max(len(verts), E, proxy, R)
            if len(E):
                model = RMMTCModel(len(verts), E, we, R, wv=wv)
                budget = max(0.5, 0.6 * ctx.time_left()) if math.isfinite(ctx.time_limit) else 60.0
                sol, bound, res = model.solve(budget, start=warm)
                chosen = sol if sol is not None else warm
                info.append({"status": res.get("status"), "lower_bound": bound, "warm_start_used": sol is None})
            else:
                chosen = warm
            for k, (tv, te) in zip(robots, chosen):
                trees[k] = ({int(verts[v]) for v in tv} | {roots[k]},
                            [(int(verts[E[e, 0]]), int(verts[E[e, 1]])) for e in te])
        cost = [sum(w_v[v] for v in t[0]) + sum(l * md["augmented"][tuple(sorted(e))][1]
                                                 for e in t[1] if tuple(sorted(e)) in md["augmented"])
                for t in trees]
        return {"trees": trees, "roots": roots, "entries": entries, "tree_costs": cost, "mip": info}

    def metrics(self, out, rep):
        c = np.array(out["tree_costs"], dtype=float)
        return {"max_tree_cost": float(c.max()) if len(c) else 0.0,
                "imbalance": float(c.max() / c.mean()) if len(c) and c.mean() > 0 else None}


class UnifiedCFS(Stage):
    kind = "route_construction"
    name = "unified_cfs"
    consumes = ("cover", "model")
    produces = "routes"

    def run(self, data, rep, ctx, params):
        md, cv = data["model"], data["cover"]
        iso = md["isolines"]
        tuples, aug = md["tuples"], md["augmented"]

        def tuples_of(u, v):
            if (u, v) in tuples:
                return list(tuples[(u, v)])
            if (v, u) in tuples:
                return [(p, q) for q, p in tuples[(v, u)]]
            key = (min(u, v), max(u, v))
            if key in aug:
                ts = aug[key][0]
                return list(ts) if key == (u, v) else [(p, q) for q, p in ts]
            return []

        paths = {}
        for k, r in enumerate(rep.team):
            verts, tedges = cv["trees"][k]
            adj = defaultdict(set)
            for u, v in tedges:
                adj[u].add(v)
                adj[v].add(u)
            cyc, jumps = cfs_path(iso, adj, cv["roots"][k], cv["entries"][k], tuples_of, ctx.rng,
                                  params["selector"])
            paths[r.id] = (cyc, jumps)
        return {"paths": paths}


class MCFS(StagedPlanner):
    manifest = Manifest(
        name="mcfs",
        version="1.0",
        reference="Tang & Ma (2024b), Multi-robot connected Fermat spiral coverage -- MCFS and MCFS+AUG (the MMRTC "
                  "solution refinement is not implemented)",
        family="swath",
        adapter="swath",
        adapter_params={"mode": "offset"},
        constraints={"multi_robot", "return_to_depot"},
        mode="staged",
        stages=["isograph", "isograph_mmrtc", "unified_cfs"],
        hyperparameters=[
            HyperParam("delta", "int", 4, low=0, high=8, doc="isograph augmentation level (< 2: MCFS without AUG)"),
            HyperParam("selector", "categorical", "cfs", choices=["cfs", "random"], doc="stitching tuple selector"),
        ],
        external_dependencies=["MILP solver (HiGHS via SciPy)"],
        core_modules=["uninbench/planners/mcfs.py", "uninbench/planners/mip.py", "uninbench/planners/mfc.py"],
        description="Layered isolines, isograph tree cover by MIP, connected Fermat spirals per tree.",
    )
    stages = [IsographModel(), IsographTreeCover(), UnifiedCFS()]

    def finish(self, data, rep, ctx) -> PlanResult:
        md = data["model"]
        iso = md["isolines"]
        free = md["free"]
        ts = TrajectorySet(metadata={"planner": self.manifest.name})
        for r in rep.team:
            cyc, jumps = data["routes"]["paths"][r.id]
            b = TrajectoryBuilder(r.id, *r.start, tool_active_in_turns=self.manifest.tool_active_in_turns)
            first = iso[cyc[0][0]].anchors[cyc[0][1]]
            tp = rep.transit(r)
            b.follow(tp.path(b.pos, first), TRANSIT)
            for a, c in zip(cyc[:-1], cyc[1:]):
                if a[0] == c[0] and (c[1] == (a[1] + 1) % iso[a[0]].n or a[1] == (c[1] + 1) % iso[a[0]].n) \
                        and frozenset((a, c)) not in jumps:
                    if c[1] == (a[1] + 1) % iso[a[0]].n:
                        pts = iso[a[0]].arc(a[1], c[1])
                    else:
                        pts = iso[a[0]].arc(c[1], a[1])[::-1]
                    b.follow(pts, SERVICE)
                else:
                    P, Q = iso[a[0]].anchors[a[1]], iso[c[0]].anchors[c[1]]
                    if frozenset((a, c)) not in jumps and free(P, Q):
                        b.move_to(Q[0], Q[1], SERVICE)
                    else:
                        b.follow(tp.path(b.pos, Q), TRANSIT)
            if ctx.return_to_depot:
                depot = r.depot if r.depot is not None else r.start[:2]
                b.follow(tp.path(b.pos, depot), TRANSIT)
            ts.add(b.build())
        layers = rep.layers()
        cv = data["cover"]
        return PlanResult(ts, native={r.id: [list(n) for n in data["routes"]["paths"][r.id][0]] for r in rep.team},
                          layers=layers, info={"tree_costs": cv["tree_costs"], "mip": cv["mip"]})

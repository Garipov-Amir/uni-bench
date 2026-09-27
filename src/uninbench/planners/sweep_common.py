"""Shared machinery of sweep planners (swath and cellular-decomposition families):
pass ordering, contiguous work splitting, and route construction with
collision-free transit, Dubins headland turns and depot trips."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
import shapely
from scipy.optimize import linear_sum_assignment
from shapely.geometry import LineString
from shapely.ops import substring

from ..geometry.dubins import dubins_paths
from ..trajectory import SERVICE, TRANSIT, TrajectoryBuilder
from ..util import wrap_angle


@dataclass
class Pass:
    pts: np.ndarray  # (n, 2)
    closed: bool = False
    group: int = 0

    @property
    def length(self) -> float:
        return float(np.hypot(*np.diff(self.pts, axis=0).T).sum())

    def oriented(self, rev: bool) -> np.ndarray:
        return self.pts[::-1] if rev and not self.closed else self.pts

    def start(self, rev: bool) -> np.ndarray:
        return self.oriented(rev)[0]

    def end(self, rev: bool) -> np.ndarray:
        return self.oriented(rev)[-1]

    def heading_in(self, rev: bool) -> float:
        p = self.oriented(rev)
        d = p[1] - p[0]
        return math.atan2(d[1], d[0])

    def heading_out(self, rev: bool) -> float:
        p = self.oriented(rev)
        d = p[-1] - p[-2]
        return math.atan2(d[1], d[0])


def connection_cost(a: Pass, ra: bool, b: Pass, rb: bool, v: float, tc: float) -> float:
    p, q = a.end(ra), b.start(rb)
    d = float(np.hypot(*(q - p)))
    if d > 1e-9:
        h = math.atan2(q[1] - p[1], q[0] - p[0])
        turn = abs(float(wrap_angle(h - a.heading_out(ra)))) + abs(float(wrap_angle(b.heading_in(rb) - h)))
    else:
        turn = abs(float(wrap_angle(b.heading_in(rb) - a.heading_out(ra))))
    return d / v + turn * tc


def skip_order(passes: list[Pass], start_xy, k: int) -> list[tuple[int, bool]]:
    """Agricultural skip pattern: consecutive passes are k (then k-1) rows apart so that
    headland turns keep at least the minimum turning radius (blocks 0,k,1,k+1,...)."""
    order = sorted(range(len(passes)), key=lambda i: (passes[i].group, float(np.min(passes[i].pts[:, 0]))))
    rings = [i for i in order if passes[i].closed]
    order = [i for i in order if not passes[i].closed]
    if not order:
        return [(i, False) for i in rings]
    if k <= 1:
        seq = order
    else:
        # near-equal blocks of at least 2k passes (no short remainder block), each visited
        # as j, j+h, j+1, j+h+1, ... with h >= k, so neighbouring visits are >= k-1 rows apart
        n = len(order)
        nb = max(1, n // (2 * k))
        bounds = np.linspace(0, n, nb + 1).round().astype(int)
        seq = []
        for b0, b1 in zip(bounds[:-1], bounds[1:]):
            block = order[b0:b1]
            h = (len(block) + 1) // 2
            for j in range(h):
                seq.append(block[j])
                if j + h < len(block):
                    seq.append(block[j + h])
    # start from the end nearer to the robot and alternate directions
    first = passes[seq[0]]
    rev = bool(np.hypot(*(first.end(False) - start_xy)) < np.hypot(*(first.start(False) - start_xy)))
    out = []
    for i in seq:
        out.append((i, rev))
        rev = not rev
    return out + [(i, False) for i in rings]  # headland contours last, as in field practice


def nn_order(passes: list[Pass], start_xy, v: float, tc: float) -> list[tuple[int, bool]]:
    """Greedy nearest-neighbour order with direction choice."""
    n = len(passes)
    if n == 0:
        return []
    S = np.array([p.start(False) for p in passes])
    E = np.array([p.end(False) for p in passes])
    left = np.ones(n, dtype=bool)
    pos = np.asarray(start_xy, dtype=float)
    seq = []
    prev = None
    for _ in range(n):
        ds = np.hypot(*(S - pos).T)
        de = np.hypot(*(E - pos).T)
        cand = np.where(left, np.minimum(ds, de), np.inf)
        best = float(cand.min())
        ties = np.flatnonzero(cand <= best + 1e-9)
        if prev is not None and len(ties) > 1:
            k = min(ties, key=lambda i: connection_cost(passes[prev[0]], prev[1], passes[i], de[i] < ds[i], v, tc))
        else:
            k = int(ties[0])
        rev = bool(de[k] < ds[k])
        seq.append((int(k), rev))
        left[k] = False
        pos = passes[k].end(rev)
        prev = (k, rev)
    return seq


def sequence_time(seq, passes, v, tc, start_xy=None) -> float:
    t = 0.0
    for k, (i, r) in enumerate(seq):
        t += passes[i].length / v
        if k > 0:
            j, rj = seq[k - 1]
            t += connection_cost(passes[j], rj, passes[i], r, v, tc)
        elif start_xy is not None:
            t += float(np.hypot(*(passes[i].start(r) - np.asarray(start_xy)))) / v
    return t


def two_opt(seq, passes, v, tc, start_xy, max_rounds: int = 20, deadline=None):
    """2-opt on a sequence of oriented passes; reversing a block also flips the
    orientation of its passes, so only the two boundary connections change."""
    seq = list(seq)
    n = len(seq)
    if n < 3:
        return seq
    start_xy = np.asarray(start_xy, dtype=float)

    def cost(prev, cur):
        if cur is None:
            return 0.0
        if prev is None:
            return float(np.hypot(*(passes[cur[0]].start(cur[1]) - start_xy))) / v
        return connection_cost(passes[prev[0]], prev[1], passes[cur[0]], cur[1], v, tc)

    for _ in range(max_rounds):
        improved = False
        for i in range(0, n - 1):
            prev = seq[i - 1] if i > 0 else None
            for j in range(i + 1, n):
                nxt = seq[j + 1] if j + 1 < n else None
                a, b = seq[i], seq[j]
                a2, b2 = (b[0], not b[1]), (a[0], not a[1])
                old = cost(prev, a) + (cost(b, nxt) if nxt is not None else 0.0)
                new = cost(prev, a2) + (cost(b2, nxt) if nxt is not None else 0.0)
                if new < old - 1e-9:
                    seq[i:j + 1] = [(k, not r) for k, r in reversed(seq[i:j + 1])]
                    improved = True
            if deadline is not None and deadline():
                return seq
        if not improved:
            break
    return seq


def split_contiguous(seq, passes, robots, iters: int = 200):
    """Split an ordered pass sequence into contiguous chunks, one per robot, and assign
    chunks (in either direction) to robots minimising the largest completion time."""
    m = len(robots)
    n = len(seq)
    v = np.array([r.speed for r in robots])
    tc = float(np.mean([r.turn_cost for r in robots]))
    vm = float(v.mean())
    serv = np.array([passes[i].length / vm for i, _ in seq])
    conn = np.zeros(n)
    for k in range(1, n):
        (a, ra), (b, rb) = seq[k - 1], seq[k]
        conn[k] = connection_cost(passes[a], ra, passes[b], rb, vm, tc)
    starts = np.array([r.start[:2] for r in robots])

    def chunk_cost(lo, hi):
        """cost matrix (robots,) for chunk [lo, hi) and best direction per robot."""
        if hi <= lo:
            return np.zeros(m), np.zeros(m, dtype=bool)
        body = serv[lo:hi].sum() + conn[lo + 1:hi].sum()
        fs = passes[seq[lo][0]].start(seq[lo][1])
        le = passes[seq[hi - 1][0]].end(seq[hi - 1][1])
        tf = np.hypot(*(starts - fs).T)
        tb = np.hypot(*(starts - le).T)
        rev = tb < tf
        return (np.minimum(tf, tb) + body * vm) / v, rev

    def evaluate(bounds):
        C = np.zeros((m, m))
        R = np.zeros((m, m), dtype=bool)
        for c in range(m):
            C[:, c], R[:, c] = chunk_cost(bounds[c], bounds[c + 1])
        scale = max(C.max(), 1e-9)
        ri, ci = linear_sum_assignment((C / scale) ** 4)
        return C[ri, ci].max(), dict(zip(ri, ci)), R

    w = serv + conn
    total = w.sum()
    cum = np.cumsum(w)
    bounds = [0] + [int(np.searchsorted(cum, total * k / m)) for k in range(1, m)] + [n]
    bounds = list(np.maximum.accumulate(np.minimum(bounds, n)))
    best_val, best_assign, best_R = evaluate(bounds)
    for _ in range(iters):
        improved = False
        for c in range(1, m):
            for delta in (-1, 1):
                nb = list(bounds)
                nb[c] += delta
                if nb[c] < nb[c - 1] or nb[c] > nb[c + 1]:
                    continue
                val, assign, R = evaluate(nb)
                if val < best_val - 1e-9:
                    bounds, best_val, best_assign, best_R, improved = nb, val, assign, R, True
        if not improved:
            break
    out = {}
    for k, r in enumerate(robots):
        c = best_assign[k]
        chunk = seq[bounds[c]:bounds[c + 1]]
        if best_R[k, c]:
            chunk = [(i, not rv) for i, rv in reversed(chunk)]
        out[r.id] = chunk
    return out, {"bounds": bounds, "estimated_makespan": float(best_val)}


# ------------------------------------------------------------------ routes
class RouteBuilder:
    """Turns an ordered list of passes into a trajectory for one robot."""

    def __init__(self, rep, robot, manifest, ctx, dubins_step: float | None = None, service_links: bool = False):
        self.rep = rep
        # short straight links between neighbouring passes keep the tool active
        self.link_max = 1.5 * robot.working_width if service_links and not robot.min_turn_radius else 0.0
        self.robot = robot
        self.ctx = ctx
        # plan with a 10 % margin on the turning radius so that piece/arc junctions stay feasible
        self.r_min = 1.1 * robot.min_turn_radius if robot.min_turn_radius else None
        self.b = TrajectoryBuilder(robot.id, *robot.start, tool_active_in_turns=manifest.tool_active_in_turns)
        cs = rep.cspace(robot)
        self.cs_tol = cs.buffer(1e-6 * max(1.0, robot.working_width))
        shapely.prepare(self.cs_tol)
        self.transit = rep.transit(robot)
        self.step = dubins_step or (self.r_min / 20 if self.r_min else None)
        self.dubins_fallbacks = 0
        self.skipped_pieces = 0

    def connect(self, target, heading: float | None = None, required: bool = True) -> bool:
        """Move to ``target``; with a turning radius and ``required=False`` return False
        (and do nothing) when no feasible Dubins connection exists."""
        b = self.b
        target = np.asarray(target, dtype=float)
        d = float(np.hypot(*(target - np.asarray(b.pos))))
        if d <= 1e-9 and (not self.r_min or heading is None
                          or abs(float(wrap_angle(heading - b.heading))) < 1e-6):
            return True
        if d <= self.link_max and self.transit.segment_free(b.pos, target):
            b.move_to(target[0], target[1], SERVICE)
            return True
        if self.r_min:
            curve = self.plan_curve((b.pos[0], b.pos[1], b.heading), target, heading)
            if curve is not None:
                b.follow_curve(curve, TRANSIT)
                return True
            if not required:
                return False
            self.dubins_fallbacks += 1
        b.follow(self.transit.path(b.pos, target), TRANSIT)
        return True

    def plan_curve(self, pose, target, heading=None, depth: int = 0):
        """Collision-free forward path respecting the turning radius (or None).

        Tries a direct Dubins path, then a Dubins-smoothed visibility path with
        turning-radius clearance, and finally a detour through a hub pose in open
        space (e.g. to leave a headland contour before heading for a nearby depot)."""
        h = heading if heading is not None else math.atan2(target[1] - pose[1], target[0] - pose[0])
        curve = self._dubins(pose, (target[0], target[1], h))
        if curve is not None:
            return curve
        curve = self._smoothed(pose, target, h)
        if curve is not None or depth > 0:
            return curve
        for c in self._hubs(pose):
            for hc in np.linspace(-math.pi, math.pi, 8, endpoint=False):
                c1 = self._dubins(pose, (c[0], c[1], hc))
                if c1 is None:
                    continue
                c2 = self.plan_curve((c[0], c[1], hc), target, heading, depth=1)
                if c2 is not None:
                    return np.vstack([c1, c2[1:]])
        return None

    def _hubs(self, pose, k: int = 6) -> np.ndarray:
        cache = self.rep.__dict__.setdefault("_cache", {})
        key = ("hubs", self.robot.body_fp, self.r_min)
        if key not in cache:
            from ..geometry.decomposition import polygons_of

            region = self.rep.cspace(self.robot).buffer(-2 * self.r_min)
            pts = []
            for poly in polygons_of(region):
                ring = poly.exterior
                n = max(4, int(ring.length / max(self.r_min, 1e-9)))
                pts += [ring.interpolate(t, normalized=True).coords[0] for t in np.linspace(0, 1, n, endpoint=False)]
                q = poly.representative_point()
                pts.append((q.x, q.y))
            cache[key] = np.asarray(pts, dtype=float).reshape(-1, 2)
        hubs = cache[key]
        if len(hubs) == 0:
            return hubs
        d = np.hypot(hubs[:, 0] - pose[0], hubs[:, 1] - pose[1])
        return hubs[np.argsort(d)[:k]]

    def _smoothed(self, pose, target, h):
        """Dubins-smoothed visibility path with turning-radius clearance."""
        path = self.clear_transit().path(pose[:2], target)
        pieces = []
        for k in range(1, len(path)):
            q = path[k]
            hk = h if k == len(path) - 1 else math.atan2(path[k + 1][1] - q[1], path[k + 1][0] - q[0])
            c = self._dubins(pose, (q[0], q[1], hk))
            if c is None:
                return None
            pieces.append(c)
            pose = (q[0], q[1], hk)
        return np.vstack([pieces[0]] + [c[1:] for c in pieces[1:]]) if pieces else None

    def can_escape(self, piece: np.ndarray) -> bool:
        """After driving ``piece`` the robot must still be able to reach its depot."""
        d = piece[-1] - piece[-2]
        pose = (piece[-1][0], piece[-1][1], math.atan2(d[1], d[0]))
        depot = self.robot.depot if self.robot.depot is not None else self.robot.start[:2]
        return self.plan_curve(pose, np.asarray(depot, dtype=float)) is not None

    def clear_transit(self):
        """Visibility graph on the configuration space shrunk by the turning radius, so that
        Dubins arcs around its vertices stay clear of obstacles."""
        cache = self.rep.__dict__.setdefault("_cache", {})
        key = ("clear_transit", self.robot.body_fp, self.r_min)
        if key not in cache:
            from ..geometry.transit import TransitPlanner

            inner = self.rep.cspace(self.robot).buffer(-self.r_min)
            cache[key] = TransitPlanner(inner if not inner.is_empty else self.rep.cspace(self.robot),
                                        simplify=0.05 * self.r_min)
        return cache[key]

    def _dubins(self, start, goal):
        if np.hypot(goal[0] - start[0], goal[1] - start[1]) < 1e-9 and abs(goal[2] - start[2]) < 1e-9:
            return np.array([start])
        for _, _, pts in dubins_paths(start, goal, self.r_min, self.step):
            if self.cs_tol.covers(LineString(pts[:, :2])):
                return pts
        return None

    def service(self, pts: np.ndarray):
        pieces = curvature_pieces(pts, self.r_min) if self.r_min and len(pts) > 2 else [pts]
        for piece in pieces:
            if len(piece) < 2:
                continue
            h = math.atan2(piece[1][1] - piece[0][1], piece[1][0] - piece[0][0])
            if self.r_min:
                # a turning-radius robot skips pieces it cannot drive or reach feasibly
                if (not self.cs_tol.covers(LineString(piece)) or not self.can_escape(piece)
                        or not self.connect(piece[0], h, required=False)):
                    self.skipped_pieces += 1
                    continue
            else:
                self.connect(piece[0], h)
            self.b.follow(piece, SERVICE)

    def run(self, jobs, passes, return_to_depot: bool = False):
        cap = self.robot.capacity
        w = self.robot.working_width
        depot = self.robot.depot if self.robot.depot is not None else self.robot.start[:2]
        left = cap if cap else math.inf
        link, first = self.link_max, True
        for i, rev in jobs:
            pts = passes[i].oriented(rev)
            if passes[i].closed:  # enter a closed contour at its vertex nearest to the robot
                ring = pts[:-1] if np.allclose(pts[0], pts[-1]) else pts
                k = int(np.argmin(np.hypot(*(ring - np.asarray(self.b.pos)).T)))
                ring = np.r_[ring[k:], ring[:k]]
                ring = ring[::-1] if rev else ring
                pts = np.r_[ring, ring[:1]]
            for piece in _split_by_capacity(pts, cap / w if cap else None):
                area = float(np.hypot(*np.diff(piece, axis=0).T).sum()) * w
                if area > left + 1e-9:
                    self.connect(depot)
                    left = cap
                self.link_max = 0.0 if first else link  # never service the approach from the start
                self.service(piece)
                self.link_max = link
                first = False
                left -= area
        if return_to_depot:
            self.connect(depot)
        return self.b.build()


def _split_by_capacity(pts: np.ndarray, max_len: float | None):
    if not max_len:
        return [pts]
    line = LineString(pts)
    L = line.length
    if L <= max_len:
        return [pts]
    n = int(math.ceil(L / max_len))
    cuts = np.linspace(0, L, n + 1)
    out = []
    for a, b in zip(cuts[:-1], cuts[1:]):
        seg = substring(line, a, b)
        out.append(np.asarray(seg.coords))
    return out


def curvature_pieces(pts: np.ndarray, r_min: float, slack: float = 1.0) -> list[np.ndarray]:
    """Cut a polyline where it bends tighter than a turn of radius ``r_min`` allows.

    Vertices whose local curvature (turn / mean adjacent chord) exceeds
    ``slack / r_min`` form tight runs; each run is removed together with the
    fillet length ``r tan(turn/2)`` on both sides, so that the remaining pieces
    lie exactly on the original polyline and a Dubins manoeuvre can join them."""
    pts = np.asarray(pts, dtype=float)
    d = np.diff(pts, axis=0)
    L = np.hypot(d[:, 0], d[:, 1])
    keep = np.r_[True, L > 1e-9]
    pts = pts[keep]
    if len(pts) < 3:
        return [pts]
    d = np.diff(pts, axis=0)
    L = np.hypot(d[:, 0], d[:, 1])
    h = np.arctan2(d[:, 1], d[:, 0])
    dpsi = np.abs((np.diff(h) + np.pi) % (2 * np.pi) - np.pi)  # turn at vertex i+1
    kappa = dpsi / (0.5 * (L[:-1] + L[1:]))
    tight = kappa > slack / r_min
    if not tight.any():
        return [pts]
    s = np.r_[0.0, np.cumsum(L)]  # arclength of every vertex
    runs = []
    k = 0
    while k < len(tight):
        if tight[k]:
            j = k
            while j + 1 < len(tight) and tight[j + 1]:
                j += 1
            runs.append((k + 1, j + 1, float(dpsi[k:j + 1].sum())))  # vertex indices
            k = j + 1
        else:
            k += 1
    line = LineString(pts)
    cuts = []  # arclength intervals to remove
    for a, b, turn in runs:
        t = r_min * math.tan(min(turn, 3.0) / 2)
        cuts.append((max(0.0, s[a] - t), min(s[-1], s[b] + t)))
    cuts.sort()
    merged = []
    for a, b in cuts:
        if merged and a <= merged[-1][1]:
            merged[-1] = (merged[-1][0], max(merged[-1][1], b))
        else:
            merged.append((a, b))
    keep_iv, cur = [], 0.0
    for a, b in merged:
        if a > cur:
            keep_iv.append((cur, a))
        cur = max(cur, b)
    if cur < s[-1]:
        keep_iv.append((cur, s[-1]))
    out = []
    for a, b in keep_iv:
        if b - a > 1e-6:
            out.append(_strip_slivers(np.asarray(substring(line, a, b).coords), 0.05 * r_min))
    return [p for p in out if len(p) >= 2]


def _strip_slivers(p: np.ndarray, min_len: float) -> np.ndarray:
    """Drop end chords shorter than ``min_len`` created by cutting inside a chord."""
    while len(p) > 2 and np.hypot(*(p[1] - p[0])) < min_len:
        p = p[1:]
    while len(p) > 2 and np.hypot(*(p[-1] - p[-2])) < min_len:
        p = p[:-1]
    return p

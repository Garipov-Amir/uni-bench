"""Coverage counting at the evaluation resolution.

For every evaluation cell x the evaluator counts the number of distinct
sweeps n(x) = sum_j n_j(x), where n_j(x) is the number of maximal time
intervals during which x lies inside robot j's tool footprint while the tool
is active.  One continuous pass is counted once regardless of how many
waypoints it has; a later return is counted again.

Implementation: along each active segment the set of parameters during which
a cell centre is inside the (convex) footprint is a single interval, computed
in closed form.  Intervals are processed in time order and merged with the
previous interval of the same cell when they belong to the same run of
consecutive active segments and touch in time.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from ..geometry.footprint import Footprint
from ..geometry.raster import RasterGrid
from ..trajectory import EPS, Trajectory
from ..util import wrap_angle


@dataclass
class CoverageMap:
    grid: RasterGrid
    counts: np.ndarray  # (ny, nx) int32, n(x) summed over robots
    owner: np.ndarray  # (ny, nx) int16, index of the robot sweeping the cell most often (-1: none)
    robots: list  # robot ids in index order
    events: dict | None = None  # {"cell": int32[], "t": float32[], "robot": int16[]}


def sweep_counts(grid: RasterGrid, traj: Trajectory, tool: Footprint, record_events: bool = False,
                 chunk_factor: float = 4.0):
    """Number of distinct sweeps of one robot for every cell of ``grid``."""
    N = grid.nx * grid.ny
    n = np.zeros(N, dtype=np.int32)
    last_exit = np.full(N, -np.inf)
    last_run = np.full(N, -1, dtype=np.int64)
    ev_c, ev_t = [], []
    xs, ys = grid.xs(), grid.ys()
    R = tool.radius
    chunk = max(chunk_factor * R, 8 * grid.res)
    t = traj.t
    if t is None:
        raise ValueError("trajectory must be re-timed before coverage counting")
    run = 0
    prev_active = False

    def process(i0, i1, j0, j1, ok, tin, tout):
        if not np.any(ok):
            return
        jj, ii = np.meshgrid(np.arange(j0, j1), np.arange(i0, i1))
        idx = (ii * grid.nx + jj)[ok]
        tin = tin[ok]
        tout = tout[ok]
        eps = 1e-9 * (1.0 + abs(float(tin.max())))
        cont = (last_run[idx] == run) & (tin <= last_exit[idx] + eps)
        new = ~cont
        n[idx[new]] += 1
        if record_events and np.any(new):
            ev_c.append(idx[new].astype(np.int32))
            ev_t.append(tin[new].astype(np.float32))
        last_exit[idx] = np.where(cont, np.maximum(last_exit[idx], tout), tout)
        last_run[idx] = run

    for k in range(traj.K):
        if not traj.active[k]:
            if prev_active:
                run += 1
            prev_active = False
            continue
        prev_active = True
        p0, p1 = traj.xy[k], traj.xy[k + 1]
        T0, T1 = float(t[k]), float(t[k + 1])
        L = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
        if L <= EPS:
            i0, i1, j0, j1 = grid.window(p0[0] - R, p0[1] - R, p0[0] + R, p0[1] + R)
            if i1 <= i0 or j1 <= j0:
                continue
            qx, qy = np.meshgrid(xs[j0:j1], ys[i0:i1])
            if tool.kind == "disc":
                ok = tool.contains(qx, qy, p0[0], p0[1])
            else:
                d = float(wrap_angle(traj.psi[k + 1] - traj.psi[k]))
                steps = max(1, int(math.ceil(abs(d) / math.radians(5))))
                ok = np.zeros(qx.shape, dtype=bool)
                for a in np.linspace(0.0, d, steps + 1):
                    ok |= tool.contains(qx, qy, p0[0], p0[1], traj.psi[k] + a)
            process(i0, i1, j0, j1, ok, np.full(qx.shape, T0), np.full(qx.shape, T1))
            continue
        ux, uy = (p1[0] - p0[0]) / L, (p1[1] - p0[1]) / L
        psi = float(traj.psi[k])
        nch = max(1, int(math.ceil(L / chunk)))
        for c in range(nch):
            sa, sb = L * c / nch, L * (c + 1) / nch
            ax, ay = p0[0] + ux * sa, p0[1] + uy * sa
            bx, by = p0[0] + ux * sb, p0[1] + uy * sb
            i0, i1, j0, j1 = grid.window(min(ax, bx) - R, min(ay, by) - R, max(ax, bx) + R, max(ay, by) + R)
            if i1 <= i0 or j1 <= j0:
                continue
            qx, qy = np.meshgrid(xs[j0:j1], ys[i0:i1])
            ok, s0, s1 = tool.segment_intervals(qx, qy, p0, p1, psi)
            s0 = np.maximum(s0, sa)
            s1 = np.minimum(s1, sb)
            ok &= s0 <= s1 + 1e-12
            tin = T0 + (T1 - T0) * s0 / L
            tout = T0 + (T1 - T0) * s1 / L
            process(i0, i1, j0, j1, ok, tin, tout)
    events = None
    if record_events:
        events = {
            "cell": np.concatenate(ev_c) if ev_c else np.zeros(0, np.int32),
            "t": np.concatenate(ev_t) if ev_t else np.zeros(0, np.float32),
        }
    return n.reshape(grid.shape), events


def coverage_map(grid: RasterGrid, retimed, team, record_events: bool = False) -> CoverageMap:
    robots = {r.id: r for r in team}
    total = np.zeros(grid.shape, dtype=np.int32)
    owner = np.full(grid.shape, -1, dtype=np.int16)
    best = np.zeros(grid.shape, dtype=np.int32)
    evs = []
    for k, tr in enumerate(retimed):
        cnt, ev = sweep_counts(grid, tr, robots[tr.robot_id].tool, record_events)
        total += cnt
        better = cnt > best
        owner[better] = k
        best[better] = cnt[better]
        if ev is not None:
            ev["robot"] = np.full(len(ev["cell"]), k, dtype=np.int16)
            evs.append(ev)
    events = None
    if record_events:
        if evs:
            cells = np.concatenate([e["cell"] for e in evs])
            ts = np.concatenate([e["t"] for e in evs])
            rob = np.concatenate([e["robot"] for e in evs])
            order = np.argsort(ts, kind="stable")
            events = {"cell": cells[order], "t": ts[order], "robot": rob[order],
                      "robots": [tr.robot_id for tr in retimed]}
        else:
            events = {"cell": np.zeros(0, np.int32), "t": np.zeros(0, np.float32), "robot": np.zeros(0, np.int16),
                      "robots": []}
    return CoverageMap(grid=grid, counts=total, owner=owner, robots=[tr.robot_id for tr in retimed], events=events)

"""Static collisions with the uninflated canonical obstacles and inter-robot conflicts."""

from __future__ import annotations

import math

import numpy as np
import shapely

from ..trajectory import EPS, TURN, WAIT, Trajectory
from ..util import wrap_angle


def static_collisions(traj: Trajectory, body, free, tol: float, res: float) -> tuple[float, float]:
    """(collision time, collision length) of one re-timed trajectory.

    Disc bodies are checked exactly against the configuration space
    ``F_c (-) B``; rectangular bodies are checked by sampling footprints along
    segments whose swept hull leaves the free space.
    """
    if traj.K == 0:
        return 0.0, 0.0
    L = traj.seg_lengths()
    dt = np.diff(traj.t)
    coll_t = 0.0
    coll_len = 0.0
    if body.kind == "disc":
        cs = body.erode(free).buffer(tol)
        shapely.prepare(cs)
        moving = L > EPS
        idx = np.flatnonzero(moving)
        if len(idx):
            lines = shapely.linestrings(np.stack([traj.xy[idx], traj.xy[idx + 1]], axis=1))
            ok = shapely.covers(cs, lines)
            bad = idx[~ok]
            if len(bad):
                outside = shapely.length(shapely.difference(lines[~ok], cs))
                outside = np.where(outside > tol, outside, 0.0)
                coll_len += float(outside.sum())
                coll_t += float((outside / L[bad] * dt[bad]).sum())
        still = np.flatnonzero(~moving)
        if len(still):
            ok = shapely.covers(cs, shapely.points(traj.xy[still]))
            coll_t += float(dt[still][~ok].sum())
        return coll_t, coll_len
    ftol = free.buffer(tol)
    shapely.prepare(ftol)
    for k in range(traj.K):
        p0, p1 = traj.xy[k], traj.xy[k + 1]
        if L[k] > EPS:
            psi = traj.psi[k]
            hull = shapely.convex_hull(shapely.union(body.polygon(*p0, psi), body.polygon(*p1, psi)))
            if ftol.covers(hull):
                continue
            n = max(2, int(math.ceil(L[k] / res)) + 1)
            s = np.linspace(0, 1, n)
            polys = [body.polygon(p0[0] + (p1[0] - p0[0]) * a, p0[1] + (p1[1] - p0[1]) * a, psi) for a in s]
            frac = 1.0 - float(np.mean(shapely.covers(ftol, polys)))
            coll_t += frac * dt[k]
            coll_len += frac * L[k]
        else:
            d = float(wrap_angle(traj.psi[k + 1] - traj.psi[k]))
            n = max(2, int(math.ceil(abs(d) / math.radians(5))) + 1)
            polys = [body.polygon(p0[0], p0[1], traj.psi[k] + a) for a in np.linspace(0, d, n)]
            if not np.all(shapely.covers(ftol, polys)):
                coll_t += dt[k]
    return coll_t, coll_len


def inter_robot_conflicts(retimed: list[Trajectory], team, res: float, block: int = 4000) -> dict:
    """Sample the re-timed trajectories so that no robot moves more than ``res``
    between samples and count instants at which two body footprints overlap.

    Rectangular bodies are approximated conservatively by their circumscribed disc.
    """
    robots = {r.id: r for r in team}
    m = len(retimed)
    out = {"conflict_instants": 0, "conflict_pair_instants": 0, "conflict_time": 0.0, "conflict_dt": None}
    if m < 2:
        return out
    T = max(tr.end_time for tr in retimed)
    vmax = max(robots[tr.robot_id].speed for tr in retimed)
    dt = res / vmax if vmax > 0 else T
    if T <= 0:
        return out
    radii = np.array([robots[tr.robot_id].body_fp.radius for tr in retimed])
    thr = radii[:, None] + radii[None, :] - 1e-6 * max(1.0, radii.max())
    iu = np.triu_indices(m, k=1)
    times_all = np.arange(0.0, T + dt, dt)
    inst = 0
    pairs = 0
    for b0 in range(0, len(times_all), block):
        ts = times_all[b0:b0 + block]
        P = np.stack([tr.position_at(ts) for tr in retimed])  # (m, S, 2)
        D = np.hypot(P[:, None, :, 0] - P[None, :, :, 0], P[:, None, :, 1] - P[None, :, :, 1])
        over = D[iu[0], iu[1]] < thr[iu][:, None]  # (pairs, S)
        pairs += int(over.sum())
        inst += int(over.any(axis=0).sum())
    out.update(conflict_instants=inst, conflict_pair_instants=pairs, conflict_time=inst * dt, conflict_dt=dt)
    return out


def capacity_violations(traj: Trajectory, robot, pos_tol: float) -> float:
    """Largest serviced area between two depot visits minus the capacity (<=0 is feasible)."""
    if robot.capacity is None:
        return -math.inf
    depot = robot.depot if robot.depot is not None else robot.start[:2]
    L = traj.seg_lengths()
    serv = np.where(traj.active & (traj.labels != TURN) & (traj.labels != WAIT), L, 0.0) * robot.working_width
    at_depot = np.hypot(traj.xy[:, 0] - depot[0], traj.xy[:, 1] - depot[1]) <= pos_tol
    worst = 0.0
    acc = 0.0
    for k in range(traj.K):
        if at_depot[k]:
            acc = 0.0
        acc += serv[k]
        worst = max(worst, acc)
    return worst - robot.capacity

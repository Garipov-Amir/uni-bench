"""Prioritized path deconfliction by waiting (deconfliction profile).

Robots are processed in priority order; a lower-priority robot waits at the
start of a segment whenever executing it would bring its body into contact
with an already fixed higher-priority robot (cf. the path deconfliction of
Tang, Mao & Ma, 2025).  Remaining conflicts are reported, and the evaluator
enforces them as hard constraints in the deconfliction profile.
"""

from __future__ import annotations

import math

import numpy as np

from ..trajectory import WAIT, Trajectory, TrajectorySet, retime


def maybe_deconflict(ts: TrajectorySet, rep, ctx):
    if "deconfliction" not in ctx.profiles or len(ts) < 2:
        return ts, {}
    return deconflict(ts, rep.team)


def deconflict(ts: TrajectorySet, team, res: float | None = None, max_wait: float | None = None):
    res = res or 0.1 * min(r.working_width for r in team)
    fixed: list[tuple[Trajectory, float]] = []
    out = TrajectorySet(metadata=dict(ts.metadata))
    info = {"waits_inserted": 0, "wait_time": 0.0, "unresolved": []}
    # priority: longest trajectories first (they park last, so fewer robots run into parked ones)
    order = sorted((r for r in team if r.id in ts.trajectories),
                   key=lambda r: -retime(ts[r.id], r.speed, r.turn_cost).end_time)
    for r in order:
        if r.id not in ts.trajectories:
            continue
        tr = ts[r.id]
        if not fixed:
            out.add(tr)
            fixed.append((retime(tr, r.speed, r.turn_cost), r.body_fp.radius))
            continue
        rt = retime(tr, r.speed, r.turn_cost)
        dt = np.diff(rt.t)
        step = max(res / r.speed, 1e-3) * 5
        limit = max_wait if max_wait is not None else max(60.0, 0.25 * rt.end_time)
        xy, psi, labels, active, wait = [tr.xy[0]], [tr.psi[0]], [], [], []
        t = 0.0
        total_wait = 0.0
        rb = r.body_fp.radius

        def conflict(p0, p1, t0, d):
            n = max(2, int(math.ceil(max(np.hypot(*(p1 - p0)), 1e-9) / res)) + 1)
            times = t0 + np.linspace(0.0, max(d, 1e-9), n)
            a = np.linspace(0.0, 1.0, n)[:, None]
            P = p0[None, :] * (1 - a) + p1[None, :] * a
            for f, fb in fixed:
                Q = f.position_at(times)
                if np.any(np.hypot(*(P - Q).T) < rb + fb - 1e-9):
                    return True
            return False

        for k in range(rt.K):
            p0, p1 = rt.xy[k], rt.xy[k + 1]
            waited = 0.0
            while conflict(p0, p1, t, dt[k]) and waited < limit:
                xy.append(p0)
                psi.append(psi[-1])
                labels.append(WAIT)
                active.append(False)
                wait.append(step)
                t += step
                waited += step
                info["waits_inserted"] += 1
            total_wait += waited
            if waited >= limit:
                info["unresolved"].append(r.id)
            xy.append(p1)
            psi.append(rt.psi[k + 1])
            labels.append(rt.labels[k])
            active.append(bool(rt.active[k]))
            wait.append(float(rt.wait[k]))
            t += dt[k]
        new = Trajectory(r.id, np.array(xy), np.array(psi), np.array(labels), np.array(active), np.array(wait))
        info["wait_time"] += total_wait
        out.add(new)
        fixed.append((retime(new, r.speed, r.turn_cost), rb))
    info["unresolved"] = sorted(set(info["unresolved"]))
    return out, {"deconfliction": info}

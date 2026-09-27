"""Dubins shortest paths (forward motion, minimum turning radius).

Used by planners that support the agricultural profile.  Each of the six
words is computed in closed form, sampled, and accepted only if the sampled
end pose matches the goal, which guards the closed-form expressions.
"""

from __future__ import annotations

import math

import numpy as np

TWO_PI = 2 * math.pi


def _mod(a: float) -> float:
    return a - TWO_PI * math.floor(a / TWO_PI)


def _words(alpha, beta, d):
    sa, sb, ca, cb = math.sin(alpha), math.sin(beta), math.cos(alpha), math.cos(beta)
    cab = math.cos(alpha - beta)
    out = {}
    p2 = 2 + d * d - 2 * cab + 2 * d * (sa - sb)
    if p2 >= 0:
        tmp = math.atan2(cb - ca, d + sa - sb)
        out["LSL"] = (_mod(-alpha + tmp), math.sqrt(p2), _mod(beta - tmp))
    p2 = 2 + d * d - 2 * cab + 2 * d * (sb - sa)
    if p2 >= 0:
        tmp = math.atan2(ca - cb, d - sa + sb)
        out["RSR"] = (_mod(alpha - tmp), math.sqrt(p2), _mod(-beta + tmp))
    p2 = -2 + d * d + 2 * cab + 2 * d * (sa + sb)
    if p2 >= 0:
        p = math.sqrt(p2)
        tmp = math.atan2(-ca - cb, d + sa + sb) - math.atan2(-2.0, p)
        out["LSR"] = (_mod(-alpha + tmp), p, _mod(-_mod(beta) + tmp))
    p2 = -2 + d * d + 2 * cab - 2 * d * (sa + sb)
    if p2 >= 0:
        p = math.sqrt(p2)
        tmp = math.atan2(ca + cb, d - sa - sb) - math.atan2(2.0, p)
        out["RSL"] = (_mod(alpha - tmp), p, _mod(beta - tmp))
    tmp = (6.0 - d * d + 2 * cab + 2 * d * (sa - sb)) / 8.0
    if abs(tmp) <= 1:
        p = _mod(TWO_PI - math.acos(tmp))
        t = _mod(alpha - math.atan2(ca - cb, d - sa + sb) + _mod(p / 2.0))
        out["RLR"] = (t, p, _mod(alpha - beta - t + _mod(p)))
    tmp = (6.0 - d * d + 2 * cab + 2 * d * (-sa + sb)) / 8.0
    if abs(tmp) <= 1:
        p = _mod(TWO_PI - math.acos(tmp))
        t = _mod(-alpha - math.atan2(ca - cb, d + sa - sb) + p / 2.0)
        out["LRL"] = (t, p, _mod(_mod(beta) - alpha - t + _mod(p)))
    return out


def _sample(start, word, params, rho, step):
    x, y, th = start
    pts = [(x, y, th)]
    for mode, prm in zip(word, params):
        seg_len = prm * rho
        n = max(1, int(math.ceil(seg_len / step)))
        for k in range(1, n + 1):
            s = seg_len * k / n
            if mode == "S":
                px, py, pth = x + s * math.cos(th), y + s * math.sin(th), th
            elif mode == "L":
                phi = s / rho
                px = x + rho * (math.sin(th + phi) - math.sin(th))
                py = y + rho * (-math.cos(th + phi) + math.cos(th))
                pth = th + phi
            else:
                phi = s / rho
                px = x + rho * (-math.sin(th - phi) + math.sin(th))
                py = y + rho * (math.cos(th - phi) - math.cos(th))
                pth = th - phi
            pts.append((px, py, pth))
        x, y, th = pts[-1]
    return np.array(pts)


def dubins_paths(start, goal, rho: float, step: float):
    """All valid Dubins paths from start to goal, shortest first.

    Returns a list of ``(length, word, samples)`` with samples of shape (N, 3).
    """
    x0, y0, t0 = start
    x1, y1, t1 = goal
    dx, dy = x1 - x0, y1 - y0
    D = math.hypot(dx, dy)
    d = D / rho
    theta = _mod(math.atan2(dy, dx)) if D > 1e-12 else 0.0
    alpha = _mod(t0 - theta)
    beta = _mod(t1 - theta)
    res = []
    tol = max(1e-6, 1e-4 * rho)
    for word, prm in _words(alpha, beta, d).items():
        pts = _sample(start, word, prm, rho, step)
        end = pts[-1]
        err = math.hypot(end[0] - x1, end[1] - y1)
        herr = abs((end[2] - t1 + math.pi) % TWO_PI - math.pi)
        if err <= tol and herr <= 1e-4:
            pts[-1] = (x1, y1, t1)
            res.append((sum(prm) * rho, word, pts))
    res.sort(key=lambda r: r[0])
    return res

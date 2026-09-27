"""Deriving team-size and start-configuration variants of an instance."""

from __future__ import annotations

import copy
import math

import numpy as np
import shapely
from shapely.geometry import Point
from shapely.ops import nearest_points

from ..geometry.decomposition import polygons_of
from .model import Instance, Robot

START_CONFIGS = ("clustered", "dispersed", "given")


def sample_points(geom, n: int, rng: np.random.Generator, max_rounds: int = 50) -> np.ndarray:
    """Uniform samples inside a polygonal geometry (rejection sampling)."""
    if geom.is_empty or n <= 0:
        return np.zeros((0, 2))
    minx, miny, maxx, maxy = geom.bounds
    shapely.prepare(geom)
    frac = max(geom.area / max((maxx - minx) * (maxy - miny), 1e-12), 1e-3)
    out = []
    got = 0
    for _ in range(max_rounds):
        k = int(1.3 * (n - got) / frac) + 16
        pts = np.c_[rng.uniform(minx, maxx, k), rng.uniform(miny, maxy, k)]
        ok = shapely.contains_xy(geom, pts[:, 0], pts[:, 1])
        out.append(pts[ok])
        got += int(ok.sum())
        if got >= n:
            break
    pts = np.concatenate(out) if out else np.zeros((0, 2))
    return pts[:n]


def _component(cspace, anchor):
    comps = polygons_of(cspace)
    if not comps:
        return None, anchor
    p = Point(anchor)
    d = [c.distance(p) for c in comps]
    k = int(np.argmin(d))
    comp = comps[k]
    if d[k] > 0:
        q = nearest_points(comp.buffer(-1e-9 * max(1.0, comp.length)), p)[0]
        anchor = (q.x, q.y)
    return comp, anchor


def start_positions(cspace, m: int, config: str, anchor, spacing: float, rng: np.random.Generator) -> np.ndarray:
    comp, anchor = _component(cspace, anchor)
    if comp is None:
        raise ValueError("empty configuration space: no start position possible")
    inner = comp.buffer(-1e-6 * spacing)
    if inner.is_empty:
        inner = comp
    shapely.prepare(inner)
    ax, ay = anchor
    if config == "clustered":
        radius = spacing * (math.sqrt(m) + 2)
        for _ in range(12):
            k = int(math.ceil(radius / spacing))
            g = np.arange(-k, k + 1) * spacing
            X, Y = np.meshgrid(ax + g, ay + g)
            pts = np.c_[X.ravel(), Y.ravel()]
            pts = pts[shapely.contains_xy(inner, pts[:, 0], pts[:, 1])]
            if len(pts) >= m:
                order = np.argsort(np.hypot(pts[:, 0] - ax, pts[:, 1] - ay), kind="stable")
                return pts[order[:m]]
            radius *= 2
        raise ValueError("not enough room for a clustered start configuration")
    if config == "dispersed":
        cand = sample_points(inner, max(2000, 50 * m), rng)
        if len(cand) < m:
            raise ValueError("not enough room for a dispersed start configuration")
        chosen = [int(np.argmin(np.hypot(cand[:, 0] - ax, cand[:, 1] - ay)))]
        dmin = np.hypot(*(cand - cand[chosen[0]]).T)
        for _ in range(m - 1):
            nxt = int(np.argmax(dmin))
            chosen.append(nxt)
            dmin = np.minimum(dmin, np.hypot(*(cand - cand[nxt]).T))
        return cand[chosen]
    raise ValueError(f"unknown start configuration {config!r}")


def with_team(inst: Instance, m: int, start_config: str = "clustered", seed: int = 0,
              template: Robot | None = None) -> Instance:
    """Derive an instance with ``m`` identical robots (core profile).

    ``given`` keeps the instance's own team when its size equals ``m``.
    Start positions are deterministic functions of the instance, ``m``, the
    configuration and ``seed``.
    """
    if start_config == "given":
        if inst.m != m:
            raise ValueError(f"instance {inst.id} has {inst.m} robots, not {m}")
        return inst
    template = template or inst.team[0]
    rng = np.random.default_rng([seed, m, sum(map(ord, start_config)), int(inst.hash[:8], 16)])
    body = template.body_fp
    cspace = body.erode(inst.free)
    spacing = 2.0 * body.radius * 1.1
    pts = start_positions(cspace, m, start_config, template.start[:2], spacing, rng)
    team = []
    for k, (x, y) in enumerate(pts):
        r = copy.deepcopy(template)
        r.id = f"r{k}"
        r.start = (float(x), float(y), float(template.start[2]))
        team.append(r)
    base = inst.metadata.get("base_id", inst.id)
    suffix = f"__m{m}_{start_config}" + (f"_s{seed}" if seed else "")
    meta = copy.deepcopy(inst.metadata)
    meta["base_id"] = base
    meta.setdefault("factors", {})
    meta["factors"]["team_size"] = m
    meta["factors"]["start_config"] = start_config
    return inst.copy(id=base + suffix, team=team, metadata=meta)

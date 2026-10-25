"""IF5: seeded synthetic instance generator.

Every parameter, including the seed, is stored in the instance metadata so
that each synthetic instance can be regenerated exactly (:func:`regenerate`).
The generator varies environment size, obstacle density and shape, and the
non-convexity of the boundary.
"""

from __future__ import annotations

import itertools
from pathlib import Path

import numpy as np
from shapely import affinity
from shapely.geometry import Point, Polygon, box
from shapely.ops import nearest_points

from ..geometry.decomposition import polygons_of
from ..geometry.footprint import Footprint
from ..instance.model import Environment, EvalSettings, Instance, Robot, TaskSpec

GENERATOR_VERSION = "1.0"
DEFAULTS = {
    "width": 40.0,
    "height": 30.0,
    "obstacle_density": 0.1,
    "obstacle_shape": "mixed",  # rect | polygon | circle | mixed
    "obstacle_size": [0.06, 0.16],  # fraction of the smaller side
    "nonconvexity": 0,  # number of rectangular notches cut into the boundary
    "notch_depth": 0.25,  # fraction of the side
    "min_gap": 3.0,  # in working widths
    "robot_width": 1.0,
    "robot_kind": "disc",
    "speed": 1.0,
    "turn_cost": 0.25,
    "max_attempts": 4000,
}


def _shape(kind: str, rng, size: float) -> Polygon:
    if kind == "rect":
        w, h = size * rng.uniform(0.5, 1.0), size * rng.uniform(0.3, 1.0)
        g = box(-w / 2, -h / 2, w / 2, h / 2)
    elif kind == "circle":
        g = Point(0, 0).buffer(size / 2, quad_segs=8)
    elif kind == "polygon":
        n = int(rng.integers(3, 8))
        ang = np.sort(rng.uniform(0, 2 * np.pi, n))
        rad = size / 2 * rng.uniform(0.5, 1.0, n)
        g = Polygon(np.c_[rad * np.cos(ang), rad * np.sin(ang)]).convex_hull
        if rng.random() < 0.5 and n > 4:  # occasionally non-convex (L-shapes)
            cut = box(0, 0, size, size)
            h = g.difference(affinity.rotate(cut, rng.uniform(0, 360), origin=(0, 0)))
            g = h if h.geom_type == "Polygon" and h.area > 0.3 * g.area else g
    else:
        raise ValueError(kind)
    return affinity.rotate(g, float(rng.uniform(0, 180)), origin=(0, 0))


def _boundary(W: float, H: float, notches: int, depth: float, gap: float, rng) -> Polygon:
    b = box(0, 0, W, H)
    for _ in range(notches * 10):
        if len(b.interiors) or notches <= 0:
            break
        side = int(rng.integers(4))
        L = W if side in (0, 2) else H
        D = H if side in (0, 2) else W
        nw = rng.uniform(0.15, 0.3) * L
        nd = rng.uniform(0.5, 1.0) * depth * D
        pos = rng.uniform(gap, max(gap, L - nw - gap))
        if side == 0:
            cut = box(pos, -1, pos + nw, nd)
        elif side == 1:
            cut = box(W - nd, pos, W + 1, pos + nw)
        elif side == 2:
            cut = box(pos, H - nd, pos + nw, H + 1)
        else:
            cut = box(-1, pos, nd, pos + nw)
        nb = b.difference(cut)
        if nb.geom_type == "Polygon" and nb.buffer(-gap / 2).geom_type == "Polygon":
            b = nb
            notches -= 1
            if notches == 0:
                break
    return b


def generate(seed: int = 0, iid: str | None = None, **params) -> Instance:
    p = {**DEFAULTS, **params}
    unknown = set(params) - set(DEFAULTS)
    if unknown:
        raise ValueError(f"unknown generator parameters {sorted(unknown)}")
    rng = np.random.default_rng(seed)
    W, H = float(p["width"]), float(p["height"])
    w = float(p["robot_width"])
    gap = float(p["min_gap"]) * w
    boundary = _boundary(W, H, int(p["nonconvexity"]), float(p["notch_depth"]), gap, rng)
    inner = boundary.buffer(-gap)
    target = float(p["obstacle_density"]) * boundary.area
    lo, hi = p["obstacle_size"]
    side = min(W, H)
    obstacles: list[Polygon] = []
    area = 0.0
    kinds = ["rect", "polygon", "circle"] if p["obstacle_shape"] == "mixed" else [p["obstacle_shape"]]
    for _ in range(int(p["max_attempts"])):
        if area >= target:
            break
        size = side * rng.uniform(lo, hi)
        g = _shape(kinds[int(rng.integers(len(kinds)))], rng, size)
        minx, miny, maxx, maxy = inner.bounds
        g = affinity.translate(g, rng.uniform(minx, maxx), rng.uniform(miny, maxy))
        if not inner.contains(g):
            continue
        if any(g.distance(o) < gap for o in obstacles):
            continue
        obstacles.append(g)
        area += g.area
    env = Environment(boundary=boundary, holes=obstacles)
    robot = Robot(id="r0", start=(0.0, 0.0, 0.0), tool=Footprint(p["robot_kind"], w), speed=float(p["speed"]),
                  turn_cost=float(p["turn_cost"]))
    cs = robot.body_fp.erode(env.free)
    comps = polygons_of(cs)
    main = max(comps, key=lambda c: c.area)
    corner = Point(0, 0)
    q = nearest_points(main.buffer(-1e-6 * w), corner)[0]
    robot.start = (round(q.x, 9), round(q.y, 9), 0.0)
    gen = {"kind": "synthetic", "version": GENERATOR_VERSION, "seed": int(seed), "params": p}
    factors = {"width": W, "height": H, "obstacle_density": float(p["obstacle_density"]),
               "obstacle_shape": p["obstacle_shape"], "nonconvexity": int(p["nonconvexity"])}
    iid = iid or (f"syn_{int(W)}x{int(H)}_d{p['obstacle_density']:.2f}_{p['obstacle_shape']}"
                  f"_n{p['nonconvexity']}_s{seed}")
    return Instance(id=iid, environment=env, team=[robot], family="IF5", task=TaskSpec(), evaluation=EvalSettings(),
                    metadata={"source": "synthetic", "generator": gen, "factors": factors,
                              "realized_density": area / boundary.area, "components": len(comps)})


def regenerate(inst: Instance) -> Instance:
    gen = inst.metadata["generator"]
    return generate(seed=gen["seed"], iid=inst.id, **gen["params"])


def generate_family(out_dir: str | Path | None = None, count: int = 5, base_seed: int = 0, sizes=None,
                    densities=None, shapes=None, nonconvexities=None, aspect: float = 0.75, **params):
    """Full-factorial sweep; each combination gets ``count`` seeded instances."""
    sizes = sizes or [params.pop("width", DEFAULTS["width"])]
    densities = densities or [params.pop("obstacle_density", DEFAULTS["obstacle_density"])]
    shapes = shapes or [params.pop("obstacle_shape", DEFAULTS["obstacle_shape"])]
    nonconvexities = nonconvexities or [params.pop("nonconvexity", DEFAULTS["nonconvexity"])]
    params.pop("height", None)
    out = []
    for k, (size, dens, shape, nc) in enumerate(itertools.product(sizes, densities, shapes, nonconvexities)):
        for i in range(count):
            seed = base_seed + 1000 * k + i
            inst = generate(seed=seed, width=float(size), height=float(size) * aspect, obstacle_density=float(dens),
                            obstacle_shape=shape, nonconvexity=int(nc), **params)
            out.append(inst)
            if out_dir is not None:
                inst.save(Path(out_dir))
    return out


def scale_family(out_dir=None, sizes=(20, 40, 80, 160), density: float = 0.1, count: int = 3, base_seed: int = 0,
                 **params):
    """Environment-size sweep at constant obstacle density (Engelsons et al., 2022)."""
    return generate_family(out_dir, count=count, base_seed=base_seed, sizes=list(sizes), densities=[density],
                           **params)


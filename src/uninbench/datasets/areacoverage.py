"""IF1: AreaCoverage dataset (Agarwal & Akella, 2022): VM25, AC300, H2, RAL_main.

Layout (github.com/UNCCharlotte-CS-Robotics/AreaCoverage-dataset)::

    <subset>/<instance>/outer_polygon     "x y" per line
    <subset>/<instance>/holes             hole polygons separated by blank lines
    <subset>/<instance>/robot_polygon     optional robot polygon (VM25)
    <subset>/<instance>/env_<n>.wkt       optional original WKT (VM25)
    <subset>/<instance>/mem_*/..._route_data<k>   reference route of robot k: "x y flag"

Scales: VM25 1 unit = 0.1 m; AC300 and H2 1 unit = 1 m.  The route flag marks
whether the travel from a point to the next one is servicing (1) or
deadheading (0) (``flag_convention="from"``); ``"to"`` reads the flag as
describing the travel *to* the point instead.  The evaluator sanity check
(:mod:`uninbench.validation.reference`) reports which convention yields
complete coverage.
"""

from __future__ import annotations

import re
from pathlib import Path

import numpy as np
import shapely
from shapely import affinity
from shapely.geometry import Point, Polygon
from shapely.ops import nearest_points

from ..geometry.decomposition import polygons_of
from ..geometry.footprint import Footprint
from ..instance.model import Environment, Instance, Robot
from ..trajectory import SERVICE, TRANSIT, TrajectorySet, from_labelled_polyline

# tool: working width (None: from robot_polygon); body: diameter of the body disc used for
# collision checking ("inscribed": inscribed disc of the robot polygon, which translates
# without rotating in the original formulation).  AC300 models aerial robots: a 3 m
# square sensor footprint carried by a point-like vehicle whose routes may run along the
# workspace boundary (body 1e-4 m).
SUBSETS = {
    "VM25": {"scale": 0.1, "width": None, "tool_kind": "rect", "body": "inscribed"},
    "AC300": {"scale": 1.0, "width": 3.0, "tool_kind": "rect", "body": 1e-4},
    "H2": {"scale": 1.0, "width": 1.0, "tool_kind": "rect", "body": 1e-4},
    "RAL_main": {"scale": 1.0, "width": 1.0, "tool_kind": "rect", "body": 1e-4},
}


def read_blocks(path) -> list[np.ndarray]:
    path = Path(path)
    blocks, cur = [], []
    for line in path.read_text().splitlines():
        s = line.strip()
        if not s:
            if cur:
                blocks.append(np.array(cur, dtype=float))
                cur = []
            continue
        cur.append([float(v) for v in re.split(r"[\s,]+", s)[:3]])
    if cur:
        blocks.append(np.array(cur, dtype=float))
    return blocks


def load_environment(d: Path, scale: float) -> Environment:
    if (d / "outer_polygon").exists():
        outer = read_blocks(d / "outer_polygon")[0][:, :2] * scale
        holes = []
        if (d / "holes").exists():
            holes = [Polygon(b[:, :2] * scale) for b in read_blocks(d / "holes") if len(b) >= 3]
        return Environment(boundary=shapely.make_valid(Polygon(outer)), holes=[shapely.make_valid(h) for h in holes])
    wkts = sorted(d.glob("*.wkt"))
    if not wkts:
        raise FileNotFoundError(f"{d}: neither outer_polygon nor *.wkt")
    g = affinity.scale(shapely.from_wkt(wkts[0].read_text()), scale, scale, origin=(0, 0))
    polys = polygons_of(g)
    boundary = shapely.unary_union([Polygon(p.exterior) for p in polys])
    return Environment(boundary=boundary, holes=[Polygon(h) for p in polys for h in p.interiors])


def load_route(path: Path, scale: float, flag_convention: str = "from"):
    a = read_blocks(path)[0]
    pts = a[:, :2] * scale
    flags = a[:, 2].astype(int) if a.shape[1] > 2 else np.ones(len(a), dtype=int)
    seg = flags[:-1] if flag_convention == "from" else flags[1:]
    labels = [SERVICE if f == 1 else TRANSIT for f in seg]
    return pts, labels


def route_sets(d: Path, scale: float, flag_convention: str = "from") -> dict[str, list]:
    out = {}
    for mem in sorted(p for p in d.glob("mem_*") if p.is_dir()):
        files = sorted(mem.glob("*route_data*"), key=lambda p: int(re.findall(r"(\d+)$", p.name)[0])
                       if re.findall(r"(\d+)$", p.name) else 0)
        if files:
            out[mem.name] = [load_route(f, scale, flag_convention) for f in files]
    return out


def robot_footprints(d: Path, scale: float, width: float | None, tool_kind: str = "rect", body="inscribed"):
    """(tool, body) footprints of the dataset robot."""
    if width:
        tool = Footprint(tool_kind, float(width), float(width) if tool_kind == "rect" else None)
    elif (d / "robot_polygon").exists():
        tool = Footprint.from_polygon(read_blocks(d / "robot_polygon")[0][:, :2] * scale)
    else:
        tool = Footprint("disc", 1.0)
    if body == "inscribed":
        bfp = Footprint("disc", 2 * tool.inradius)
    elif body is None:
        bfp = None
    else:
        bfp = Footprint("disc", float(body))
    return tool, bfp


def _start_in_cspace(env: Environment, fp: Footprint, hint) -> tuple[float, float]:
    cs = fp.erode(env.free)
    comps = polygons_of(cs)
    if not comps:
        return tuple(hint)
    p = Point(hint)
    comp = min(comps, key=lambda c: c.distance(p))
    if comp.covers(p):
        return tuple(hint)
    q = nearest_points(comp.buffer(-1e-6 * fp.width), p)[0]
    return (q.x, q.y)


def import_instance(d: Path, subset: str, scale: float | None = None, width: float | None = None,
                    flag_convention: str = "from") -> Instance:
    d = Path(d)
    cfg = SUBSETS.get(subset, {"scale": 1.0, "width": 1.0})
    scale = cfg["scale"] if scale is None else scale
    width = cfg["width"] if width is None else width
    env = load_environment(d, scale)
    tool, body = robot_footprints(d, scale, width, cfg.get("tool_kind", "rect"), cfg.get("body", "inscribed"))
    fp = body or tool
    routes = route_sets(d, scale, flag_convention)
    hint = next(iter(routes.values()))[0][0][0] if routes else np.asarray(env.boundary.exterior.coords[0]
                                                                           if env.boundary.geom_type == "Polygon"
                                                                           else polygons_of(env.boundary)[0]
                                                                           .exterior.coords[0])
    start = _start_in_cspace(env, fp, hint)
    robot = Robot(id="r0", start=(start[0], start[1], 0.0), tool=tool, body=body)
    meta = {"source": "AreaCoverage", "subset": subset, "source_dir": d.name, "scale": scale,
            "reference_routes": sorted(routes), "flag_convention": flag_convention,
            "factors": {"holes": len(env.holes), "subset": subset}}
    return Instance(id=f"ac_{subset}_{d.name}", environment=env, team=[robot], family="IF1", metadata=meta)


def reference_trajectories(inst: Instance, d: Path, mem: str, flag_convention: str = "from"):
    """(team-adapted instance, TrajectorySet) for a stored reference solution."""
    scale = inst.metadata.get("scale", 1.0)
    routes = route_sets(Path(d), scale, flag_convention)[mem]
    tmpl = inst.team[0]
    team, ts = [], TrajectorySet(metadata={"planner": f"reference:{mem}"})
    for k, (pts, labels) in enumerate(routes):
        r = Robot(id=f"r{k}", start=(pts[0, 0], pts[0, 1], 0.0), tool=tmpl.tool, body=tmpl.body, speed=tmpl.speed,
                  turn_cost=tmpl.turn_cost, depot=(pts[0, 0], pts[0, 1]))
        team.append(r)
        ts.add(from_labelled_polyline(r.id, pts, labels, start_psi=0.0, tool_active_in_turns=True))
    returns = all(np.hypot(*(pts[-1] - pts[0])) < 1e-6 for pts, _ in routes)
    new = inst.copy(id=f"{inst.id}__{mem}", team=team)
    new.task.return_to_depot = bool(returns)
    return new, ts


def import_areacoverage(src, out_dir=None, subsets=None, width: float | None = None, limit: int | None = None,
                        flag_convention: str = "from") -> list[Instance]:
    src = Path(src)
    subsets = subsets or [s for s in SUBSETS if (src / s).is_dir()]
    out = []
    for sub in subsets:
        dirs = sorted(p for p in (src / sub).iterdir() if p.is_dir())
        for d in dirs[:limit] if limit else dirs:
            try:
                inst = import_instance(d, sub, width=width, flag_convention=flag_convention)
            except (FileNotFoundError, ValueError) as e:
                print(f"skipping {d}: {e}")
                continue
            inst.metadata["source_path"] = str(d)
            out.append(inst)
            if out_dir is not None:
                inst.save(Path(out_dir))
    return out


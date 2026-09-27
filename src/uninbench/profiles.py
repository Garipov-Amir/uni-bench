"""Optional profiles (R12): domain-specific constraints on top of the core benchmark.

A planner takes part in a profile only if its manifest declares every
constraint of the profile (see :data:`uninbench.planners.PROFILE_CONSTRAINTS`);
otherwise it is evaluated in the core profile only.
"""

from __future__ import annotations

import math

from shapely.geometry import Point
from shapely.ops import nearest_points

from .geometry.footprint import Footprint
from .instance.model import Instance

PROFILE_DOC = {
    "core": "one-shot direct coverage with identical robots; conflicts reported, not enforced",
    "agricultural": "capacity (depot trips), minimum turning radius, headlands; mission time includes depot trips",
    "deconfliction": "inter-robot conflicts are hard constraints; makespan measured after deconfliction",
    "heterogeneous": "robots differ in speed, footprint or capacity",
}

AGRI_DEFAULTS = {
    "min_turn_radius": 2.0,  # in working widths
    "capacity": 0.25,  # fraction of the coverable area serviced per depot visit
    "tool_length": 0.2,  # implement length in working widths (rectangular tool)
    "return_to_depot": True,
    "headland": None,  # default: turning radius + one working width
}


def apply_profile(inst: Instance, profile: str, params: dict | None = None) -> Instance:
    params = dict(params or {})
    new = inst.copy()
    if profile not in new.task.profiles:
        new.task.profiles = [p for p in new.task.profiles if p != "core"] + [profile]
    if profile == "core":
        new.task.profiles = ["core"]
    elif profile == "agricultural":
        p = {**AGRI_DEFAULTS, **params}
        area = inst.derived.coverable_area
        for r in new.team:
            w = r.working_width
            r.min_turn_radius = float(p["min_turn_radius"]) * w
            r.capacity = float(p["capacity"]) * area if p["capacity"] else None
            if not p.get("depot"):
                # depot / start where a full turning circle fits (field entrance with turning room)
                room = r.body_fp.erode(inst.free).buffer(-2 * r.min_turn_radius)
                if not room.is_empty:
                    q = nearest_points(room.buffer(-1e-6 * w), Point(r.start[:2]))[0]
                    c = room.centroid
                    r.start = (q.x, q.y, math.atan2(c.y - q.y, c.x - q.x))
            r.depot = tuple(p["depot"]) if p.get("depot") else (r.start[0], r.start[1])
            if p["tool_length"]:
                r.body = r.body_fp
                r.tool = Footprint("rect", w, float(p["tool_length"]) * w)
        new.task.return_to_depot = bool(p["return_to_depot"])
        # default headland: room for a U-turn of the turning radius beyond the swath ends
        # (planners use skip patterns so that consecutive swaths are >= 2 r apart)
        headland = p["headland"] if p["headland"] is not None else (float(p["min_turn_radius"]) + 1.0) * \
            inst.min_width
        new.metadata["profile_params"] = {"agricultural": {**p, "headland": headland}}
    elif profile == "deconfliction":
        new.evaluation.conflicts = "enforce"
    elif profile == "heterogeneous":
        spread = float(params.get("speed_spread", 0.5))
        wspread = float(params.get("width_spread", 0.0))
        m = len(new.team)
        for k, r in enumerate(new.team):
            a = (k / (m - 1) - 0.5) if m > 1 else 0.0
            r.speed = r.speed * (1 + spread * a)
            if wspread:
                r.tool = Footprint(r.tool.kind, r.tool.width * (1 + wspread * a), r.tool.length)
        new.metadata["profile_params"] = {"heterogeneous": {"speed_spread": spread, "width_spread": wspread}}
    else:
        raise ValueError(f"unknown profile {profile!r}")
    new.__dict__.pop("hash", None)
    new.__dict__.pop("_derived", None)
    return new


def profile_adapter_params(inst: Instance, adapter: str) -> dict:
    """Adapter defaults implied by a profile (e.g. headlands for swaths)."""
    pp = inst.metadata.get("profile_params", {})
    if "agricultural" in pp and adapter == "swath":
        return {"headland": float(pp["agricultural"]["headland"]), "perimeter": False}
    return {}


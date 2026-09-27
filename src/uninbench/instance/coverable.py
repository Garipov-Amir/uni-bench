"""Reachable coverable set F* and the makespan lower bound T_LB.

    F* = ( reach(F_c (-) B_body) ) (+) B_tool

computed once per instance and team so that every planner is judged against
the same target area.
"""

from __future__ import annotations

import math
import warnings

import shapely
from shapely.geometry import Point
from shapely.geometry.base import BaseGeometry

from ..geometry.decomposition import polygons_of
from .model import Derived, Instance, Robot


def configuration_space(free: BaseGeometry, robot: Robot, psi: float = 0.0) -> BaseGeometry:
    """Configuration space of the robot reference point, F_c (-) B_body."""
    return robot.body_fp.erode(free, psi)


def reachable(cspace: BaseGeometry, points, tol: float) -> tuple[BaseGeometry, list[int]]:
    """Connected components of ``cspace`` containing (or within ``tol`` of) a point."""
    comps = polygons_of(cspace)
    keep = set()
    missing = []
    for k, (x, y) in enumerate(points):
        p = Point(x, y)
        if not comps:
            missing.append(k)
            continue
        d = [c.distance(p) for c in comps]
        best = min(range(len(comps)), key=lambda i: d[i])
        if d[best] <= tol:
            keep.add(best)
        else:
            missing.append(k)
    if not keep:
        return shapely.Polygon(), missing
    return shapely.unary_union([comps[i] for i in sorted(keep)]), missing


def coverable_set(inst: Instance) -> tuple[BaseGeometry, BaseGeometry, list[str]]:
    free = inst.free
    psi = inst.evaluation.footprint_heading
    parts_cs, parts_cov, unreachable = [], [], []
    groups: dict[tuple, list[Robot]] = {}
    for r in inst.team:
        key = (r.body_fp, r.tool)
        groups.setdefault(key, []).append(r)
    for (body, tool), robots in groups.items():
        cs = body.erode(free, psi)
        tol = max(0.5 * body.radius, 0.01 * tool.radius, 1e-6)
        reach, missing = reachable(cs, [r.start[:2] for r in robots], tol)
        unreachable += [robots[k].id for k in missing]
        if reach.is_empty:
            continue
        parts_cs.append(reach)
        parts_cov.append(shapely.intersection(tool.dilate(reach, psi), free))
    if unreachable:
        warnings.warn(f"{inst.id}: start poses outside the configuration space: {unreachable}")
    cspace = shapely.unary_union(parts_cs) if parts_cs else shapely.Polygon()
    cov = shapely.unary_union(parts_cov) if parts_cov else shapely.Polygon()
    return cspace, cov, unreachable


def makespan_lower_bound(area: float, team: list[Robot]) -> float:
    """T_LB = max(0, mu(F*) - pi sum b_j^2) / (2 sum b_j v_j)."""
    num = max(0.0, area - math.pi * sum(r.tool.radius ** 2 for r in team))
    den = 2.0 * sum(r.tool.radius * r.speed for r in team)
    return num / den if den > 0 else math.inf


def compute_derived(inst: Instance) -> Derived:
    cspace, cov, unreachable = coverable_set(inst)
    area = float(cov.area)
    return Derived(cspace=cspace, coverable=cov, coverable_area=area,
                   T_LB=makespan_lower_bound(area, inst.team), unreachable_starts=unreachable)

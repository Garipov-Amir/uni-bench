"""Canonical benchmark instance (R1).

An instance consists of an environment (vector geometry in metres), a robot
team, a task specification and evaluation settings.  It is the single source
of truth: every planner input is derived from it by an adapter, and the
evaluator judges every trajectory set against it.
"""

from __future__ import annotations

import copy
import math
from dataclasses import dataclass, field
from functools import cached_property
from pathlib import Path
from typing import Any

import numpy as np
import shapely
from shapely.geometry import Polygon
from shapely.geometry.base import BaseGeometry

from .. import TASK_ONE_SHOT
from ..geometry.footprint import Footprint
from ..geometry.raster import RasterGrid
from ..util import content_hash, dump_json, load_json, pack_array, unpack_array

SCHEMA_VERSION = 1


def to_wkt(g: BaseGeometry) -> str:
    return shapely.to_wkt(g, rounding_precision=-1, trim=True)


def from_wkt(s: str) -> BaseGeometry:
    return shapely.from_wkt(s)


# ----------------------------------------------------------------- environment
@dataclass
class TerrainField:
    """Optional terrain-cost field c(x) >= 0 on a raster (for weighted planners)."""

    grid: RasterGrid
    values: np.ndarray

    def cost_at(self, x, y, default: float = 1.0):
        i, j = self.grid.cell_of(x, y)
        ok = self.grid.inside(i, j)
        out = np.full(np.shape(x), default, dtype=float)
        out[ok] = self.values[i[ok], j[ok]]
        return out

    def to_dict(self) -> dict:
        return {"grid": self.grid.to_dict(), "values": pack_array(self.values.astype(np.float32))}

    @classmethod
    def from_dict(cls, d: dict) -> "TerrainField":
        return cls(RasterGrid.from_dict(d["grid"]), unpack_array(d["values"]).astype(float))


@dataclass
class Environment:
    """Workspace W (outer boundary) and obstacle region O_c (holes)."""

    boundary: BaseGeometry
    holes: list[BaseGeometry] = field(default_factory=list)
    terrain: TerrainField | None = None
    source_resolution: float | None = None

    def __post_init__(self):
        if isinstance(self.boundary, str):
            self.boundary = from_wkt(self.boundary)
        self.holes = [from_wkt(h) if isinstance(h, str) else h for h in self.holes]

    @cached_property
    def free(self) -> BaseGeometry:
        """Free space F_c = W minus O_c."""
        f = self.boundary
        if self.holes:
            f = shapely.difference(f, shapely.unary_union(self.holes))
        f = shapely.make_valid(f) if not f.is_valid else f
        # keep polygonal parts only
        if f.geom_type == "GeometryCollection":
            f = shapely.unary_union([g for g in f.geoms if g.geom_type in ("Polygon", "MultiPolygon")])
        return f

    @property
    def area(self) -> float:
        return float(self.free.area)

    @property
    def bounds(self):
        return self.boundary.bounds

    def to_dict(self) -> dict:
        d: dict[str, Any] = {"boundary": to_wkt(self.boundary), "holes": [to_wkt(h) for h in self.holes]}
        if self.terrain is not None:
            d["terrain"] = self.terrain.to_dict()
        if self.source_resolution is not None:
            d["source_resolution"] = float(self.source_resolution)
        return d

    @classmethod
    def from_dict(cls, d: dict) -> "Environment":
        return cls(
            boundary=from_wkt(d["boundary"]),
            holes=[from_wkt(h) for h in d.get("holes", [])],
            terrain=TerrainField.from_dict(d["terrain"]) if d.get("terrain") else None,
            source_resolution=d.get("source_resolution"),
        )


# ------------------------------------------------------------------- robots
@dataclass
class Robot:
    id: str
    start: tuple[float, float, float]
    tool: Footprint = field(default_factory=lambda: Footprint("disc", 1.0))
    body: Footprint | None = None
    speed: float = 1.0
    turn_cost: float = 0.25  # seconds per radian of heading change
    min_turn_radius: float | None = None  # agricultural profile
    capacity: float | None = None  # serviced area per depot visit (agricultural profile)
    depot: tuple[float, float] | None = None

    def __post_init__(self):
        self.start = tuple(float(v) for v in self.start)
        if len(self.start) == 2:
            self.start = (self.start[0], self.start[1], 0.0)
        if self.depot is not None:
            self.depot = (float(self.depot[0]), float(self.depot[1]))

    @property
    def body_fp(self) -> Footprint:
        return self.body if self.body is not None else self.tool

    @property
    def working_width(self) -> float:
        return self.tool.working_width

    def to_dict(self) -> dict:
        d = {
            "id": self.id,
            "start": list(self.start),
            "tool": self.tool.to_dict(),
            "speed": self.speed,
            "turn_cost": self.turn_cost,
        }
        if self.body is not None:
            d["body"] = self.body.to_dict()
        for k in ("min_turn_radius", "capacity"):
            if getattr(self, k) is not None:
                d[k] = getattr(self, k)
        if self.depot is not None:
            d["depot"] = list(self.depot)
        return d

    @classmethod
    def from_dict(cls, d: dict) -> "Robot":
        return cls(
            id=str(d["id"]),
            start=tuple(d["start"]),
            tool=Footprint.from_dict(d["tool"]),
            body=Footprint.from_dict(d["body"]) if d.get("body") else None,
            speed=float(d.get("speed", 1.0)),
            turn_cost=float(d.get("turn_cost", 0.25)),
            min_turn_radius=d.get("min_turn_radius"),
            capacity=d.get("capacity"),
            depot=tuple(d["depot"]) if d.get("depot") is not None else None,
        )


# ----------------------------------------------------------------- task / eval
@dataclass
class TaskSpec:
    task_class: str = TASK_ONE_SHOT
    tolerance: float = 0.02  # completeness tolerance tau
    return_to_depot: bool = False
    time_limit: float = 60.0  # planning time limit [s]
    memory_limit_mb: int | None = 4096
    profiles: list[str] = field(default_factory=lambda: ["core"])

    def to_dict(self) -> dict:
        return dict(self.__dict__)

    @classmethod
    def from_dict(cls, d: dict) -> "TaskSpec":
        return cls(**{k: v for k, v in d.items() if k in cls.__dataclass_fields__})


@dataclass
class EvalSettings:
    resolution: float | None = None  # delta_e; default: resolution_factor * min working width
    resolution_factor: float = 0.1
    metrics: list[str] | None = None
    collision: str = "invalidate"
    conflicts: str = "report"  # 'report' (core) or 'enforce' (deconfliction profile)
    sensitivity: bool = True
    sensitivity_tolerance: float = 0.01
    max_refinements: int = 1
    turn_merge_distance: float | None = None  # default: half the working width
    footprint_heading: float = 0.0  # reference heading for non-disc footprints in F*

    def to_dict(self) -> dict:
        return dict(self.__dict__)

    @classmethod
    def from_dict(cls, d: dict) -> "EvalSettings":
        return cls(**{k: v for k, v in d.items() if k in cls.__dataclass_fields__})


# ------------------------------------------------------------------- instance
@dataclass
class Instance:
    id: str
    environment: Environment
    team: list[Robot]
    task: TaskSpec = field(default_factory=TaskSpec)
    evaluation: EvalSettings = field(default_factory=EvalSettings)
    family: str = "custom"
    metadata: dict = field(default_factory=dict)
    schema_version: int = SCHEMA_VERSION

    # -------------------------------------------------------------- identity
    def to_dict(self, include_hash: bool = True) -> dict:
        d = {
            "schema_version": self.schema_version,
            "id": self.id,
            "family": self.family,
            "environment": self.environment.to_dict(),
            "team": [r.to_dict() for r in self.team],
            "task": self.task.to_dict(),
            "evaluation": self.evaluation.to_dict(),
            "metadata": self.metadata,
        }
        if include_hash:
            d["hash"] = self.hash
        return d

    @cached_property
    def hash(self) -> str:
        d = self.to_dict(include_hash=False)
        d["metadata"] = {k: v for k, v in self.metadata.items() if k not in ("split", "notes")}
        return content_hash(d)

    @classmethod
    def from_dict(cls, d: dict) -> "Instance":
        inst = cls(
            id=d["id"],
            family=d.get("family", "custom"),
            environment=Environment.from_dict(d["environment"]),
            team=[Robot.from_dict(r) for r in d["team"]],
            task=TaskSpec.from_dict(d.get("task", {})),
            evaluation=EvalSettings.from_dict(d.get("evaluation", {})),
            metadata=d.get("metadata", {}),
            schema_version=d.get("schema_version", SCHEMA_VERSION),
        )
        return inst

    def save(self, path: str | Path) -> Path:
        path = Path(path)
        if path.is_dir() or path.suffix == "":
            path = path / f"{self.id}.json"
        dump_json(self.to_dict(), path)
        return path

    @classmethod
    def load(cls, path: str | Path) -> "Instance":
        return cls.from_dict(load_json(path))

    def copy(self, **changes) -> "Instance":
        d = copy.deepcopy(self.to_dict(include_hash=False))
        new = Instance.from_dict(d)
        for k, v in changes.items():
            setattr(new, k, v)
        new.__dict__.pop("hash", None)
        new.__dict__.pop("_derived", None)
        return new

    # ------------------------------------------------------------ properties
    @property
    def m(self) -> int:
        return len(self.team)

    @property
    def free(self) -> BaseGeometry:
        return self.environment.free

    @property
    def min_width(self) -> float:
        return min(r.working_width for r in self.team)

    @property
    def split(self) -> str | None:
        return self.metadata.get("split")

    def eval_resolution(self) -> float:
        if self.evaluation.resolution:
            return float(self.evaluation.resolution)
        return self.evaluation.resolution_factor * self.min_width

    def robot(self, rid: str) -> Robot:
        for r in self.team:
            if r.id == rid:
                return r
        raise KeyError(rid)

    # ------------------------------------------------------------- derived
    @property
    def derived(self) -> "Derived":
        if "_derived" not in self.__dict__:
            from .coverable import compute_derived

            self.__dict__["_derived"] = compute_derived(self)
        return self.__dict__["_derived"]

    def describe(self) -> dict:
        d = self.derived
        minx, miny, maxx, maxy = self.environment.bounds
        return {
            "id": self.id,
            "hash": self.hash,
            "family": self.family,
            "robots": self.m,
            "working_width": self.min_width,
            "extent": [maxx - minx, maxy - miny],
            "free_area": self.environment.area,
            "coverable_area": d.coverable_area,
            "holes": len(self.environment.holes),
            "T_LB": d.T_LB,
            "delta_e": self.eval_resolution(),
            "split": self.split,
            "factors": self.metadata.get("factors", {}),
        }


@dataclass
class Derived:
    """Quantities derived once per instance and team (evaluation settings)."""

    cspace: BaseGeometry  # union of reachable configuration spaces
    coverable: BaseGeometry  # F*
    coverable_area: float
    T_LB: float
    unreachable_starts: list[str]


def default_robot(width: float = 1.0, speed: float = 1.0, turn_cost: float = 0.25,
                  kind: str = "disc", start=(0.0, 0.0, 0.0), rid: str = "r0") -> Robot:
    return Robot(id=rid, start=tuple(start), tool=Footprint(kind, width), speed=speed, turn_cost=turn_cost)


def instance_from_polygon(iid: str, polygon: BaseGeometry, robot: Robot | None = None, family: str = "custom",
                          **meta) -> Instance:
    """Convenience constructor: polygon (with holes) -> canonical instance."""
    polys = [polygon] if polygon.geom_type == "Polygon" else list(polygon.geoms)
    boundary = shapely.unary_union([Polygon(p.exterior) for p in polys])
    holes = [Polygon(h) for p in polys for h in p.interiors]
    env = Environment(boundary=boundary, holes=holes)
    if robot is None:
        robot = default_robot()
    inst = Instance(id=iid, environment=env, team=[robot], family=family, metadata=dict(meta))
    return inst


def _finite(x: float) -> float:
    return x if math.isfinite(x) else float("inf")

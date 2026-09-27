"""Representation adapters  T_i : (E_c, team, theta_T) -> (R_i, rho_i)   (R2, R3)."""

from __future__ import annotations

import time
from dataclasses import dataclass, field

import numpy as np
from shapely.geometry.base import BaseGeometry

from .. import __version__
from ..geometry.raster import RasterGrid
from ..geometry.transit import TransitPlanner
from ..reconstruction import Reconstruction
from ..util import content_hash

DEFAULT_EPS_MAX = 0.05


@dataclass
class AdapterRecord:
    """Transformation record rho_i stored alongside every result."""

    adapter: str
    family: str
    version: str
    params: dict
    runtime: float
    fidelity: dict = field(default_factory=dict)
    info: dict = field(default_factory=dict)

    @property
    def flagged(self) -> bool:
        return bool(self.fidelity.get("flagged", False))

    @property
    def key(self) -> str:
        return content_hash({"adapter": self.adapter, "version": self.version, "params": self.params})

    def to_dict(self) -> dict:
        return {"adapter": self.adapter, "family": self.family, "version": self.version, "params": self.params,
                "runtime": self.runtime, "fidelity": self.fidelity, "info": self.info, "software": __version__}


class Representation:
    """Native planner input R_i.  Subclasses add family-specific structures."""

    family = "abstract"

    def __init__(self, team, free: BaseGeometry, params: dict):
        self.team = team
        self.free = free  # the adapter's view of the free space (possibly inflated)
        self.params = params
        self.reconstruction: Reconstruction | None = None

    @property
    def robots(self):
        return {r.id: r for r in self.team}

    def cspace(self, robot=None) -> BaseGeometry:
        """Reachable configuration space: components of (free (-) body) containing a start."""
        robot = robot or self.team[0]
        key = ("cspace", robot.body_fp)
        cache = self.__dict__.setdefault("_cache", {})
        if key not in cache:
            cache[key] = reachable_cspace(self.free, robot.body_fp, [r.start[:2] for r in self.team])
        return cache[key]

    def transit(self, robot=None) -> TransitPlanner:
        robot = robot or self.team[0]
        key = ("transit", robot.body_fp)
        cache = self.__dict__.setdefault("_cache", {})
        if key not in cache:
            cache[key] = TransitPlanner(self.cspace(robot), simplify=0.02 * robot.body_fp.inradius)
        return cache[key]

    def layers(self) -> dict:
        """Intermediate structures for the simulator (JSON-serialisable)."""
        return {}

    def __getstate__(self):
        d = dict(self.__dict__)
        d.pop("_cache", None)
        return d


def reachable_cspace(free: BaseGeometry, body, starts) -> BaseGeometry:
    from ..instance.coverable import reachable

    cs = body.erode(free)
    reach, _ = reachable(cs, starts, max(0.5 * body.radius, 1e-6))
    return reach if not reach.is_empty else cs


class Adapter:
    name = "abstract"
    family = "abstract"
    version = "1.0"
    defaults: dict = {}

    def __init__(self, **params):
        unknown = set(params) - set(self.defaults)
        if unknown:
            raise ValueError(f"{self.name}: unknown adapter parameters {sorted(unknown)}")
        self.params = {**self.defaults, **params}

    def convert(self, inst) -> Representation:
        raise NotImplementedError

    def resolved_params(self, inst) -> dict:
        return dict(self.params)

    def __call__(self, inst, eps_max: float = DEFAULT_EPS_MAX) -> tuple[Representation, AdapterRecord]:
        t0 = time.perf_counter()
        rep = self.convert(inst)
        runtime = time.perf_counter() - t0
        rec = AdapterRecord(adapter=self.name, family=self.family, version=self.version,
                            params=self.resolved_params(inst), runtime=runtime,
                            info=getattr(rep, "info", {}))
        rec.fidelity = fidelity(inst, rep.reconstruction, eps_max)
        return rep, rec


def fidelity(inst, recon: Reconstruction | None, eps_max: float = DEFAULT_EPS_MAX) -> dict:
    """Reconstruction error eps_i = mu(\\hat F_i xor F_c) at the evaluation resolution."""
    res = inst.eval_resolution()
    if recon is None:
        return {"abs_error": None, "rel_error": None, "eps_max": eps_max, "flagged": False, "resolution": res}
    from ..evaluator.evaluate import eval_grid

    grid: RasterGrid = eval_grid(inst, res)
    fc = grid.rasterize(inst.free)
    hat = recon.mask(grid)
    A = grid.cell_area
    err = float(np.count_nonzero(hat ^ fc)) * A
    missing = float(np.count_nonzero(fc & ~hat)) * A
    extra = float(np.count_nonzero(hat & ~fc)) * A
    mu = max(float(np.count_nonzero(fc)) * A, 1e-12)
    rel = err / mu
    return {"abs_error": err, "rel_error": rel, "missing_area": missing, "extra_area": extra, "eps_max": eps_max,
            "flagged": rel > eps_max, "resolution": res}

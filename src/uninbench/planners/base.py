"""Planner interface (R4, R6): manifests, black-box planners and staged planners."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from .. import TASK_ONE_SHOT
from ..trajectory import TrajectorySet

CONSTRAINTS = ("multi_robot", "heterogeneous", "capacity", "turning_radius", "deconfliction", "return_to_depot",
               "terrain")
PROFILE_CONSTRAINTS = {
    "core": set(),
    "agricultural": {"capacity", "turning_radius"},
    "deconfliction": {"deconfliction"},
    "heterogeneous": {"heterogeneous"},
}
STAGE_KINDS = ("environment_model", "work_distribution", "route_construction", "optimization")
PROVENANCES = ("original", "composition", "baseline")


# ------------------------------------------------------------------ manifest
@dataclass
class HyperParam:
    name: str
    kind: str  # 'float' | 'int' | 'categorical' | 'bool'
    default: Any
    low: float | None = None
    high: float | None = None
    choices: list | None = None
    log: bool = False
    doc: str = ""

    def to_dict(self) -> dict:
        return {k: v for k, v in self.__dict__.items() if v is not None and v != ""}


@dataclass
class Manifest:
    name: str
    version: str
    reference: str
    family: str
    adapter: str
    adapter_params: dict = field(default_factory=dict)
    task_class: str = TASK_ONE_SHOT
    constraints: set = field(default_factory=set)
    max_robots: int | None = None
    stochastic: bool = False
    hyperparameters: list[HyperParam] = field(default_factory=list)
    mode: str = "blackbox"  # 'blackbox' | 'staged'
    stages: list[str] = field(default_factory=list)
    # 'original': reimplementation of the cited publication; 'composition': stages combined by the benchmark;
    # 'baseline': a reference method written for the benchmark.  Only originals are reported under the
    # name of a published algorithm; the others are marked in every report (R6).
    provenance: str = "original"
    composition_of: list[str] | None = None  # components of a composition (or of a baseline)
    distinguishes_service_transit: bool = True
    tool_active_in_turns: bool = True
    parallelism: int = 1
    external_dependencies: list[str] = field(default_factory=list)
    core_modules: list[str] = field(default_factory=list)
    description: str = ""

    def defaults(self) -> dict:
        return {h.name: h.default for h in self.hyperparameters}

    def __post_init__(self):
        if self.provenance not in PROVENANCES:
            raise ValueError(f"{self.name}: unknown provenance {self.provenance!r}")
        if self.composition_of and self.provenance == "original":
            self.provenance = "composition"

    @property
    def is_composition(self) -> bool:
        return self.provenance != "original"

    def to_dict(self) -> dict:
        d = dict(self.__dict__)
        d["constraints"] = sorted(self.constraints)
        d["hyperparameters"] = [h.to_dict() for h in self.hyperparameters]
        return d


def required_constraints(inst) -> set:
    req = set()
    if inst.m > 1:
        req.add("multi_robot")
    t0 = inst.team[0]
    if any((r.tool, r.body, r.speed, r.turn_cost, r.capacity) != (t0.tool, t0.body, t0.speed, t0.turn_cost,
                                                                   t0.capacity) for r in inst.team):
        req.add("heterogeneous")
    if inst.task.return_to_depot:
        req.add("return_to_depot")
    if any(r.capacity for r in inst.team):
        req.add("capacity")
    if any(r.min_turn_radius for r in inst.team):
        req.add("turning_radius")
    for p in inst.task.profiles:
        req |= PROFILE_CONSTRAINTS.get(p, set())
    if inst.m < 2:
        req.discard("deconfliction")  # a single robot cannot conflict with another
    return req


def admission(manifest: Manifest, inst, deconfliction_stage: str | None = None) -> tuple[str, str | None]:
    """'admitted', 'excluded' (task class mismatch, not compared) or 'not_applicable'.

    With a framework ``deconfliction_stage`` a planner without its own
    deconfliction is admitted to the deconfliction profile as a composition
    (see :func:`uninbench.planners.execution.framework_stages`).
    """
    if manifest.task_class != inst.task.task_class:
        return "excluded", f"declared task class {manifest.task_class!r} != {inst.task.task_class!r}"
    missing = required_constraints(inst) - set(manifest.constraints)
    if deconfliction_stage and inst.m > 1:
        missing.discard("deconfliction")
    if manifest.max_robots is not None and inst.m > manifest.max_robots:
        missing.add(f"team_size>{manifest.max_robots}")
    if missing:
        return "not_applicable", "unsupported: " + ", ".join(sorted(missing))
    return "admitted", None


# ------------------------------------------------------------------ run context
@dataclass
class PlanContext:
    seed: int = 0
    time_limit: float = math.inf
    profiles: set = field(default_factory=lambda: {"core"})
    return_to_depot: bool = False
    start_time: float = field(default_factory=time.perf_counter)

    def __post_init__(self):
        self.rng = np.random.default_rng(self.seed)

    def time_left(self) -> float:
        return self.time_limit - (time.perf_counter() - self.start_time)

    def out_of_time(self, fraction: float = 0.9) -> bool:
        return (time.perf_counter() - self.start_time) > fraction * self.time_limit


@dataclass
class PlanResult:
    trajectories: TrajectorySet
    native: Any = None
    layers: dict = field(default_factory=dict)
    stage_outputs: dict = field(default_factory=dict)
    stage_metrics: dict = field(default_factory=dict)
    info: dict = field(default_factory=dict)


class Planner:
    """Black-box planner: receives R_i, the team (inside R_i), theta_A, a seed and limits."""

    manifest: Manifest

    def __init__(self, params: dict | None = None):
        defaults = self.manifest.defaults()
        params = dict(params or {})
        unknown = set(params) - set(defaults)
        if unknown:
            raise ValueError(f"{self.manifest.name}: unknown hyperparameters {sorted(unknown)}")
        self.params = {**defaults, **params}

    def plan(self, rep, ctx: PlanContext) -> PlanResult:
        raise NotImplementedError


# ------------------------------------------------------------------ staged mode
class Stage:
    """One exchangeable pipeline stage with a typed interface."""

    kind = "abstract"
    name = "abstract"
    consumes: tuple = ()
    produces: str = ""
    hyperparameters: list[HyperParam] = []

    def run(self, data: dict, rep, ctx: PlanContext, params: dict):
        raise NotImplementedError

    def metrics(self, output, rep) -> dict:
        """Separate evaluation of the intermediate output (e.g. partition balance)."""
        return {}

    def summary(self, output) -> Any:
        return None


class StagedPlanner(Planner):
    """Composition of environment model, work distribution, route construction and
    optimization stages.  Stages 2 and 3 may appear in either order."""

    stages: list[Stage] = []

    @classmethod
    def check_composition(cls):
        have = {"representation"}
        for st in cls.stages:
            missing = set(st.consumes) - have
            if missing:
                raise TypeError(f"{cls.__name__}: stage {st.name} consumes {sorted(missing)} not produced earlier")
            if st.kind not in STAGE_KINDS:
                raise TypeError(f"stage {st.name}: unknown kind {st.kind}")
            have.add(st.produces)
        if "routes" not in have:
            raise TypeError(f"{cls.__name__}: no stage produces 'routes'")

    def plan(self, rep, ctx: PlanContext) -> PlanResult:
        self.check_composition()
        data: dict[str, Any] = {"representation": rep}
        outputs, metrics, timings = {}, {}, {}
        for st in self.stages:
            t0 = time.perf_counter()
            out = st.run(data, rep, ctx, self.params)
            timings[st.name] = time.perf_counter() - t0
            data[st.produces] = out
            m = st.metrics(out, rep)
            if m:
                metrics[f"{st.kind}:{st.name}"] = m
            s = st.summary(out)
            if s is not None:
                outputs[f"{st.kind}:{st.name}"] = s
        res = self.finish(data, rep, ctx)
        res.stage_outputs.update(outputs)
        res.stage_metrics.update(metrics)
        res.info["stage_times"] = timings
        return res

    def finish(self, data: dict, rep, ctx: PlanContext) -> PlanResult:
        raise NotImplementedError

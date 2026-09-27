"""Planner registry.

Built-in planners are registered here; external planners can be registered
with :func:`register`, via the ``uninbench.planners`` entry-point group, or by
naming ``"package.module:ClassName"`` in an experiment configuration.
"""

from __future__ import annotations

import importlib

from .base import (CONSTRAINTS, PROFILE_CONSTRAINTS, HyperParam, Manifest, PlanContext, Planner, PlanResult, Stage,
                   StagedPlanner, admission, required_constraints)
from .darp import DARPBF, DARPSTC
from .mstc import MSTC
from .stc import STC
from .sweep import BCD, SwathBF
from .targets import KMeansTSP

PLANNERS: dict[str, type[Planner]] = {}


def register(cls: type[Planner]) -> type[Planner]:
    PLANNERS[cls.manifest.name] = cls
    return cls


for _cls in (STC, MSTC, DARPSTC, DARPBF, SwathBF, BCD, KMeansTSP):
    register(_cls)


def _load_entry_points():
    try:
        from importlib.metadata import entry_points

        for ep in entry_points(group="uninbench.planners"):
            register(ep.load())
    except Exception:  # pragma: no cover - optional plugins
        pass


_load_entry_points()


def get_planner(name: str) -> type[Planner]:
    if name in PLANNERS:
        return PLANNERS[name]
    if ":" in name:
        mod, attr = name.split(":", 1)
        cls = getattr(importlib.import_module(mod), attr)
        return register(cls)
    raise KeyError(f"unknown planner {name!r}; available: {sorted(PLANNERS)}")


__all__ = [
    "CONSTRAINTS", "PROFILE_CONSTRAINTS", "HyperParam", "Manifest", "PlanContext", "Planner", "PlanResult", "Stage",
    "StagedPlanner", "admission", "required_constraints", "PLANNERS", "register", "get_planner",
]

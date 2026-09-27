"""Experiment configuration (YAML or JSON)."""

from __future__ import annotations

import copy
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

DEFAULTS: dict[str, Any] = {
    "name": "experiment",
    "output": None,
    "instances": [],
    "split": "test",
    "team_sizes": [1],
    "start_configs": ["clustered"],
    "planners": [],
    "adapters": {},
    "factors": {"planning_resolution": [1.0]},
    "profiles": ["core"],
    "task": {},
    "evaluation": {},
    "seeds": 20,
    "repetitions": 5,
    "eps_max": 0.05,
    "workers": 1,
    "inline": False,
    "store": {"trajectories": True, "coverage": True, "events": True},
    "tuning_dir": None,
    "validation_dir": None,
    "require_validation": False,
    "grace": 5.0,
}


@dataclass
class PlannerSpec:
    name: str
    configs: list[str] = field(default_factory=lambda: ["default"])
    params: dict = field(default_factory=dict)
    label: str | None = None

    @property
    def display(self) -> str:
        return self.label or self.name


@dataclass
class ExperimentConfig:
    raw: dict
    path: Path | None = None

    def __getattr__(self, item):
        try:
            return self.raw[item]
        except KeyError as e:
            raise AttributeError(item) from e

    @property
    def output_dir(self) -> Path:
        out = self.raw.get("output") or f"results/{self.raw['name']}"
        p = Path(out)
        if not p.is_absolute() and self.path is not None and self.raw.get("relative_to_config", False):
            p = self.path.parent / p
        return p

    @property
    def planner_specs(self) -> list[PlannerSpec]:
        out = []
        for p in self.raw["planners"]:
            if isinstance(p, str):
                out.append(PlannerSpec(p))
            else:
                cfg = p.get("configs", ["default"])
                out.append(PlannerSpec(p["name"], [cfg] if isinstance(cfg, str) else list(cfg), p.get("params", {}),
                                       p.get("label")))
        return out

    def to_dict(self) -> dict:
        return copy.deepcopy(self.raw)


def load_config(path_or_dict) -> ExperimentConfig:
    if isinstance(path_or_dict, (str, Path)):
        path = Path(path_or_dict)
        raw = yaml.safe_load(path.read_text())
    else:
        path, raw = None, copy.deepcopy(path_or_dict)
    cfg = copy.deepcopy(DEFAULTS)
    for k, v in (raw or {}).items():
        if isinstance(v, dict) and isinstance(cfg.get(k), dict):
            cfg[k] = {**cfg[k], **v}
        else:
            cfg[k] = v
    if not cfg["planners"]:
        raise ValueError("experiment configuration lists no planners")
    if not cfg["instances"]:
        raise ValueError("experiment configuration lists no instances")
    return ExperimentConfig(cfg, path)

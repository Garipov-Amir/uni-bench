"""Actions available in the user interfaces.

Every long-running action is translated into a headless ``uninbench`` command and
started as a background job; quick queries (planner manifests, configuration
checks, run estimates) are answered directly.
"""

from __future__ import annotations

import itertools
from pathlib import Path

import yaml

from ..planners import PLANNERS, get_planner
from ..runner.config import load_config
from .jobs import JobManager
from .workspace import Workspace, WorkspaceError

DATASETS = {
    "areacoverage": "AreaCoverage (VM25 / AC300 / H2) directory",
    "fields": "directory of field files (GML/XML, GeoJSON, WKT)",
    "rooms": "directory of occupancy maps with *_furnitures.png variants",
    "safelog": "directory of SafeLog edge lists (*.txt)",
}


def _flag_list(name: str, values) -> list[str]:
    values = [str(v) for v in (values or []) if str(v).strip() != ""]
    return [name, *values] if values else []


class Actions:
    def __init__(self, ws: Workspace, jobs: JobManager | None = None):
        self.ws = ws
        self.jobs = jobs or JobManager(ws)

    # ------------------------------------------------------------------ queries
    def planners(self) -> list[dict]:
        status = self.ws.validation_status()
        out = []
        for name, cls in sorted(PLANNERS.items()):
            m = cls.manifest.to_dict()
            m["validation"] = status.get(name)
            out.append(m)
        return out

    def check_experiment(self, text: str) -> dict:
        """Parse and check a configuration; estimate the number of runs without executing."""
        try:
            raw = yaml.safe_load(text) or {}
        except yaml.YAMLError as e:
            mark = getattr(e, "problem_mark", None)
            return {"ok": False, "errors": [f"YAML error{f' at line {mark.line + 1}' if mark else ''}: {e}"]}
        errors, warnings = [], []
        try:
            cfg = load_config(raw)
        except (ValueError, TypeError) as e:
            return {"ok": False, "errors": [str(e)]}
        planners = []
        for spec in cfg.planner_specs:
            try:
                planners.append((spec, get_planner(spec.name)))
            except (KeyError, ImportError, AttributeError) as e:
                errors.append(str(e))
        n_inst = 0
        for spec in cfg.instances:
            spec = {"path": spec} if isinstance(spec, str) else spec
            try:
                n_inst += self._count_instances(spec, cfg.split)
            except (WorkspaceError, OSError, ValueError) as e:
                errors.append(f"instances {spec}: {e}")
        if n_inst == 0 and not errors:
            warnings.append("no instances selected (check the paths and the split)")
        m_list = [int(m) for m in cfg.team_sizes]
        starts = list(cfg.start_configs)
        res = cfg.factors.get("planning_resolution", [1.0]) or [1.0]
        runs, na = 0, 0
        for spec, cls in planners:
            man = cls.manifest
            seeds = int(cfg.seeds) if man.stochastic else 1
            facs = len(res) if man.adapter == "grid" else 1
            for m in m_list:
                n = n_inst * len(starts) * len(spec.configs) * facs
                if man.max_robots is not None and m > man.max_robots:
                    na += n
                else:
                    runs += n * seeds
            if "tuned" in spec.configs and cfg.tuning_dir:
                tdir = self.ws.resolve(cfg.tuning_dir)
                if not list(tdir.glob(f"{man.name}__*.json")):
                    warnings.append(f"{spec.display}: no frozen tuned configuration yet (run tuning first)")
        out = cfg.raw.get("output") or f"results/{cfg.name}"
        if (self.ws.resolve(out) / "runs.jsonl").exists():
            warnings.append(f"{out} already has results: finished runs are skipped (resume)")
        return {"ok": not errors, "errors": errors, "warnings": warnings, "name": cfg.name, "output": out,
                "instances": n_inst, "team_sizes": m_list, "start_configs": starts,
                "planners": [s.display for s, _ in planners], "estimated_runs": runs,
                "not_applicable": na}

    def _count_instances(self, spec: dict, split: str) -> int:
        if "generate" in spec:
            g = dict(spec["generate"])
            kind = g.pop("kind", "synthetic")
            if kind == "safelog":
                dens = g.get("densities") or [0] * 21
                return len(dens) * int(g.get("count", 1))
            combos = itertools.product(g.get("sizes") or [0], g.get("densities") or [0], g.get("shapes") or [0],
                                       g.get("nonconvexities") or [0])
            n = sum(1 for _ in combos) * int(g.get("count", 5))
            want = spec.get("split", split)
            if want and want != "all":
                frac = float(spec.get("tuning_fraction", 0.2))
                n = round(n * (frac if want == "tuning" else 1 - frac))
            return n
        path = self.ws.resolve(spec["path"])
        want = spec.get("split", split)
        if not path.exists():
            raise ValueError(f"not found: {spec['path']}")
        if path.is_file():
            return 1
        files = [f for f in path.glob("*.json") if f.name != "index.json"]
        idx = path / "index.json"
        if want and want != "all" and idx.exists():
            import json

            rows = json.loads(idx.read_text()).get("instances", [])
            files = [r for r in rows if r.get("split", "test") == want]
        n = len(files)
        return min(n, int(spec["limit"])) if spec.get("limit") else n

    # ------------------------------------------------------------------ jobs
    def run_experiment(self, config: str, workers: int | None = None, resume: bool = True) -> dict:
        self.ws.resolve(config, must_exist=True)
        argv = ["run", config] + (["--workers", str(workers)] if workers else []) + ([] if resume else
                                                                                      ["--no-resume"])
        return self.jobs.start("run", argv, f"run {Path(config).stem}")

    def tune(self, config: str, budget: int = 20, planners=None, tuning_seeds: int = 3,
             workers: int | None = None) -> dict:
        self.ws.resolve(config, must_exist=True)
        argv = ["tune", config, "--budget", str(budget), "--tuning-seeds", str(tuning_seeds)]
        for p in planners or []:
            argv += ["--planner", p]
        if workers:
            argv += ["--workers", str(workers)]
        return self.jobs.start("tune", argv, f"tune {Path(config).stem}")

    def validate(self, planners=None) -> dict:
        argv = ["validate", "--out", self.ws.rel(self.ws.validation_dir)]
        for p in planners or []:
            argv += ["--planner", p]
        return self.jobs.start("validate", argv, "validate " + (", ".join(planners) if planners else "all planners"))

    def report(self, results: str) -> dict:
        self.ws.resolve(results, must_exist=True)
        return self.jobs.start("report", ["report", results], f"report {Path(results).name}")

    def replay(self, results: str, instance: str | None = None, planners=None) -> dict:
        self.ws.resolve(results, must_exist=True)
        argv = ["replay", results] + (["--instance", instance] if instance else []) + _flag_list("--planners",
                                                                                               planners)
        return self.jobs.start("replay", argv, f"replay {instance or Path(results).name}")

    def generate(self, kind: str, out: str, count: int = 5, seed: int = 0, sizes=None, densities=None, shapes=None,
                 nonconvexity=None, rows: int = 20, cols: int = 20, tuning_fraction: float = 0.2,
                 stratify: str | None = None) -> dict:
        self.ws.resolve(out)
        argv = ["generate", kind, "--out", out, "--count", str(count), "--seed", str(seed),
                "--tuning-fraction", str(tuning_fraction)]
        if kind == "synthetic":
            argv += _flag_list("--sizes", sizes) + _flag_list("--densities", densities) + \
                _flag_list("--shapes", shapes) + _flag_list("--nonconvexity", nonconvexity)
        else:
            argv += ["--rows", str(rows), "--cols", str(cols)] + _flag_list("--densities", densities)
        if stratify:
            argv += ["--stratify", stratify]
        return self.jobs.start("generate", argv, f"generate {kind} → {out}")

    def import_dataset(self, dataset: str, src: str, out: str, params: dict | None = None,
                       tuning_fraction: float = 0.2, stratify: str | None = None) -> dict:
        if dataset not in DATASETS:
            raise ValueError(f"unknown dataset {dataset!r}")
        if not Path(src).expanduser().exists():
            raise ValueError(f"source not found: {src}")
        self.ws.resolve(out)
        argv = ["import", dataset, str(Path(src).expanduser()), "--out", out, "--tuning-fraction",
                str(tuning_fraction)]
        for k, v in (params or {}).items():
            argv += ["--param", f"{k}={v}"]
        if stratify:
            argv += ["--stratify", stratify]
        return self.jobs.start("import", argv, f"import {dataset} → {out}")

    def split(self, set_path: str, tuning_fraction: float = 0.2, stratify: str | None = None) -> dict:
        self.ws.resolve(set_path, must_exist=True)
        argv = ["split", set_path, "--tuning-fraction", str(tuning_fraction)] + (["--stratify", stratify]
                                                                                if stratify else [])
        return self.jobs.start("split", argv, f"split {set_path}")

    def check_evaluator(self, src: str) -> dict:
        argv = ["check-evaluator", str(Path(src).expanduser()), "--out",
                self.ws.rel(self.ws.validation_dir / "evaluator_reference_routes.json")]
        return self.jobs.start("check-evaluator", argv, "check evaluator on reference routes")

    def demo(self, out: str = "demo") -> dict:
        return self.jobs.start("demo", ["demo", "--out", out], "demo pipeline")

"""The workspace: one directory that holds instance sets, experiment configurations,
results, validation reports and background jobs.  Both user interfaces (TUI and web)
operate on a workspace, so they always show the same state.

Layout::

    <root>/
      data/<set>/*.json         instance sets (one canonical instance per file, index.json)
      experiments/*.yaml        experiment configurations
      results/<experiment>/     experiment outputs (runs.jsonl, report/, replay/, ...)
      results/tuning/           frozen tuned configurations
      validation/               planner validation reports
      datasets/                 raw downloaded datasets
      .uninbench/jobs/<id>/     background jobs (job.json, status.json, output.log)
"""

from __future__ import annotations

import json
import os
import time
from collections import Counter
from pathlib import Path

import yaml

from ..util import load_json

EXAMPLE_EXPERIMENT = """\
# A small experiment on generated instances.  Edit freely; see docs/experiments.md.
name: {name}
output: results/{name}
tuning_dir: results/tuning
instances:
  - generate: {{kind: synthetic, count: 3, sizes: [20, 30], densities: [0.05, 0.1], base_seed: 1}}
    family: IF5
    split: all
team_sizes: [1, 2, 4]
start_configs: [dispersed]
planners: [stc, mstc, darp_stc, darp_bf, swath_bf, bcd, kmeans_tsp]
seeds: 3
repetitions: 3
workers: {workers}
task: {{time_limit: 60, tolerance: 0.02}}
evaluation: {{resolution_factor: 0.1, sensitivity: true}}
"""


class WorkspaceError(ValueError):
    pass


class Workspace:
    def __init__(self, root: str | Path = "."):
        self.root = Path(root).expanduser().resolve()
        self._cache: dict = {}

    # ------------------------------------------------------------------ layout
    @property
    def data(self) -> Path:
        return self.root / "data"

    @property
    def experiments_dir(self) -> Path:
        return self.root / "experiments"

    @property
    def results_dir(self) -> Path:
        return self.root / "results"

    @property
    def validation_dir(self) -> Path:
        return self.root / "validation"

    @property
    def datasets_dir(self) -> Path:
        return self.root / "datasets"

    @property
    def jobs_dir(self) -> Path:
        return self.root / ".uninbench" / "jobs"

    def init(self, example: bool = True) -> list[str]:
        """Create the workspace layout (idempotent) and an example experiment."""
        made = []
        for d in (self.data, self.experiments_dir, self.results_dir, self.validation_dir, self.datasets_dir,
                  self.jobs_dir):
            if not d.exists():
                d.mkdir(parents=True)
                made.append(self.rel(d))
        ex = self.experiments_dir / "quick.yaml"
        if example and not ex.exists():
            workers = max(1, min(4, (os.cpu_count() or 2) - 1))
            ex.write_text(EXAMPLE_EXPERIMENT.format(name="quick", workers=workers), encoding="utf8")
            made.append(self.rel(ex))
        return made

    @property
    def initialized(self) -> bool:
        return self.experiments_dir.is_dir() or self.data.is_dir()

    def rel(self, p: str | Path) -> str:
        p = Path(p)
        try:
            return str(p.resolve().relative_to(self.root))
        except ValueError:
            return str(p)

    def resolve(self, rel: str | Path, must_exist: bool = False) -> Path:
        """Absolute path of a workspace-relative path; refuses paths outside the workspace."""
        p = (self.root / rel).resolve() if not Path(rel).is_absolute() else Path(rel).resolve()
        if p != self.root and self.root not in p.parents:
            raise WorkspaceError(f"path outside the workspace: {rel}")
        if must_exist and not p.exists():
            raise WorkspaceError(f"no such file: {rel}")
        return p

    # ------------------------------------------------------------------ summary
    def summary(self) -> dict:
        sets = self.instance_sets()
        return {
            "root": str(self.root),
            "initialized": self.initialized,
            "instance_sets": len(sets),
            "instances": sum(s["count"] for s in sets),
            "experiments": len(self.experiments()),
            "results": len(self.results()),
        }

    # ------------------------------------------------------------------ instances
    def instance_sets(self) -> list[dict]:
        out = []
        if not self.data.is_dir():
            return out
        for d in sorted(p for p in self.data.iterdir() if p.is_dir()):
            files = [f for f in d.glob("*.json") if f.name != "index.json"]
            if not files:
                continue
            idx = d / "index.json"
            families, splits = Counter(), Counter()
            if idx.exists():
                try:
                    for row in load_json(idx).get("instances", []):
                        families[row.get("family", "?")] += 1
                        splits[row.get("split") or "none"] += 1
                except (OSError, ValueError):
                    pass
            out.append({"name": d.name, "path": self.rel(d), "count": len(files), "families": dict(families),
                        "splits": dict(splits), "indexed": idx.exists(),
                        "modified": max(f.stat().st_mtime for f in files)})
        return out

    def instances(self, set_path: str) -> list[dict]:
        """Light-weight rows for every instance of a set (cached by modification time)."""
        d = self.resolve(set_path, must_exist=True)
        files = sorted(f for f in d.glob("*.json") if f.name != "index.json")
        key = ("instances", str(d))
        stamp = tuple((f.name, f.stat().st_mtime) for f in files)
        hit = self._cache.get(key)
        if hit and hit[0] == stamp:
            return hit[1]
        rows = []
        for f in files:
            try:
                raw = json.loads(f.read_text(encoding="utf8"))
            except (OSError, ValueError):
                continue
            env = raw.get("environment", {})
            meta = raw.get("metadata", {})
            rows.append({
                "id": raw.get("id", f.stem), "file": self.rel(f), "family": raw.get("family"),
                "split": meta.get("split"), "holes": len(env.get("holes", [])), "robots": len(raw.get("team", [])),
                "width": (raw.get("team") or [{}])[0].get("tool", {}).get("width"),
                "factors": meta.get("factors", {}), "source": meta.get("source"), "hash": raw.get("hash"),
            })
        self._cache[key] = (stamp, rows)
        return rows

    def instance_detail(self, file: str, simplify: float | None = None) -> dict:
        from ..geometry.decomposition import polygons_of
        from ..instance.model import Instance

        inst = Instance.load(self.resolve(file, must_exist=True))
        d = inst.describe()
        tol = simplify if simplify is not None else 0.002 * max(d["extent"])

        def coords(g):
            out = []
            for p in polygons_of(g.simplify(tol) if tol else g):
                out.append([[[round(x, 3), round(y, 3)] for x, y in ring.coords]
                            for ring in [p.exterior, *p.interiors]])
            return out

        d["bounds"] = list(inst.environment.bounds)
        d["free_geometry"] = coords(inst.free)
        d["coverable_geometry"] = coords(inst.derived.coverable)
        d["team"] = [r.to_dict() for r in inst.team]
        d["task"] = inst.task.to_dict()
        d["metadata"] = {k: v for k, v in inst.metadata.items() if k not in ("graph",)}
        d["file"] = self.rel(self.resolve(file))
        return d

    # ------------------------------------------------------------------ experiments
    def experiments(self) -> list[dict]:
        out = []
        if not self.experiments_dir.is_dir():
            return out
        for f in sorted(self.experiments_dir.glob("*.y*ml")):
            row = {"name": f.stem, "path": self.rel(f), "modified": f.stat().st_mtime, "error": None}
            try:
                cfg = yaml.safe_load(f.read_text(encoding="utf8")) or {}
                row["name"] = cfg.get("name", f.stem)
                row["planners"] = [p if isinstance(p, str) else p.get("name") for p in cfg.get("planners", [])]
                row["team_sizes"] = cfg.get("team_sizes", [1])
                row["output"] = cfg.get("output") or f"results/{row['name']}"
                row["has_results"] = (self.resolve(row["output"]) / "runs.jsonl").exists()
            except (yaml.YAMLError, AttributeError, WorkspaceError) as e:
                row["error"] = str(e)
            out.append(row)
        return out

    def read_text(self, rel: str) -> str:
        return self.resolve(rel, must_exist=True).read_text(encoding="utf8")

    def write_experiment(self, name: str, text: str) -> str:
        safe = "".join(c for c in name if c.isalnum() or c in "-_") or "experiment"
        path = self.experiments_dir / f"{safe}.yaml"
        self.experiments_dir.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding="utf8")
        return self.rel(path)

    def new_experiment_text(self, name: str, **fields) -> str:
        workers = max(1, min(4, (os.cpu_count() or 2) - 1))
        text = EXAMPLE_EXPERIMENT.format(name=name, workers=workers)
        if not fields:
            return text
        cfg = yaml.safe_load(text)
        cfg.update({k: v for k, v in fields.items() if v not in (None, "", [])})
        return yaml.safe_dump(cfg, sort_keys=False)

    # ------------------------------------------------------------------ results
    def results(self) -> list[dict]:
        out = []
        if not self.results_dir.is_dir():
            return out
        for d in sorted(p for p in self.results_dir.iterdir() if p.is_dir()):
            runs = d / "runs.jsonl"
            if not runs.exists():
                continue
            summary = {}
            if (d / "summary.json").exists():
                try:
                    summary = load_json(d / "summary.json")
                except ValueError:
                    summary = {}
            out.append({"name": d.name, "path": self.rel(d), "runs": _count_lines(runs),
                        "outcomes": summary.get("outcomes", {}), "report": (d / "report" / "index.html").exists(),
                        "replays": sorted(self.rel(p) for p in (d / "replay").glob("*.html"))
                        if (d / "replay").is_dir() else [],
                        "modified": runs.stat().st_mtime})
        return out

    def results_summary(self, rel: str) -> dict:
        from ..reporting import aggregate as A
        from ..runner.experiment import load_runs

        d = self.resolve(rel, must_exist=True)
        recs = load_runs(d)
        outcomes = Counter(r.get("outcome", "?") for r in recs)
        succ = A.success_table(recs)
        desc = [r for r in A.descriptive_table(recs, [A.PRIMARY, "coverage_ratio", "planning_time"])]
        instances = sorted({r.get("instance_id") for r in recs if (r.get("artifacts") or {}).get("trajectories")})
        exp = load_json(d / "experiment.json") if (d / "experiment.json").exists() else {}
        return {"path": self.rel(d), "name": exp.get("config", {}).get("name", d.name), "runs": len(recs),
                "outcomes": dict(outcomes), "success": succ, "descriptives": desc, "replay_instances": instances,
                "report": self.rel(d / "report" / "index.html") if (d / "report" / "index.html").exists() else None,
                "replays": sorted(self.rel(p) for p in (d / "replay").glob("*.html")) if (d / "replay").is_dir()
                else [], "started": exp.get("started")}

    def runs(self, rel: str, planner: str | None = None, outcome: str | None = None, limit: int = 500) -> list[dict]:
        from ..reporting.aggregate import flatten
        from ..runner.experiment import load_runs

        recs = load_runs(self.resolve(rel, must_exist=True))
        rows = [flatten(r) for r in recs
                if (not planner or r.get("planner") == planner) and (not outcome or r.get("outcome") == outcome)]
        return rows[:limit]

    # ------------------------------------------------------------------ validation
    def validation_status(self) -> dict:
        out = {}
        if self.validation_dir.is_dir():
            for f in self.validation_dir.glob("*.json"):
                try:
                    rep = load_json(f)
                except ValueError:
                    continue
                if "planner" in rep:
                    out[rep["planner"]] = {"passed": rep.get("passed"), "summary": rep.get("summary"),
                                           "validated_at": rep.get("validated_at")}
        return out


def _count_lines(path: Path) -> int:
    n = 0
    with open(path, "rb") as fh:
        for _ in fh:
            n += 1
    return n


def now() -> float:
    return time.time()

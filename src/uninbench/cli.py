"""Command-line interface: ``uninbench <command> ...``."""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

import yaml


def _kv(items) -> dict:
    out = {}
    for it in items or []:
        k, _, v = it.partition("=")
        out[k] = yaml.safe_load(v)
    return out


def _split(out_dir, fraction, stratify):
    from .instance.splits import split_directory

    s = split_directory(out_dir, fraction, stratify)
    n_t = sum(v == "tuning" for v in s.values())
    print(f"{len(s)} instances in {out_dir} ({n_t} tuning / {len(s) - n_t} test)")


# ------------------------------------------------------------------ commands
def cmd_generate(a):
    from .datasets.safelog import synthetic_safelog
    from .datasets.synthetic import generate_family

    Path(a.out).mkdir(parents=True, exist_ok=True)
    if a.kind == "synthetic":
        insts = generate_family(a.out, count=a.count, base_seed=a.seed, sizes=a.sizes, densities=a.densities,
                                shapes=a.shapes, nonconvexities=a.nonconvexity, aspect=a.aspect, **_kv(a.param))
    else:
        insts = synthetic_safelog(a.out, rows=a.rows, cols=a.cols, densities=a.densities, count=a.count,
                                  seed=a.seed, cell_size=a.cell_size)
    print(f"generated {len(insts)} instances")
    _split(a.out, a.tuning_fraction, a.stratify)


def cmd_import(a):
    from .datasets import import_areacoverage, import_fields, import_rooms, import_safelog

    Path(a.out).mkdir(parents=True, exist_ok=True)
    p = _kv(a.param)
    fn = {"areacoverage": import_areacoverage, "fields": import_fields, "rooms": import_rooms,
          "safelog": import_safelog}[a.dataset]
    insts = fn(a.src, a.out, **p)
    print(f"imported {len(insts)} instances into {a.out}")
    _split(a.out, a.tuning_fraction, a.stratify)


def cmd_split(a):
    _split(a.dir, a.tuning_fraction, a.stratify)


def cmd_info(a):
    from .instance.model import Instance

    for p in a.instances:
        print(json.dumps(Instance.load(p).describe(), indent=1, default=str))


def cmd_planners(a):
    from .planners import PLANNERS

    if a.json:
        print(json.dumps({n: c.manifest.to_dict() for n, c in PLANNERS.items()}, indent=1, default=str))
        return
    for n, c in sorted(PLANNERS.items()):
        m = c.manifest
        comp = f" composition of {'+'.join(m.composition_of)}" if m.composition_of else ""
        print(f"{n:12s} {m.family:10s} {m.mode:9s} {'stochastic' if m.stochastic else 'deterministic':13s} "
              f"constraints={sorted(m.constraints)}{comp}\n{'':12s} {m.reference}")


def cmd_adapt(a):
    from .adapters import make_adapter
    from .instance.model import Instance
    from .instance.team import with_team

    inst = Instance.load(a.instance)
    if a.team_size:
        inst = with_team(inst, a.team_size, a.start_config)
    rep, rec = make_adapter(a.adapter, **_kv(a.param))(inst, eps_max=a.eps_max)
    print(json.dumps(rec.to_dict(), indent=1, default=str))


def cmd_validate(a):
    from .validation import validate_all

    res = validate_all(a.out, a.planner or None)
    ok = True
    for n, r in res.items():
        print(f"{n:12s} {'PASS' if r['passed'] else 'FAIL'}  {r['summary']}")
        for c in r["cases"]:
            if not c["passed"]:
                ok = False
                bad = [x for x in c["results"] if not x["ok"]]
                print(f"    {c['case']}: {bad} {c['errors']}")
    return 0 if ok else 1


def cmd_check_evaluator(a):
    from .validation.reference import check_reference_routes

    rep = check_reference_routes(a.src, a.subsets, a.limit, a.flag_convention, a.out)
    print(rep["summary"])
    return 0 if rep["passed"] else 1


def cmd_run(a):
    from .runner import run_experiment

    s = run_experiment(a.config, workers=a.workers, inline=a.inline or None, resume=not a.no_resume,
                       output=a.output)
    print(json.dumps(s, indent=1))


def cmd_tune(a):
    from .tuning import tune_experiment

    res = tune_experiment(a.config, budget=a.budget, planners=a.planner or None, workers=a.workers,
                          inline=a.inline or None, seed=a.seed, penalty=a.penalty, tuning_seeds=a.tuning_seeds)
    for (p, fam), r in res.items():
        print(f"{p} / {fam}: objective {r['objective_theta0']} (default) -> {r['objective_star']} (tuned) "
              f"theta* = {r['theta_star']}")


def cmd_evaluate(a):
    from .evaluator import evaluate
    from .instance.model import Instance
    from .trajectory import TrajectorySet
    from .util import load_json, to_jsonable

    inst = Instance.load(a.instance)
    ts = TrajectorySet.from_dict(load_json(a.trajectories))
    ev = evaluate(inst, ts)
    print(json.dumps(to_jsonable(ev.summary()), indent=1))


def cmd_report(a):
    from .reporting import build_report

    print(build_report(a.results, a.out))


def cmd_replay(a):
    from .simulator import build_replay

    print(build_replay(a.results, a.instance, a.planners or None, a.out, a.max_cells))


def cmd_figure(a):
    from .simulator import plot_run

    print(plot_run(a.results, a.run_id, a.out))


def cmd_demo(a):
    """Small end-to-end pipeline: generate -> validate -> run -> report -> replay."""
    from .reporting import build_report
    from .runner import run_experiment
    from .simulator import build_replay

    out = Path(a.out)
    cfg = {
        "name": "demo", "output": str(out / "results"),
        "instances": [{"generate": {"kind": "synthetic", "count": 3, "sizes": [20, 30], "densities": [0.06],
                                    "base_seed": 7}, "family": "IF5", "split": "all"},
                      # grid-native instances (SafeLog-like graphs) on which the grid planners are complete
                      {"generate": {"kind": "safelog", "rows": 10, "cols": 10, "densities": [0.75, 1.0],
                                    "count": 2, "seed": 7}, "family": "IF4", "split": "all"}],
        "team_sizes": [1, 2, 4], "start_configs": ["dispersed"],
        "planners": ["stc", "mstc", "darp_stc", "darp_bf", "swath_bf", "bcd", "kmeans_tsp"],
        "seeds": 3, "repetitions": 2, "workers": a.workers, "task": {"time_limit": 30},
    }
    out.mkdir(parents=True, exist_ok=True)
    (out / "demo.yaml").write_text(yaml.safe_dump(cfg, sort_keys=False))
    s = run_experiment(cfg)
    print(json.dumps(s["outcomes"]))
    print("report:", build_report(out / "results"))
    print("replay:", build_replay(out / "results"))


# ------------------------------------------------------------------ parser
def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(prog="uninbench", description="Unified benchmark for multi-robot coverage path "
                                                                "planning")
    sub = ap.add_subparsers(dest="cmd", required=True)

    g = sub.add_parser("generate", help="generate synthetic instance families (IF5, IF4-like)")
    g.add_argument("kind", choices=["synthetic", "safelog"])
    g.add_argument("--out", required=True)
    g.add_argument("--count", type=int, default=5)
    g.add_argument("--seed", type=int, default=0)
    g.add_argument("--sizes", type=float, nargs="*")
    g.add_argument("--densities", type=float, nargs="*")
    g.add_argument("--shapes", nargs="*")
    g.add_argument("--nonconvexity", type=int, nargs="*")
    g.add_argument("--aspect", type=float, default=0.75)
    g.add_argument("--rows", type=int, default=20)
    g.add_argument("--cols", type=int, default=20)
    g.add_argument("--cell-size", type=float, default=2.0)
    g.add_argument("--param", action="append", help="generator parameter key=value")
    g.add_argument("--tuning-fraction", type=float, default=0.2)
    g.add_argument("--stratify", default=None, help="factor to stratify the split by")
    g.set_defaults(fn=cmd_generate)

    i = sub.add_parser("import", help="import a public dataset into canonical instances")
    i.add_argument("dataset", choices=["areacoverage", "fields", "rooms", "safelog"])
    i.add_argument("src")
    i.add_argument("--out", required=True)
    i.add_argument("--param", action="append", help="importer parameter key=value (e.g. width=3.0)")
    i.add_argument("--tuning-fraction", type=float, default=0.2)
    i.add_argument("--stratify", default=None)
    i.set_defaults(fn=cmd_import)

    s = sub.add_parser("split", help="(re)assign tuning/test splits of a directory")
    s.add_argument("dir")
    s.add_argument("--tuning-fraction", type=float, default=0.2)
    s.add_argument("--stratify", default=None)
    s.set_defaults(fn=cmd_split)

    s = sub.add_parser("info", help="describe instances")
    s.add_argument("instances", nargs="+")
    s.set_defaults(fn=cmd_info)

    s = sub.add_parser("planners", help="list registered planners and their manifests")
    s.add_argument("--json", action="store_true")
    s.set_defaults(fn=cmd_planners)

    s = sub.add_parser("adapt", help="run an adapter on an instance and print its transformation record")
    s.add_argument("instance")
    s.add_argument("--adapter", default="grid")
    s.add_argument("--param", action="append")
    s.add_argument("--team-size", type=int)
    s.add_argument("--start-config", default="dispersed")
    s.add_argument("--eps-max", type=float, default=0.05)
    s.set_defaults(fn=cmd_adapt)

    s = sub.add_parser("validate", help="validate planner implementations (worked examples)")
    s.add_argument("--planner", action="append")
    s.add_argument("--out", default=None, help="directory for validation reports")
    s.set_defaults(fn=cmd_validate)

    s = sub.add_parser("check-evaluator", help="re-evaluate AreaCoverage reference routes (sanity check)")
    s.add_argument("src")
    s.add_argument("--subsets", nargs="*")
    s.add_argument("--limit", type=int)
    s.add_argument("--flag-convention", choices=["from", "to"], default="from")
    s.add_argument("--out", default=None)
    s.set_defaults(fn=cmd_check_evaluator)

    s = sub.add_parser("run", help="run an experiment")
    s.add_argument("config")
    s.add_argument("--workers", type=int)
    s.add_argument("--inline", action="store_true", help="run in-process (debugging)")
    s.add_argument("--no-resume", action="store_true")
    s.add_argument("--output")
    s.set_defaults(fn=cmd_run)

    s = sub.add_parser("tune", help="tune planners on the tuning split (Bayesian optimisation)")
    s.add_argument("config")
    s.add_argument("--budget", type=int, default=20)
    s.add_argument("--planner", action="append")
    s.add_argument("--workers", type=int)
    s.add_argument("--inline", action="store_true")
    s.add_argument("--seed", type=int, default=0)
    s.add_argument("--penalty", type=float, default=10.0)
    s.add_argument("--tuning-seeds", type=int, default=3)
    s.set_defaults(fn=cmd_tune)

    s = sub.add_parser("evaluate", help="evaluate a trajectory set (common trajectory format) on an instance")
    s.add_argument("instance")
    s.add_argument("trajectories")
    s.set_defaults(fn=cmd_evaluate)

    s = sub.add_parser("report", help="statistical report of a results directory")
    s.add_argument("results")
    s.add_argument("--out")
    s.set_defaults(fn=cmd_report)

    s = sub.add_parser("replay", help="HTML replay of stored runs (side by side)")
    s.add_argument("results")
    s.add_argument("--instance")
    s.add_argument("--planners", nargs="*")
    s.add_argument("--out")
    s.add_argument("--max-cells", type=int, default=320)
    s.set_defaults(fn=cmd_replay)

    s = sub.add_parser("figure", help="static figure of one run")
    s.add_argument("results")
    s.add_argument("run_id")
    s.add_argument("--out")
    s.set_defaults(fn=cmd_figure)

    s = sub.add_parser("demo", help="small end-to-end demonstration")
    s.add_argument("--out", default="demo")
    s.add_argument("--workers", type=int, default=max(1, min(4, (os.cpu_count() or 2) - 1)))
    s.set_defaults(fn=cmd_demo)
    return ap


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    rc = args.fn(args)
    return int(rc or 0)


if __name__ == "__main__":
    sys.exit(main())

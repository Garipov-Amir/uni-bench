# uninbench

**A unified benchmark for multi-robot coverage path planning.**

uninbench compares coverage planners fairly:

* Every planner solves the same canonical instances, converted into its native representation (grid, polygon,
  swaths or targets).
* Every plan is judged by the same planner-independent evaluator.
* Every planner gets the same tuning budget.
* Every result is recorded with what is needed to reproduce it.

You work with it through a **terminal UI**, a **web UI**, or **headless commands**.

## Install

```bash
python3 -m venv .venv
.venv/bin/pip install -e ".[test]"
source .venv/bin/activate          # or call .venv/bin/uninbench directly
```

It requires Python 3.10 or newer. `uninbench validate` should print `PASS` for every planner.

## Start

```bash
mkdir ~/coverage-study && cd ~/coverage-study
uninbench init                     # data/, experiments/, results/ and an example experiment
uninbench                          # terminal UI
uninbench web --open               # … or the web UI at http://localhost:8350/
```

In either UI:

1. **Instances → Generate…** creates synthetic maps, or **Import…** imports the AreaCoverage, field or indoor-room
   datasets.
2. **Experiments → Check** estimates the number of runs; **Run…** starts the experiment as a background job.
3. **Jobs** follows its progress. Jobs keep running when you close the UI.
4. **Results → Build report / Open report** gives success rates, statistical tests, performance profiles,
   scaling curves and Pareto fronts. **Replay** animates all planners side by side.

The same without a UI:

```bash
uninbench run experiments/quick.yaml
uninbench report results/quick     # → results/quick/report/index.html
uninbench replay results/quick     # → results/quick/replay/<instance>.html
```

Or try everything at once with `uninbench demo --out demo`.

## What's included

* **Instance families:** AreaCoverage polygons (IF1), agricultural fields (IF2), paired empty/furnished rooms
  (IF3), SafeLog connectivity graphs (IF4) and a seeded synthetic generator (IF5).
* **Planners:** STC, MSTC, DARP+STC, a DARP + back-and-forth composition, boustrophedon swaths, boustrophedon
  cellular decomposition, and a k-means/TSP target baseline. You can add your own.
* **Profiles:** core; agricultural (turning radius, capacity, headlands); deconfliction; heterogeneous teams.
* **Evaluation:** coverage against the reachable coverable set F\*, split into representation loss and planning
  loss. Also overlap, makespan normalized by a lower bound, turns, workload balance, collisions and conflicts,
  with a resolution-sensitivity check.
* **Statistics:** medians with bootstrap CIs, Wilcoxon + Holm, Friedman + Nemenyi critical-difference diagrams,
  Dolan–Moré performance profiles, scaling exponents and Pareto fronts.

## Documentation

The user guide is in [`docs/`](docs/README.md):

| | |
|---|---|
| [Getting started](docs/getting-started.md) | install, first workspace, first experiment |
| [Workspaces and jobs](docs/workspace.md) | files, background jobs |
| [Terminal UI](docs/tui.md) · [Web UI](docs/web.md) | every tab, key and button |
| [Instances and datasets](docs/instances.md) | generating, importing, splitting; the instance format |
| [Experiments](docs/experiments.md) | the configuration reference, sweeps, profiles, scale |
| [Planners and adapters](docs/planners.md) | built-in planners and their hyperparameters |
| [Evaluation and metrics](docs/evaluation.md) | outcomes, metrics, the trajectory format, evaluating external planners |
| [Tuning and validation](docs/tuning-and-validation.md) | fair tuning, validating implementations |
| [Results, reports and replays](docs/results.md) | output files, reading the report, the replay player |
| [Extending](docs/extending.md) | your own planners, stages, adapters, importers |
| [Command reference](docs/cli.md) · [Troubleshooting](docs/troubleshooting.md) | |

## Development

```bash
.venv/bin/python -m pytest          # test suite
```

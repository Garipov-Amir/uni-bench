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

It requires Python 3.10 or newer. After `scripts/fetch_datasets.sh`, `uninbench validate` should print `PASS` for every planner.

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

* **Instance families:** AreaCoverage polygons (IF1), the 350 Fields2Benchmark fields (IF2), paired
  empty/furnished rooms (IF3), the 21 SafeLog connectivity graphs (IF4) and a seeded synthetic generator (IF5).
  `scripts/fetch_datasets.sh` downloads the public datasets.
* **Planners (reimplementations of published algorithms):** STC, MSTC, MSTC\*, TMSTC\*, MFC, MIP, LS-MCPP, DARP,
  GM-VPC (grid/graph); CRC of Karapetyan et al. and SCoPP (geometric); MCFS (swath/isoline).
  MFC, MSTC\*, MIP and LS-MCPP are validated against published results.
* **Compositions and baselines** (always marked `*`): DARP + back-and-forth, boustrophedon swaths, boustrophedon
  cells with a contiguous split, k-means/TSP and a genetic algorithm on coverage targets. You can add your own.
* **Profiles:** core; agricultural (turning radius, capacity, headlands); deconfliction; heterogeneous teams.
* **Evaluation:** coverage against the reachable coverable set F\*, split into representation loss and planning
  loss. Also overlap, makespan normalized by a lower bound, turns, workload balance, collisions and conflicts,
  with a resolution-sensitivity check.
* **Statistics:** medians with bootstrap CIs, Wilcoxon + Holm, Friedman + Nemenyi critical-difference diagrams,
  Dolan–Moré performance profiles, scaling exponents and Pareto fronts.

## Design requirements and where they are implemented

The framework implements the design of the accompanying thesis (Chapter 4). Requirements R1–R12:

| | Requirement | Implementation |
|---|---|---|
| R1 | one physical ground truth of the highest geometric fidelity | `instance/` (vector canonical instance, content hash), `instance/coverable.py` (F\*) |
| R2 | native representations, explicit and recorded conversions | `adapters/` (grid, graph, geometric, swath, target; transformation record ρ_i) |
| R3 | environmental equivalence verified; representation loss separated | `adapters/base.py` (fidelity ε̄, flag), `reconstruction.py`, `evaluator/` (U_rep / U_plan), `runner/worker.py` (failure attribution) |
| R4 | only planners of the same task are compared | `planners/base.py` (manifest, admission) |
| R5 | metrics from executed trajectories only | `trajectory.py` (normal form, re-timing), `evaluator/` |
| R6 | exchangeable, separately evaluated stages | `planners/base.py` (staged mode), stage metrics in reports, provenance marking of compositions |
| R7 | diverse, controlled instances | `datasets/` (IF1–IF5), `instance/team.py` (team size and start configuration) |
| R8 | equal tuning without overfitting | `instance/splits.py` (stratified splits), `tuning/` (Bayesian optimisation, frozen θ\*) |
| R9 | reimplementations validated before use | `validation/` (worked examples, published results), runner gate |
| R10 | evaluation resolution finer than and independent of planning resolution; sensitivity | `evaluator/evaluate.py` (δ_e/2 check), `runner/experiment.py` (δ_e < δ_p) |
| R11 | deterministic and stochastic planners with statistical support | `runner/` (seeds, repetitions, fresh representation per call, per-run memory), `reporting/` |
| R12 | optional domain constraints | `profiles.py`, `planners/execution.py` (framework deconfliction stage as a composition) |

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

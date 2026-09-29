# uninbench user guide

uninbench benchmarks multi-robot coverage path planners (MCPP) fairly.

* Every planner receives the same canonical instance, converted into its native representation (grid,
  polygon, swaths or targets).
* Every plan is judged by the same planner-independent evaluator.
* Every result is recorded with enough information to reproduce it.

You can drive it in three ways, and all three work on the same *workspace*:

| Interface | Start it with | Best for |
|-----------|---------------|----------|
| Terminal UI (TUI) | `uninbench` | day-to-day work in a terminal, including over SSH |
| Web UI | `uninbench web --open` | browsing instances, editing experiments, reading results |
| Headless commands | `uninbench run …`, `uninbench report …` | scripts, clusters, CI |

## Contents

1. [Getting started](getting-started.md): installation, your first workspace, your first experiment.
2. [Workspaces and jobs](workspace.md): what lives where, and how background jobs work.
3. [The terminal UI](tui.md): tabs, keys and workflows.
4. [The web UI](web.md): pages, security, remote access.
5. [Instances and datasets](instances.md): generating, importing and splitting instances; the instance format.
6. [Experiments](experiments.md): the configuration reference, factor sweeps, profiles, and running at scale.
7. [Planners and adapters](planners.md): the built-in planners, their hyperparameters, and representation
   adapters.
8. [Evaluation and metrics](evaluation.md): how plans are judged; outcomes; the trajectory format; evaluating
   your own planner's output.
9. [Tuning and validation](tuning-and-validation.md): fair tuning on the tuning split; validating
   implementations.
10. [Results, reports and replays](results.md): output files, reading the statistical report, the replay player.
11. [Extending uninbench](extending.md): adding planners, stages, adapters and importers.
12. [Command reference](cli.md): every headless command and option.
13. [Troubleshooting and FAQ](troubleshooting.md).

## Vocabulary

| Term | Meaning |
|------|---------|
| **Instance** | One benchmark problem: the environment (a polygon with holes, in metres), a robot team, a task specification and evaluation settings. Stored as JSON and identified by a content hash. |
| **Instance set** | A directory of instances (e.g. `data/IF5`) with an `index.json` that assigns every instance to the *tuning* or the *test* split. |
| **Family** | IF1–IF5: polygonal environments, agricultural fields, indoor rooms, connectivity graphs, synthetic maps. |
| **Adapter** | Converts the canonical instance into a planner's native representation, and measures how faithful the conversion is. |
| **Planner** | An algorithm with a *manifest* declaring its task class, supported constraints and hyperparameters. |
| **Run** | One planner on one instance, with one team size, start configuration, configuration and seed. |
| **Outcome** | `success`, `incomplete`, `invalid`, `timeout`, `crash` or `not_applicable`. |
| **F\*** | The reachable coverable set: the area the robots can physically sweep. Completeness is measured against it. |
| **T_LB** | A lower bound on the makespan. `T / T_LB` (normalized makespan) is the main quality measure. |
| **Job** | A background command started from a UI, such as a run, tuning, a report or an import. |

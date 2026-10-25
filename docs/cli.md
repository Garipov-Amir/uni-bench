# Command reference

Every command works on files relative to the current directory (normally the workspace root). Running `uninbench`
without a command opens the terminal UI; everything below is the headless interface used by the UIs'
background jobs, by scripts and on clusters. `uninbench <command> --help` prints the same information.

| Command | Purpose |
|---|---|
| [`tui`](#tui) | open the terminal user interface (default when no command is given) |
| [`web`](#web) | serve the browser user interface for a workspace |
| [`init`](#init) | create a workspace (data/, experiments/, results/, ...) |
| [`generate`](#generate) | generate synthetic instance families (IF5, IF4-like) |
| [`import`](#import) | import a public dataset into canonical instances |
| [`split`](#split) | (re)assign tuning/test splits of a directory |
| [`info`](#info) | describe instances |
| [`planners`](#planners) | list registered planners and their manifests |
| [`adapt`](#adapt) | run an adapter on an instance and print its transformation record |
| [`validate`](#validate) | validate planner implementations (worked examples and published results) |
| [`check-evaluator`](#check-evaluator) | re-evaluate AreaCoverage reference routes (sanity check) |
| [`run`](#run) | run an experiment |
| [`tune`](#tune) | tune planners on the tuning split (Bayesian optimisation) |
| [`evaluate`](#evaluate) | evaluate a trajectory set (common trajectory format) on an instance |
| [`report`](#report) | statistical report of a results directory |
| [`replay`](#replay) | HTML replay of stored runs (side by side) |
| [`figure`](#figure) | static figure of one run |
| [`demo`](#demo) | small end-to-end demonstration |

## tui

Opens the terminal UI (see [The terminal UI](tui.md)). This is the default when `uninbench` is run without a command in an interactive terminal.

```
uninbench tui [-h] [-w WORKSPACE]
```

| Argument | Meaning | Default |
|---|---|---|
| `-w, --workspace` | workspace directory (default: current directory) | `.` |

Examples:

```bash
uninbench
uninbench tui -w ~/coverage-study
```

## web

Serves the browser UI (see [The web UI](web.md)). It binds to localhost only by default; `Ctrl+C` stops it, and running jobs continue.

```
uninbench web [-h] [-w WORKSPACE] [--host HOST] [--port PORT] [--open]
```

| Argument | Meaning | Default |
|---|---|---|
| `-w, --workspace` | workspace directory (default: current directory) | `.` |
| `--host` | interface to bind (default: localhost only) | `127.0.0.1` |
| `--port` | TCP port (default: 8350) | `8350` |
| `--open` | open the browser |  |

Examples:

```bash
uninbench web --open
uninbench web -w ~/coverage-study --port 8400
```

## init

Creates the workspace layout (data/, experiments/, results/, validation/, datasets/, .uninbench/jobs/) and `experiments/quick.yaml`. It is safe to run again.

```
uninbench init [-h] [--no-example] [dir]
```

| Argument | Meaning | Default |
|---|---|---|
| `dir` | workspace directory (default: current directory) |  |
| `--no-example` | do not create experiments/quick.yaml |  |

Examples:

```bash
uninbench init ~/coverage-study
```

## generate

Generates instance sets, writes `index.json` and assigns tuning/test splits. `--param` passes generator parameters (see [Instances](instances.md#synthetic-instances-if5)).

```
uninbench generate [-h] --out OUT [--count COUNT] [--seed SEED] [--sizes [SIZES ...]] [--densities [DENSITIES ...]] [--shapes [SHAPES ...]] [--nonconvexity [NONCONVEXITY ...]] [--aspect ASPECT] [--rows ROWS] [--cols COLS] [--cell-size CELL_SIZE] [--param PARAM] [--tuning-fraction TUNING_FRACTION] [--stratify STRATIFY] {synthetic,safelog}
```

| Argument | Meaning | Default |
|---|---|---|
| `kind` | synthetic environments (IF5) or SafeLog-like graphs (IF4) (synthetic, safelog) |  |
| `--out` | output directory for the instance set, e.g. data/IF5 |  |
| `--count` | instances per factor combination (default: 5) | `5` |
| `--seed` | base seed; every instance stores its own seed |  |
| `--sizes` | synthetic: environment widths in metres |  |
| `--densities` | synthetic: obstacle area fractions; safelog: edge densities (default: 21-step sweep) |  |
| `--shapes` | synthetic: obstacle shapes (rect, polygon, circle, mixed) |  |
| `--nonconvexity` | synthetic: numbers of boundary notches |  |
| `--aspect` | synthetic: height / width (default: 0.75) | `0.75` |
| `--rows` | safelog: lattice rows (default: 20) | `20` |
| `--cols` | safelog: lattice columns (default: 20) | `20` |
| `--cell-size` | safelog: cell size in metres (default: 2) | `2.0` |
| `--param` | generator parameter key=value |  |
| `--tuning-fraction` | share of the tuning split (default: 0.2) | `0.2` |
| `--stratify` | factor(s) to stratify the split by; `auto`: the factor the instance family varies; `none`: no stratification | `auto` |

Examples:

```bash
uninbench generate synthetic --out data/IF5 --count 10 --sizes 20 40 80 --densities 0.05 0.1 0.2
uninbench generate synthetic --out data/IF5-small --count 3 --param robot_width=0.5 --param min_gap=4
uninbench generate safelog --out data/IF4 --rows 20 --cols 20
```

## import

Converts a downloaded dataset into canonical instances and assigns splits. Importer parameters are listed in [Instances](instances.md#public-datasets).

```
uninbench import [-h] --out OUT [--param PARAM] [--tuning-fraction TUNING_FRACTION] [--stratify STRATIFY] {areacoverage,fields,rooms,safelog} src
```

| Argument | Meaning | Default |
|---|---|---|
| `dataset` | dataset format (areacoverage, fields, rooms, safelog) |  |
| `src` | directory (or file) with the raw dataset |  |
| `--out` | output directory for the instance set, e.g. data/IF1 |  |
| `--param` | importer parameter key=value (e.g. width=3.0) |  |
| `--tuning-fraction` | share of the tuning split (default: 0.2) | `0.2` |
| `--stratify` | factor(s) to stratify the split by; `auto`: the factor the instance family varies; `none`: no stratification | `auto` |

Examples:

```bash
uninbench import areacoverage datasets/AreaCoverage-dataset --out data/IF1
uninbench import safelog datasets/safelog --out data/IF4
uninbench import fields datasets/fields --out data/IF2 --param width=3.0
uninbench import rooms datasets/test_maps --out data/IF3 --param resolution=0.05
uninbench import safelog datasets/safelog --out data/IF4 --param rows=20 --param cols=20
```

## split

Reassigns the tuning/test split of an instance set. Assignment is deterministic, stratified by the given factor, and keeps paired instances together.

```
uninbench split [-h] [--tuning-fraction TUNING_FRACTION] [--stratify STRATIFY] dir
```

| Argument | Meaning | Default |
|---|---|---|
| `dir` | instance set directory |  |
| `--tuning-fraction` | share of the tuning split (default: 0.2) | `0.2` |
| `--stratify` | factor(s) to stratify the split by; `auto`: the factor the instance family varies; `none`: no stratification | `auto` |

Examples:

```bash
uninbench split data/IF5 --tuning-fraction 0.2
```

## info

Prints a description of each instance: extent, free and coverable area, T_LB, factors and split.

```
uninbench info [-h] instances [instances ...]
```

| Argument | Meaning | Default |
|---|---|---|
| `instances` | instance JSON files |  |

Examples:

```bash
uninbench info data/IF5/*.json
```

## planners

Lists the registered planners (built-in, entry points).

```
uninbench planners [-h] [--json]
```

| Argument | Meaning | Default |
|---|---|---|
| `--json` | print full manifests as JSON |  |

Examples:

```bash
uninbench planners
uninbench planners --json > manifests.json
```

## adapt

Runs an adapter on one instance and prints its transformation record (parameters, runtime, fidelity), without planning.

```
uninbench adapt [-h] [--adapter ADAPTER] [--param PARAM] [--team-size TEAM_SIZE] [--start-config START_CONFIG] [--eps-max EPS_MAX] instance
```

| Argument | Meaning | Default |
|---|---|---|
| `instance` | instance JSON file |  |
| `--adapter` | grid, graph, geometric, swath or target (default: grid) | `grid` |
| `--param` | adapter parameter key=value (repeatable) |  |
| `--team-size` | derive a team of this size first |  |
| `--start-config` | clustered or dispersed (default: dispersed) | `dispersed` |
| `--eps-max` | fidelity tolerance (default: 0.05) | `0.05` |

Examples:

```bash
uninbench adapt data/IF5/syn_40x30_d0.10_mixed_n0_s0.json --adapter grid --param coarse=true
uninbench adapt data/IF2/field_test.json --adapter swath --param angle=longest_edge --team-size 2
```

## validate

Runs the validation cases of the planners and writes `<out>/<planner>.json`. The exit code is 1 if any case fails.

```
uninbench validate [-h] [--planner PLANNER] [--out OUT] [--no-published]
```

| Argument | Meaning | Default |
|---|---|---|
| `--planner` | validate only this planner (repeatable) |  |
| `--no-published` | skip the comparison with published results (`datasets/MIP-MCPP`) |  |
| `--out` | directory for validation reports |  |

Examples:

```bash
uninbench validate --out validation
uninbench validate --planner darp_stc --planner mstc
```

## check-evaluator

Re-evaluates the reference solutions shipped with the AreaCoverage dataset. The exit code is 1 if any solution is not valid and complete.

```
uninbench check-evaluator [-h] [--subsets [SUBSETS ...]] [--limit LIMIT] [--flag-convention {from,to}] [--out OUT] src
```

| Argument | Meaning | Default |
|---|---|---|
| `src` | AreaCoverage-dataset directory |  |
| `--subsets` | VM25, AC300, H2, RAL_main (default: all present) |  |
| `--limit` | at most this many instances per subset |  |
| `--flag-convention` | meaning of the route flag column (default: from) (from, to) | `from` |
| `--out` | write the check report (JSON) here |  |

Examples:

```bash
uninbench check-evaluator datasets/AreaCoverage-dataset --subsets VM25 AC300 --out validation/evaluator.json
```

## run

Runs an experiment (see [Experiments](experiments.md)). It prints a JSON summary at the end. Already recorded runs are skipped unless `--no-resume` is given.

```
uninbench run [-h] [--workers WORKERS] [--inline] [--no-resume] [--output OUTPUT] config
```

| Argument | Meaning | Default |
|---|---|---|
| `config` | experiment configuration (YAML) |  |
| `--workers` | worker processes (default: from the configuration) |  |
| `--inline` | run in-process (debugging; no timeouts) |  |
| `--no-resume` | discard recorded runs and start over |  |
| `--output` | override the output directory |  |

Examples:

```bash
uninbench run experiments/study.yaml --workers 8
uninbench run experiments/study.yaml --no-resume --output results/study-v2
```

## tune

Tunes the experiment's planners on the tuning split and freezes the best configurations into `tuning_dir` (see [Tuning](tuning-and-validation.md)).

```
uninbench tune [-h] [--budget BUDGET] [--planner PLANNER] [--workers WORKERS] [--inline] [--seed SEED] [--penalty PENALTY] [--tuning-seeds TUNING_SEEDS] config
```

| Argument | Meaning | Default |
|---|---|---|
| `config` | experiment configuration (YAML); its tuning split is used |  |
| `--budget` | evaluations per planner and family (default: 20) | `20` |
| `--planner` | tune only this planner (repeatable) |  |
| `--workers` | worker processes (default: from the configuration) |  |
| `--inline` | run in-process (debugging) |  |
| `--seed` | optimiser seed (default: 0) |  |
| `--penalty` | objective of unsuccessful runs (default: 10) | `10.0` |
| `--tuning-seeds` | seeds per evaluation for stochastic planners | `3` |

Examples:

```bash
uninbench tune experiments/study.yaml --budget 30
uninbench tune experiments/study.yaml --planner darp_stc --budget 50 --tuning-seeds 5
```

## evaluate

Evaluates a trajectory set in the common trajectory format and prints outcome and metrics as JSON (see [Evaluation](evaluation.md#evaluating-your-own-planners-output)).

```
uninbench evaluate [-h] instance trajectories
```

| Argument | Meaning | Default |
|---|---|---|
| `instance` | instance JSON file |  |
| `trajectories` | trajectory set JSON (common trajectory format) |  |

Examples:

```bash
uninbench evaluate data/custom/hall.json my_planner_output.json
```

## report

Builds the statistical report `report/index.html` and its CSV/JSON tables (see [Results](results.md#the-statistical-report)).

```
uninbench report [-h] [--out OUT] results
```

| Argument | Meaning | Default |
|---|---|---|
| `results` | results directory of an experiment |  |
| `--out` | report directory (default: <results>/report) |  |

Examples:

```bash
uninbench report results/study
```

## replay

Builds a side-by-side HTML replay of the stored runs of one instance.

```
uninbench replay [-h] [--instance INSTANCE] [--planners [PLANNERS ...]] [--out OUT] [--max-cells MAX_CELLS] results
```

| Argument | Meaning | Default |
|---|---|---|
| `results` | results directory of an experiment |  |
| `--instance` | derived instance id (default: the one with most runs) |  |
| `--planners` | only these planners |  |
| `--out` | output HTML file (default: <results>/replay/<instance>.html) |  |
| `--max-cells` | coverage-map display resolution (default: 320) | `320` |

Examples:

```bash
uninbench replay results/study
uninbench replay results/study --instance syn_40x30_d0.10_mixed_n0_s0__m4_dispersed --planners mstc darp_stc
```

## figure

Draws a static PNG of one run: coverage count map, obstacles, paths.

```
uninbench figure [-h] [--out OUT] results run_id
```

| Argument | Meaning | Default |
|---|---|---|
| `results` | results directory of an experiment |  |
| `run_id` | run id (see runs.jsonl or report/runs.csv) |  |
| `--out` | output PNG (default: <results>/figures/<run_id>.png) |  |

Examples:

```bash
uninbench figure results/study 5d8ebaeeda2dd0da
```

## demo

Generates small synthetic and graph instance sets, runs all built-in planners with 1, 2 and 4 robots, and builds a report and a replay.

```
uninbench demo [-h] [--out OUT] [--workers WORKERS]
```

| Argument | Meaning | Default |
|---|---|---|
| `--out` | output directory (default: demo) | `demo` |
| `--workers` |  | `4` |

Examples:

```bash
uninbench demo --out demo --workers 4
```


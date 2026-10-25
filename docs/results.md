# Results, reports and replays

## The results directory

```
results/<experiment>/
├── experiment.json        resolved configuration, planner manifests, hardware and software
├── runs.jsonl             one record per run, including all failures
├── summary.json           outcome counts of the last invocation
├── excluded.json          planners excluded from the comparison, with reasons
├── instances/             every derived instance that was run (<id>__<hash>.json)
├── runs/                  <run_id>.json.gz (trajectories, intermediate structures, reconstruction)
│                          <run_id>.coverage.json.gz (coverage count map, sweep events)
├── report/                index.html + CSV and JSON tables (after `report`)
├── replay/                <instance>.html (after `replay`)
└── figures/               <run_id>.png (after `figure`)
```

## Run records

Every line of `runs.jsonl` is one run:

| Field | Meaning |
|-------|---------|
| `run_id` | deterministic id (instance hash, planner, version, configuration, seed, adapter parameters, limits, factors) |
| `instance_id`, `base_instance`, `instance_hash`, `family`, `split`, `factors`, `team_size` | what was solved |
| `planner`, `planner_name`, `planner_version`, `planner_mode`, `stochastic` | who solved it |
| `provenance`, `composition_of`, `framework_stages` | `original`, `composition` or `baseline`; the components of a composition; framework stages composed with the planner (e.g. `prioritized_waiting`) |
| `config_label`, `config`, `config_hash`, `seed` | how: `default`, `tuned` or `custom`, the hyperparameters, the seed |
| `outcome`, `reason`, `warnings`, `traceback` | result classification (see [Evaluation](evaluation.md#outcomes)) |
| `failure_attribution` | for failures: `adapter` (flagged representation or adapter error) or `planner` |
| `metrics`, `per_robot` | all metrics (see [Evaluation](evaluation.md#metrics)) |
| `planning_time`, `planning_times`, `peak_rss_mb`, `adapter_time`, `eval_time` | computation |
| `memory` | `isolated` (fresh worker process), process peak after the adapter and after planning, planning growth |
| `adapter` | the transformation record: parameters, runtime, fidelity (`rel_error`, `flagged`) |
| `delta_p`, `delta_e`, `sensitivity` | planning and evaluation resolutions, and the resolution check |
| `source_resolution` | resolution of raster-derived instances (bounds the achievable fidelity) |
| `stage_metrics`, `stage_outputs`, `planner_info` | intermediate evaluations of staged planners, e.g. DARP partition balance and convergence |
| `limits`, `hardware`, `software`, `profiles`, `started`, `finished` | reproducibility information |
| `artifacts` | paths of the stored trajectories, coverage map and derived instance |

Load the records in Python:

```python
from uninbench.runner import load_runs
import pandas as pd                                       # optional
from uninbench.reporting.aggregate import flatten

runs = load_runs("results/study")
df = pd.DataFrame([flatten(r) for r in runs])             # one row per run, metrics as columns
df[df.outcome == "success"].groupby(["planner", "team_size"]).normalized_makespan.median()
```

## The statistical report

* **TUI / web:** *Results → Build report*, then *Open report*.
* **Headless:** `uninbench report results/study` writes `results/study/report/index.html`.

The report page is self-contained and can be mailed or archived. Its sections are:

### Outcomes and success rates

This section shows the share of each outcome per planner and team size, and a table of counts.

* Not-applicable combinations are excluded from the denominator.
* The success rate excludes failures attributed to a flagged adapter (column *adapter-attr.*); *incl. adapter*
  counts them against the planner.
* Quality metrics elsewhere in the report are computed on successful runs only, and always appear together with
  the success rate. A planner therefore cannot look better by failing on hard instances.
* Planner names marked `*` are not original published algorithms (compositions, baselines, `+pw` planners
  composed with the framework's deconfliction stage).

### Descriptive statistics

These are medians, interquartile ranges and 95 % confidence intervals of the key metrics, per family, team size,
configuration and planner. The confidence intervals are percentile bootstrap intervals over instances.

* Stochastic planners first take the median over seeds per instance.
* Their variability across seeds is reported separately, as the median seed IQR and coefficient of variation.
* Box plots show the distributions.

### Hypothesis tests

The comparisons are **paired**, using only instances that every compared planner ran on. For each family, team
size and configuration:

* The **Friedman test** compares all planners on per-instance ranks. Failed runs rank last. The Iman–Davenport
  correction is also given.
* The **critical-difference diagram** (Nemenyi, α = 0.05) places planners by average rank, where lower is better.
  A thick bar joins planners whose average ranks differ by less than the critical difference (CD); they are *not*
  significantly different.
* **Pairwise Wilcoxon signed-rank tests** use Holm-adjusted p-values. A failure counts as the largest possible
  difference in favour of the other planner (both failing is a tie), consistent with the Friedman ranks; the
  table lists how many pairs each planner failed. The effect sizes are the median paired difference and the
  matched-pairs rank-biserial correlation *r* (−1 … 1).
* Instances on which a planner's failure is attributed to a flagged adapter are left out of its comparisons.

### Performance profiles

For makespan and for planning time, each curve shows the fraction of instances on which a planner is within a
factor τ of the best planner on that instance. Failures have an infinite ratio.

* The value at τ = 1 is how often a planner is the best.
* The right end shows its robustness: the share of instances it solves at all.
* Curves higher and further left are better.

### Scaling

These plots show metrics and planning time against team size, environment area, obstacle density and graph
density, with 95 % bootstrap bands. For planning time the plots use log–log axes, and the slope is given as an
**empirical scaling exponent**: a slope of 2 means that doubling the factor quadruples the time.

### Quality–time trade-off and implementation complexity

The **Pareto front** plots the median quality T/T_LB against the median planning time per family; the dashed
line joins the non-dominated planners.

The complexity table lists coarse indicators per planner: provenance, source lines of the core algorithm, number
of hyperparameters, number of pipeline stages, and external dependencies (e.g. the MILP solver of `mip`, `mcfs`). They make the trade-off between
improvement and complexity explicit.

### Intermediate outputs of staged planners

Every numeric stage metric (median and IQR per family, team size and planner), e.g. partition balance and
connectivity before any route is built, tour-split imbalance, or the makespan before and after local search (R6).

### Representation fidelity and resolutions

For each planner and adapter, this section gives the median and maximum relative reconstruction error ε̄, the
number of flagged runs and of adapter-attributed failures, the source resolution of raster-derived instances, and
the planning and evaluation resolutions used. The uncovered area is split into
**representation loss** and **planning loss**.

### Raw data

The report directory also contains:

* `runs.csv`: flattened runs;
* `success.csv`, `descriptives.csv`, `seed_variability.csv`, `fidelity.csv`, `stages.csv`, `pareto.csv`;
* `stats.json`: test results, complexity and scaling data.

## Replays

A replay is an HTML page that animates several planners side by side on the same instance, using the stored,
re-timed trajectories. Create one:

* **TUI:** *Results → Replay…*
* **Web:** *Results → Replay*
* **Headless:**

  ```bash
  uninbench replay results/study --instance syn_40x30_d0.10_mixed_n0_s0__m4_dispersed [--planners mstc darp_stc]
  ```

In the player:

| Control | What it does |
|---------|--------------|
| ▶ Play / ❚❚ Pause, time slider | move through time; all panels share the clock |
| speed | slow, normal, fast |
| coverage count n(x) / work distribution | colour cells by how often they were swept (1, 2, ≥ 3), or by the robot that swept them first |
| layer toggles | show intermediate structures: planning grid, coarse grid, spanning tree, DARP partition, cells, swaths, targets, clusters, pass assignment, and the adapter's reconstruction F̂ |

Each panel lists the metrics copied from the run record. The player computes no metrics itself, so what you see
and what the report says cannot diverge.

Use the replay to debug. Overlaying F̂ exposes representation loss, and replaying a failed run shows where a plan
leaves gaps or collides.

## Static figures

```bash
uninbench figure results/study <run_id>        # → results/study/figures/<run_id>.png
```

This draws the coverage count map, the obstacles, the robot paths and, if present, swaths or spanning trees.
Run ids are listed in `runs.jsonl` and `report/runs.csv`.

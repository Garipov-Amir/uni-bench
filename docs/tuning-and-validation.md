# Tuning and validation

## Why tune, and how uninbench keeps it fair

A planner's published defaults may suit the published experiments better than yours. uninbench gives every planner
the **same, systematic tuning opportunity**, and prevents tuning from overfitting to the instances that results are
reported on:

1. Every instance set is split into **tuning** and **test** instances (default 20 % / 80 %, stratified by a
   factor; see [Instances](instances.md#tuning-and-test-splits)).
2. `uninbench tune` runs **Bayesian optimization** (a Gaussian process with expected improvement) on the tuning
   split only. Every planner and family gets the **same budget** of planner evaluations.
3. The objective is the mean normalized makespan T/T_LB over the tuning instances, team sizes and start
   configurations, plus a fixed number of seeds for stochastic planners. Unsuccessful runs (incomplete, invalid,
   timeout, crash) count as a fixed **penalty** (default 10).
4. The best configuration θ\* is **frozen** to `tuning_dir/<planner>__<family>.json` before the test split is
   used.
5. Experiments report both the default θ₀ (`configs: [default]`) and the tuned θ\* (`configs: [tuned]`). The
   difference between the two says how robust a planner is.

## Tuning in practice

1. Make sure your instance sets have tuning instances (the Instances tab shows the split counts).
2. Add `tuning_dir: results/tuning` to the experiment.
3. Start tuning:
   * **TUI or web:** *Experiments → Tune…*, then choose the budget and the seeds per evaluation.
   * **Headless:**

     ```bash
     uninbench tune experiments/study.yaml --budget 30 --tuning-seeds 3 [--planner darp_stc]
     ```

   The job log prints every evaluation (`eval k/30: objective …`) and a summary line per planner:
   `darp_stc / IF5: objective 1.84 (default) -> 1.61 (tuned)`.
4. Use `configs: [default, tuned]` in the experiment and run it on the `test` split.

Each tuning record (`results/tuning/<planner>__<family>.json`) contains:

* the budget, seed, penalty and tuning instances;
* θ₀ with its objective, and θ\* with its objective;
* the full history of evaluated configurations with their success rates;
* `frozen: true` with a timestamp.

The individual tuning runs are logged in `<planner>__<family>.runs.jsonl` next to it.

Notes:

* Planners without hyperparameters are evaluated once, and θ\* = θ₀.
* Adapter parameters are *not* tuned: they are fixed per experiment. Parameters that the original authors treat as
  algorithmic choices are exposed as planner hyperparameters instead.
* To re-tune, delete the frozen record or point `tuning_dir` at a new directory. Runs never silently use an
  unfrozen configuration.

## Validating implementations

Before a planner takes part in a study, its implementation is validated against the worked examples and published
properties of its original publication, with declared tolerances. Examples:

| Planner | Validation cases |
|---------|------------------|
| `stc` | the circumnavigation visits every sub-cell exactly once; exact path lengths on open and obstructed grids |
| `mstc` | the non-backtracking split yields disjoint segments that cover the tour |
| `darp_stc` | an optimal division (sizes differ by at most one cell, connected regions) on obstacle-free and uneven grids |
| `darp_bf` | the DARP stage divides optimally; the back-and-forth stage covers every region |
| `swath_bf` | exact path length and coverage on a rectangle; complete coverage with a rotated obstacle |
| `bcd` | one hole gives four boustrophedon cells, which are covered completely |
| `kmeans_tsp` | balanced clusters and complete coverage |

Run the cases:

* **TUI:** *Planners → Validate all* or *Validate selected*.
* **Web:** *Planners → Validate all*.
* **Headless:**

  ```bash
  uninbench validate --out validation            # all planners
  uninbench validate --planner darp_stc --out validation
  ```

Reports are written to `validation/<planner>.json`, listing every case, the measured and expected values, and
pass/fail. The Planners tab shows the latest result per planner.

To enforce validation in a study, set `require_validation: true`. Planners without a passing report are then
excluded, and the reason is written to `excluded.json`.

## Checking the evaluator

The AreaCoverage dataset ships reference routes computed by its authors. Re-evaluating them tests the evaluator
without involving any planner:

```bash
uninbench check-evaluator datasets/AreaCoverage-dataset --out validation/evaluator_reference_routes.json
```

Every reference solution should be *valid* with coverage ≈ 1.0. A failure points at the evaluator, or at how the dataset is interpreted, never at a
planner.

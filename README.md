# uninbench — unified benchmarking framework for multi-robot coverage path planning

`uninbench` implements the framework designed in the thesis chapter *Design of the Unified
Benchmarking Framework*. It is a layered pipeline

```
canonical instance E_c ──adapter T_i──▶ native representation R_i ──planner P_i──▶ trajectories Π_i
        │                                   (+ transformation record ρ_i,          │
        │                                      fidelity ε̄_i, reconstruction F̂_i)   ▼
        └──────────────────────────────────────────────────────────────▶ evaluator M(E_c, Π_i)
```

An experiment runner controls which instances, team sizes, seeds and configurations are
executed. The collected records are then processed by the statistical reporting protocol and
replayed by the simulator.

## Installation

```bash
python3 -m venv .venv && .venv/bin/pip install -e ".[test]"
```

Python ≥ 3.10 is required, along with numpy, scipy, shapely ≥ 2, networkx, matplotlib, pyyaml and pillow. A
`Dockerfile` provides a versioned environment; its image tag is stored with every run.

## Quick start

```bash
uninbench demo --out demo            # generate → run 7 planners × 3 team sizes → report → replay
uninbench validate                    # worked-example validation of every planner (R9)
uninbench run examples/experiments/quick.yaml
uninbench report results/quick        # results/quick/report/index.html + CSV tables
uninbench replay results/quick        # side-by-side HTML replay of one instance
```

## Requirements → components

| ID | Requirement | Where |
|----|-------------|-------|
| R1 | one physical ground truth of highest geometric fidelity | `instance/model.py` (vector workspace, holes, robots, task, evaluation settings; content hash), `instance/coverable.py` (reachable coverable set F\*, T_LB) |
| R2 | native representations, explicit parameterised conversions | `adapters/` (grid/graph, graph identity, geometric, swath/isoline, target) returning an `AdapterRecord` ρ_i |
| R3 | verified environmental equivalence; representation vs planning loss | `adapters/base.py::fidelity` (ε_i = μ(F̂_i △ F_c)), `reconstruction.py`, evaluator `uncovered_representation` / `uncovered_planning` |
| R4 | only planners of the same task are compared | `planners/base.py::Manifest.task_class`, `admission()` (excluded / not applicable) |
| R5 | performance measured on executed trajectories | `trajectory.py` (common format, re-timing, validity), `evaluator/` |
| R6 | exchangeable, separately evaluable components | `planners/base.py::StagedPlanner` (environment model, work distribution, route construction, optimisation; typed interfaces, stage metrics) |
| R7 | diverse, controlled instances | `datasets/` (IF1 AreaCoverage, IF2 fields, IF3 paired rooms, IF4 SafeLog connectivity, IF5 seeded generator), `instance/team.py` (team size, clustered/dispersed starts) |
| R8 | equal systematic tuning without overfitting | `instance/splits.py` (stratified 20/80 tuning/test split, pairs kept together), `tuning/` (GP-EI Bayesian optimisation, equal budget, frozen θ\*) |
| R9 | validated reimplementations | `validation/cases.py` (worked examples with tolerances), `validation/reference.py` (AreaCoverage reference routes re-evaluated) |
| R10 | evaluation resolution finer than, and independent of, planning resolution | δ_e = w/10 by default, re-check at δ_e/2 with refinement (`evaluator/evaluate.py`), planning-resolution factor sweep (`factors.planning_resolution`) |
| R11 | deterministic and stochastic planners, statistical support | runner seeds/repetitions, six outcomes; `reporting/` (medians/IQR, bootstrap CIs, Wilcoxon + Holm, Friedman + Nemenyi CD diagrams, performance profiles, scaling exponents, Pareto fronts) |
| R12 | optional domain constraints | `profiles.py` (agricultural, deconfliction, heterogeneous), manifest constraint declarations |

## Canonical instances

An instance is a JSON file with four parts:

* **Environment**: outer boundary and hole polygons (WKT, metres), an optional terrain-cost raster, and the source
  resolution of raster-derived maps.
* **Team**: per robot, a start pose, tool and body footprints (`disc` or heading-aligned `rect`), speed, a turning
  model (seconds per radian of heading change, or a minimum turning radius), and optional capacity and depot.
* **Task**: task class, completeness tolerance τ (default 0.02), return-to-depot, planning time and memory limits,
  and profiles.
* **Evaluation settings**: δ_e, the sensitivity check, and collision/conflict semantics.

Every instance is identified by a content hash and carries its split, its factors and, for synthetic instances,
the generator parameters and seed needed to regenerate it exactly (`datasets.synthetic.regenerate`).

The reachable coverable set is `F* = reach(F_c ⊖ B_body) ⊕ B_tool`. The makespan lower bound is
`T_LB = max(0, μ(F*) − π Σ b_j²) / (2 Σ b_j v_j)`. Both are computed once per instance and team.

## Trajectory format

Every trajectory is a sequence of waypoints `(x, y, ψ, t)`. Each segment between waypoints carries a label:

* `service`: the tool is active.
* `transit`: the tool is inactive.
* `turn`: an in-place heading change; the tool is active only if declared.
* `wait`: the robot stands still for a declared duration.

The evaluator discards planner time stamps and re-times every trajectory with the instance motion model. A
trajectory set is valid only if it starts at the declared start poses, is continuous, ends at the depot when
required and, in the agricultural profile, respects the minimum turning radius. For the last check, a polyline is
drivable if every corner can be filleted with a radius of at least r_min. `uninbench evaluate instance.json
trajectories.json` evaluates any external planner's output.

## Evaluator

Coverage is counted at δ_e as the number of *distinct sweeps* per cell. On each active segment, the time a cell
centre spends inside the convex footprint is one interval, computed in closed form. Intervals are merged within
runs of consecutive active segments, so one continuous pass counts once and every return counts again.

Metrics:

* coverage ratio ρ;
* uncovered area, split into representation and planning loss;
* overlap ratio ω;
* path length and transit share;
* makespan, sum of costs and normalised makespan T/T_LB;
* number of turns and accumulated heading change;
* workload imbalance;
* static collisions against the uninflated obstacles, using a penetration-depth tolerance;
* inter-robot conflicts, sampled so that no robot moves more than δ_e between samples;
* capacity excess;
* planning time, peak memory, and adapter and evaluation time.

Outcomes are `success`, `incomplete`, `invalid`, `timeout`, `crash` and `not_applicable`.

## Planners (reference implementations)

| name | family / adapter | mode | notes |
|------|------------------|------|-------|
| `stc` | grid (2×2 coarse cells) | black-box | Gabriely & Rimon (2001); single robot |
| `mstc` | grid | staged: model → **route** → **split** | Hazon & Kaminka (2005); non-backtracking or balanced split |
| `darp_stc` | grid | staged: model → DARP → STC | Kapoutsis et al. (2017); stochastic; per connected component |
| `darp_bf` | grid | staged: model → DARP → back-and-forth | *composition*, reported with `*`, never as DARP |
| `swath_bf` | swath | staged: swaths → order → split → 2-opt | perimeter pass, skip pattern + Dubins headland turns, depot trips (agricultural) |
| `bcd` | geometric | staged: cells → cell order → split → 2-opt | Choset (2000) boustrophedon decomposition |
| `kmeans_tsp` | target | black-box | balanced k-means + NN/2-opt tours; labels everything as service (conservative) |

To add a planner, write a `Planner` (or `StagedPlanner`) subclass with a `Manifest` (see
`examples/custom_planner.py`). Name it in a config as `package.module:Class`, or register it through the
`uninbench.planners` entry-point group. Then add `ValidationCase`s for it.

## Experiments

An experiment configuration (YAML) lists:

* the instance sources (paths, or on-the-fly generators) and the split;
* team sizes and start configurations;
* planners with their configurations (`default`, `tuned`, or custom parameters);
* experiment-wide adapter parameters and factor sweeps (`planning_resolution`);
* profiles, the task and evaluation overrides, seeds, repetitions and workers.

See `examples/experiments/` for `quick`, `core`, `resolution`, `scaling`, `connectivity`, `furniture`,
`agricultural` and `deconfliction`.

How runs are executed:

* Every run executes in a separate worker process with per-planner-call timeouts.
* A deterministic planner is planned `repetitions` times, and the median planning time is reported.
* A stochastic planner is run once per seed from the fixed seed list shared by all planners.
* Failed runs are always recorded.
* Each record stores the instance hash, the adapter record, the planner version, the configuration, the seed,
  the limits, the hardware and software (including the git revision and container tag), the metrics, stage
  metrics and artefact paths.
* `runs.jsonl` allows resuming an interrupted experiment.

Tuning: `uninbench tune config.yaml --budget 30` runs GP-EI on the tuning split for every (planner, family). All
planners get the same budget. The objective is the mean normalised makespan, with a fixed penalty for
unsuccessful runs. The result is frozen to `tuning_dir/<planner>__<family>.json` and used by `configs: [tuned]`.

## Datasets

```bash
scripts/fetch_datasets.sh                                    # AreaCoverage, Bormann test maps, sample fields
uninbench import areacoverage datasets/AreaCoverage-dataset --out data/IF1 --stratify holes
uninbench check-evaluator datasets/AreaCoverage-dataset      # reference routes must be valid and complete
uninbench import fields datasets/fields --out data/IF2       # Fields2Cover GML/XML, GeoJSON, WKT (lon/lat → metres)
uninbench import rooms datasets/test_maps --out data/IF3     # paired empty/furnished rooms from gt segmentations
uninbench generate safelog --out data/IF4                    # or: uninbench import safelog <edge-list dir>
uninbench generate synthetic --out data/IF5 --count 10 --sizes 20 40 80 --densities 0.05 0.1 0.2
```

Dataset-specific robot models:

* **VM25** uses its robot polygon: a square tool and an inscribed-disc body.
* **AC300** models aerial robots: a 3 m square sensor footprint on a point-like body.

With these models, the reference routes in the dataset re-evaluate as valid with coverage 1.0.

## Reports and simulator

`uninbench report` writes `index.html` together with the CSV tables:

* success rates and outcome shares;
* descriptive statistics over successful runs only, always next to the success rate;
* variability across seeds;
* Friedman/Nemenyi critical-difference diagrams, and pairwise Wilcoxon tests with Holm adjustment and effect
  sizes;
* performance profiles, in which failures count as +∞;
* scaling curves with log–log exponents;
* quality–time Pareto fronts;
* implementation-complexity indicators;
* adapter fidelity and resolutions.

`uninbench replay` builds a self-contained HTML player. It shows robots moving on a shared clock and the coverage
count map n(x) building up from the evaluator's stored sweep events. It also shows the work distribution and the
intermediate structures: grids, spanning trees, DARP partitions, cells, swaths, targets and F̂. The player computes
no metrics. `uninbench figure` renders a static PNG of one run.

## Observed behaviour worth knowing

* STC-type planners on 2w coarse cells lose much of the area in cluttered maps. This loss is reported as
  representation loss, not attributed to the planners.
* DARP converges to an optimal division with dispersed starts but often not with clustered starts. This is the
  start-configuration sensitivity the benchmark is meant to expose.
* The agricultural `swath_bf` baseline uses skip patterns, Dubins turns, turning-radius-smoothed headland contours
  and depot trips. Its plans are drivable, but corners it cannot reach with r_min remain uncovered.
* Grid coverage is not monotonic in the planning resolution. With δ_p = 0.75 w, the cell centres next to walls fall
  inside the body clearance, so a whole strip along every wall drops out (see
  `examples/experiments/resolution.yaml`).
* The shipped deconfliction utility is a baseline: prioritized waiting with longest-trajectory-first priority. It
  cannot resolve conflicts with robots that have finished and parked on another robot's path. In the deconfliction
  profile such runs are therefore `invalid`, which is the intended hard-constraint semantics.

## Tests

```bash
.venv/bin/python -m pytest
```

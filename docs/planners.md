# Planners and adapters

## How a planner receives its input

The planners never see the canonical instance directly. Each planner declares an **adapter**, and the adapter
converts the instance into the planner's native representation: a grid graph, a polygon or cell decomposition,
swaths or coverage targets. The adapter also records exactly how it did so.

```
canonical instance ──adapter (parameters θ_T)──▶ representation R_i + transformation record ρ_i
                                                   └─ fidelity check: reconstruct F̂_i and compare with F_c
```

The planner then returns its native plan. A planner-specific converter turns the plan into the common trajectory
format, which the evaluator judges on the canonical instance (see [Evaluation](evaluation.md)).

## Built-in planners

| Planner | Family | Mode | Kind | Idea |
|---------|--------|------|------|------|
| `stc` | grid | black-box | deterministic | Spanning Tree Coverage: circumnavigate a spanning tree of 2×2 coarse cells (single robot) |
| `mstc` | grid | staged | deterministic | Multi-robot STC: one STC tour split among the robots (route first, split second) |
| `darp_stc` | grid | staged | stochastic | DARP divides the area into balanced connected regions; STC inside each region |
| `darp_bf` * | grid | staged | stochastic | composition: DARP regions with back-and-forth (boustrophedon) routes |
| `swath_bf` | swath | staged | deterministic | parallel swaths plus boundary contours, ordered greedily, split into balanced contiguous chunks, 2-opt; supports the agricultural profile |
| `bcd` | geometric | staged | deterministic | boustrophedon cellular decomposition; cells swept back and forth |
| `kmeans_tsp` | target | black-box | stochastic | balanced k-means over coverage targets and a 2-opt tour per robot (baseline) |

`*` marks a *staged composition*. Compositions are reported with a star and never as results of an original
algorithm.

List them, including anything you registered yourself, with `uninbench planners` (add `--json` for full
manifests) or in the *Planners* tab of either UI.

## Planner reference

The tables below are generated from the planners' manifests. The *adapter parameters* are what the planner
requires; experiment-wide parameters from `adapters:` are applied on top (see
[Experiments](experiments.md#top-level-keys)).

### `bcd`

Boustrophedon cells swept back and forth; cells ordered greedily; split at pass level.

* **Reference:** Choset (2000), Coverage of known spaces: the boustrophedon cellular decomposition
* **Family / adapter:** geometric / `geometric` with planner-required parameters `{'mode': 'boustrophedon', 'angle': 'auto'}`
* **Mode:** staged — stages `cell_passes → cell_order → contiguous_split → two_opt`
* **Kind:** deterministic (planning time is repeated and the median reported)
* **Supports:** deconfliction, multi_robot, return_to_depot

| Hyperparameter | Type | Default | Range / choices | Meaning |
|---|---|---|---|---|
| `service_links` | bool | `True` | true / false | keep the tool active on short straight links between neighbouring passes |
| `perimeter` | bool | `True` | true / false | boundary-following passes |
| `two_opt` | bool | `False` | true / false | 2-opt improvement of each robot's pass order |
| `two_opt_rounds` | int | `5` | 1 – 50 | maximum number of 2-opt improvement rounds |

### `darp_bf`

Staged composition: DARP work distribution with back-and-forth route construction.

* **Reference:** Composition: DARP partition (Kapoutsis et al., 2017) + back-and-forth sweeps
* **Family / adapter:** grid / `grid` with planner-required parameters `{'coarse': True, 'connectivity': 4}`
* **Mode:** staged — stages `coarse_regions → darp → bf_per_region`
* **Kind:** stochastic (one run per seed)
* **Supports:** deconfliction, multi_robot
* **Composition** of darp + boustrophedon — shown with a `*` in reports and never reported as an original algorithm

| Hyperparameter | Type | Default | Range / choices | Meaning |
|---|---|---|---|---|
| `max_iter` | int | `10000` | 500 – 80000 (log scale) | iterations before relaxing the threshold |
| `cc_variation` | float | `0.01` | 0.001 – 0.2 (log scale) | connectivity bias range |
| `random_level` | float | `0.0001` | 1e-06 – 0.01 (log scale) | random perturbation |
| `dcells` | int | `2` | 0 – 20 | largest accepted cell imbalance threshold |
| `metric` | categorical | `euclidean` | euclidean, geodesic | distance from the robots (the original uses Euclidean distance) |
| `sweep` | categorical | `rows` | rows, cols | back-and-forth along rows or columns of sub-cells |

### `darp_stc`

DARP partition of coarse cells followed by STC inside every region.

* **Reference:** Kapoutsis, Chatzichristofis & Kosmatopoulos (2017), DARP: Divide areas algorithm for optimal multi-robot coverage path planning
* **Family / adapter:** grid / `grid` with planner-required parameters `{'coarse': True, 'connectivity': 4}`
* **Mode:** staged — stages `coarse_regions → darp → stc_per_region`
* **Kind:** stochastic (one run per seed)
* **Supports:** deconfliction, multi_robot

| Hyperparameter | Type | Default | Range / choices | Meaning |
|---|---|---|---|---|
| `max_iter` | int | `10000` | 500 – 80000 (log scale) | iterations before relaxing the threshold |
| `cc_variation` | float | `0.01` | 0.001 – 0.2 (log scale) | connectivity bias range |
| `random_level` | float | `0.0001` | 1e-06 – 0.01 (log scale) | random perturbation |
| `dcells` | int | `2` | 0 – 20 | largest accepted cell imbalance threshold |
| `metric` | categorical | `euclidean` | euclidean, geodesic | distance from the robots (the original uses Euclidean distance) |
| `tree` | categorical | `dfs` | dfs, bfs, horizontal, vertical | spanning-tree construction on the coarse grid |
| `direction` | categorical | `ccw` | ccw, cw | circumnavigation direction around the tree |

### `kmeans_tsp`

Balanced k-means over coverage targets and a 2-opt improved tour per cluster.

* **Reference:** Target-based baseline: balanced k-means partition + TSP (NN + 2-opt) per robot
* **Family / adapter:** target / `target` with planner-required parameters `{'lattice': 'square'}`
* **Mode:** blackbox
* **Kind:** stochastic (one run per seed)
* **Supports:** deconfliction, multi_robot, return_to_depot

| Hyperparameter | Type | Default | Range / choices | Meaning |
|---|---|---|---|---|
| `kmeans_iters` | int | `50` | 5 – 300 (log scale) | balanced k-means iterations |
| `balance` | float | `0.5` | 0.0 – 2.0 | strength of the cluster-size balancing |
| `two_opt_rounds` | int | `5` | 0 – 30 | 2-opt rounds per tour (0 disables 2-opt) |

### `mstc`

One STC tour over the whole region, split among the robots.

* **Reference:** Hazon & Kaminka (2005), Redundancy, efficiency and robustness in multi-robot coverage
* **Family / adapter:** grid / `grid` with planner-required parameters `{'coarse': True, 'connectivity': 4}`
* **Mode:** staged — stages `coarse_grid → stc_tour → tour_split`
* **Kind:** deterministic (planning time is repeated and the median reported)
* **Supports:** deconfliction, multi_robot

| Hyperparameter | Type | Default | Range / choices | Meaning |
|---|---|---|---|---|
| `tree` | categorical | `dfs` | dfs, bfs, horizontal, vertical | spanning-tree construction on the coarse grid |
| `direction` | categorical | `ccw` | ccw, cw | circumnavigation direction around the tree |
| `split` | categorical | `nonbacktracking` | nonbacktracking, balanced | non-backtracking (original) or balanced tour split |

### `stc`

Circumnavigates a spanning tree of 2x2 coarse cells; every sub-cell is visited once.

* **Reference:** Gabriely & Rimon (2001), Spanning-tree based coverage of continuous areas by a mobile robot
* **Family / adapter:** grid / `grid` with planner-required parameters `{'coarse': True, 'connectivity': 4}`
* **Mode:** blackbox
* **Kind:** deterministic (planning time is repeated and the median reported)
* **Supports:** return_to_depot; at most 1 robot

| Hyperparameter | Type | Default | Range / choices | Meaning |
|---|---|---|---|---|
| `tree` | categorical | `dfs` | dfs, bfs, horizontal, vertical | spanning-tree construction on the coarse grid |
| `direction` | categorical | `ccw` | ccw, cw | circumnavigation direction around the tree |

### `swath_bf`

Parallel swaths ordered greedily, split into balanced contiguous chunks, 2-opt improved.

* **Reference:** Boustrophedon swath coverage with balanced contiguous splitting (cf. Fields2Cover pipeline, Mier et al. 2023)
* **Family / adapter:** swath / `swath` with planner-required parameters `{'mode': 'parallel', 'angle': 'auto'}`
* **Mode:** staged — stages `swath_passes → nn_order → contiguous_split → two_opt`
* **Kind:** deterministic (planning time is repeated and the median reported)
* **Supports:** capacity, deconfliction, heterogeneous, multi_robot, return_to_depot, turning_radius

| Hyperparameter | Type | Default | Range / choices | Meaning |
|---|---|---|---|---|
| `service_links` | bool | `True` | true / false | keep the tool active on short straight links between neighbouring passes |
| `pattern` | categorical | `auto` | auto, nn, skip | pass order: greedy nearest neighbour, or skip pattern (auto: skip with a turning radius) |
| `two_opt` | bool | `True` | true / false | 2-opt improvement of each robot's pass order |
| `two_opt_rounds` | int | `10` | 1 – 50 | maximum number of 2-opt improvement rounds |

### Notes on behaviour

* **STC family and representation loss.** STC-type planners (`stc`, `mstc`, `darp_*`) only cover 2×2 coarse
  cells whose sub-cell centres are valid robot positions. In cluttered maps a noticeable share of the area is
  lost. The evaluator reports this loss as *representation loss* (`uncovered_representation`), not as planning
  loss, and the adapter's fidelity (ε̄) shows it before any planning.
* **DARP and start configurations.** DARP converges to a division that is optimal to within one cell when robots
  start dispersed. With clustered starts it often does not converge within `max_iter`; the best connected
  partition found is used and marked `converged: false` in the stage metrics. This is the start-configuration
  sensitivity the benchmark is designed to expose. The `metric: geodesic` hyperparameter helps in maze-like maps.
* **`swath_bf` in the agricultural profile.** With a turning radius, swaths are visited in a skip pattern, so that
  consecutive swaths are at least 2r apart, and connected with Dubins paths inside a headland. Headland contours
  are smoothed to the turning radius. Pieces the vehicle cannot drive or reach feasibly are skipped rather than
  driven infeasibly. Depot trips are inserted whenever the next piece would exceed the capacity.
* **`kmeans_tsp`** does not distinguish service from transit, so all its motion counts as service. Overlap is
  therefore counted conservatively.

## Adapters

Adapter parameters are fixed per experiment and shared by all planners of the same family. Set them in the
experiment's `adapters:` section. The transformation record of every run stores the parameters, the adapter
runtime (reported separately from planning time) and the **fidelity**:

* `rel_error` ε̄ = μ(F̂ △ F_c) / μ(F_c): how much the representation differs from the true free space;
* `missing_area` and `extra_area`: the two parts of that difference;
* `flagged`: ε̄ exceeds the experiment's `eps_max`. Planning failures on flagged runs are attributed to the
  adapter.

Inspect an adapter on one instance without running a planner:

```bash
uninbench adapt data/IF5/syn_40x30_d0.10_mixed_n0_s0.json --adapter grid --param coarse=true --param rule=full
```

### `grid`: grid and graph planners

Rasterizes the free space at the planning resolution δ_p and builds a 4- or 8-connected graph. Every edge is
checked so that the robot body stays inside the free space when moving between cell centres.

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `resolution` | `auto` | cell size in metres; `auto` = `resolution_factor` × working width |
| `resolution_factor` | 1.0 | multiplier for `auto` (the planning-resolution factor multiplies this) |
| `rule` | `center` | when a cell is free: `center` (centre in free space), `full` (completely free) or `fraction` |
| `fraction` | 0.5 | minimum free fraction for `rule: fraction` |
| `require_center_in_cspace` | true | a cell is usable only if the robot can stand at its centre |
| `connectivity` | 4 | 4 or 8 neighbours |
| `coarse` | false | form STC coarse cells of 2×2 sub-cells (set by STC-type planners) |
| `inflation` | 0.0 | shrink the free space by this many metres before rasterizing |
| `weighted` | false | edge weights from the terrain-cost field |
| `check_edges` | true | collision-check graph edges |

### `graph`: graph-native sources

Passes the original SafeLog graph (instance metadata) through unchanged. With `coarse: true` every vertex becomes
an STC coarse cell.

### `geometric`: polygon planners

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `mode` | `boustrophedon` | `polygon` (pass the polygon with holes) or `boustrophedon` (decompose it) |
| `angle` | `auto` | pass direction: `auto` (minimum width), `longest_edge`, or radians |
| `inflation` | 0.0 | shrink the free space (metres) |
| `simplify` | 0.0 | simplify the polygon (metres) |

### `swath`: swath and isoline planners

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `mode` | `parallel` | `parallel` swaths or `offset` curves (isolines) |
| `angle` | `auto` | swath direction: `auto`, `longest_edge`, or radians |
| `overlap` | 0.0 | fractional overlap between neighbouring swaths |
| `perimeter` | true | add one contour pass along every boundary and trim swath ends accordingly |
| `headland` | 0.0 | width (m) of a headland band kept free for turning (set by the agricultural profile) |
| `headland_passes` | true | cover the headland with contours smoothed to the turning radius |
| `min_length` | 0.0 | drop swaths shorter than this (m) |

### `target`: target-based planners

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `lattice` | `square` | `square` or `hex` target lattice |
| `spacing` | `auto` | target spacing; `auto` covers the plane with the tool's half-width (r√2 square, r√3 hex) |
| `boundary_samples` | true | add targets along the configuration-space boundary |

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

| Planner | Family | Provenance | Mode | Kind | Idea |
|---------|--------|------------|------|------|------|
| `crc` | geometric | original | staged | deterministic | Boustrophedon cells, Chinese-postman tour of the Reeb graph, Frederickson k-postman split. |
| `scopp` | geometric | original | staged | stochastic | Square cells, Lloyd clustering of cell perimeters, conflict auction, nearest-neighbour paths. |
| `darp_stc` | grid | original | staged | stochastic | DARP partition of coarse cells followed by STC inside every region. |
| `gm_vpc` | grid | original | staged | deterministic | Geodesic-Manhattan Voronoi partition of the 2D x 2D cells, STC inside every region. |
| `ls_mcpp` | grid | original | staged | stochastic | Local search with grow, deduplicate and exchange operators on sub-cell subgraphs; ESTC paths. |
| `mfc` | grid | original | staged | deterministic | Rooted min-max tree cover (4-approximation per bound) and STC around every tree. |
| `mip` | grid | original | staged | deterministic | Rooted min-max tree cover by mixed-integer programming, STC around every tree. |
| `mstc` | grid | original | staged | deterministic | One STC tour over the whole region, split among the robots. |
| `mstc_star` | grid | original | staged | deterministic | One Spiral-STC tour around the minimum spanning tree, cut into balanced partitions. |
| `stc` | grid | original | black-box | deterministic | Circumnavigates a spanning tree of 2x2 coarse cells; every sub-cell is visited once. |
| `tmstc_star` | grid | original | staged | deterministic | Minimum brick tiling, greedy turn-aware spanning tree, MSTC* split with turn costs. |
| `mcfs` | swath | original | staged | deterministic | Layered isolines, isograph tree cover by MIP, connected Fermat spirals per tree. |
| `bcd` | geometric | composition * | staged | deterministic | Boustrophedon cells swept back and forth; cells ordered greedily; split at pass level. |
| `darp_bf` | grid | composition * | staged | stochastic | Staged composition: DARP work distribution with back-and-forth route construction. |
| `swath_bf` | swath | composition * | staged | deterministic | Parallel swaths ordered greedily, split into balanced contiguous chunks, 2-opt improved. |
| `ga_targets` | target | baseline * | black-box | stochastic | Genetic algorithm over the visiting order and split of the coverage targets. |
| `kmeans_tsp` | target | baseline * | black-box | stochastic | Balanced k-means over coverage targets and a 2-opt improved tour per cluster. |

`*` marks results that are not those of an original published algorithm: *compositions* of stages combined by the benchmark and *baselines* written for it. They are reported with a star and never as results of an original algorithm. A planner composed with a framework stage (e.g. prioritized-waiting deconfliction) is marked `+pw*` in reports.

List them, including anything you registered yourself, with `uninbench planners` (add `--json` for full manifests) or in the *Planners* tab of either UI.

## Planner reference

Generated from the planners' manifests (`scripts/gen_planner_docs.py`). The *adapter parameters* are what the planner requires; experiment-wide parameters from `adapters:` are applied on top (see [Experiments](experiments.md#top-level-keys)).

### `bcd`

Boustrophedon cells swept back and forth; cells ordered greedily; split at pass level.

* **Reference:** Benchmark composition: boustrophedon cellular decomposition (Choset, 2000) with boundary passes, greedy cell order, contiguous multi-robot split and 2-opt
* **Provenance:** composition — boustrophedon decomposition (Choset 2000) + boundary passes + greedy cell order + contiguous split + 2-opt
* **Family / adapter:** geometric / `geometric` with planner-required parameters `{'mode': 'boustrophedon', 'angle': 'auto'}`
* **Mode:** staged — stages `cell_passes → cell_order → contiguous_split → two_opt`
* **Kind:** deterministic (planning time is repeated and the median reported)
* **Supports:** multi_robot, return_to_depot

| Hyperparameter | Type | Default | Range / choices | Meaning |
|---|---|---|---|---|
| `service_links` | bool | `True` | true / false | keep the tool active on short straight links between neighbouring passes |
| `perimeter` | bool | `True` | true / false | boundary-following passes |
| `two_opt` | bool | `False` | true / false | 2-opt improvement of each robot's pass order |
| `two_opt_rounds` | int | `5` | 1 – 50 | maximum number of 2-opt improvement rounds |

### `crc`

Boustrophedon cells, Chinese-postman tour of the Reeb graph, Frederickson k-postman split.

* **Reference:** Karapetyan, Benson, McKinney, Taslakian & Rekleitis (2017), Efficient multi-robot coverage of a known environment (Coverage with Route Clustering)
* **Provenance:** original
* **Family / adapter:** geometric / `geometric` with planner-required parameters `{'mode': 'boustrophedon', 'angle': 'auto'}`
* **Mode:** staged — stages `reeb_graph → eulerian_tour → fhk_split`
* **Kind:** deterministic (planning time is repeated and the median reported)
* **Supports:** multi_robot, return_to_depot

No hyperparameters.

### `darp_bf`

Staged composition: DARP work distribution with back-and-forth route construction.

* **Reference:** Composition: DARP partition (Kapoutsis et al., 2017) + back-and-forth sweeps
* **Provenance:** composition — darp + boustrophedon
* **Family / adapter:** grid / `grid` with planner-required parameters `{'coarse': True, 'connectivity': 4}`
* **Mode:** staged — stages `coarse_regions → darp → bf_per_region`
* **Kind:** stochastic (one run per seed)
* **Supports:** multi_robot

| Hyperparameter | Type | Default | Range / choices | Meaning |
|---|---|---|---|---|
| `max_iter` | int | `10000` | 500 – 80000 (log scale) | iterations before relaxing the threshold |
| `cc_variation` | float | `0.01` | 0.001 – 0.2 (log scale) | connectivity bias range |
| `random_level` | float | `0.0001` | 1e-06 – 0.01 (log scale) | random perturbation |
| `dcells` | int | `2` | 0 – 20 | largest accepted cell imbalance threshold |
| `sweep` | categorical | `rows` | rows, cols | back-and-forth along rows or columns of sub-cells |

### `darp_stc`

DARP partition of coarse cells followed by STC inside every region.

* **Reference:** Kapoutsis, Chatzichristofis & Kosmatopoulos (2017), DARP: Divide areas algorithm for optimal multi-robot coverage path planning
* **Provenance:** original
* **Family / adapter:** grid / `grid` with planner-required parameters `{'coarse': True, 'connectivity': 4}`
* **Mode:** staged — stages `coarse_regions → darp → stc_per_region`
* **Kind:** stochastic (one run per seed)
* **Supports:** multi_robot

| Hyperparameter | Type | Default | Range / choices | Meaning |
|---|---|---|---|---|
| `max_iter` | int | `10000` | 500 – 80000 (log scale) | iterations before relaxing the threshold |
| `cc_variation` | float | `0.01` | 0.001 – 0.2 (log scale) | connectivity bias range |
| `random_level` | float | `0.0001` | 1e-06 – 0.01 (log scale) | random perturbation |
| `dcells` | int | `2` | 0 – 20 | largest accepted cell imbalance threshold |
| `tree` | categorical | `dfs` | dfs, bfs, horizontal, vertical | spanning-tree construction on the coarse grid |
| `direction` | categorical | `ccw` | ccw, cw | circumnavigation direction around the tree |

### `ga_targets`

Genetic algorithm over the visiting order and split of the coverage targets.

* **Reference:** Benchmark baseline: genetic algorithm on coverage disks in the problem formulation of Kapanoglu, Alikalfa, Ozkan, Yazici & Parlaktuna (2012); the paper's pattern-based encoding is not reproduced
* **Provenance:** baseline — two-part chromosome GA (min-max completion time)
* **Family / adapter:** target / `target` with planner-required parameters `{'lattice': 'square'}`
* **Mode:** black-box
* **Kind:** stochastic (one run per seed)
* **Supports:** multi_robot, return_to_depot
* **Output:** does not distinguish service from transit (all motion counted as coverage)

| Hyperparameter | Type | Default | Range / choices | Meaning |
|---|---|---|---|---|
| `population` | int | `60` | 20 – 300 (log scale) | population size |
| `generations` | int | `400` | 50 – 5000 (log scale) | maximum generations |
| `crossover` | float | `0.9` | 0.5 – 1 | crossover probability |
| `mutation` | float | `0.3` | 0.01 – 0.8 | mutation probability (per operator) |

### `gm_vpc`

Geodesic-Manhattan Voronoi partition of the 2D x 2D cells, STC inside every region.

* **Reference:** Nair & Guruprasad (2020), GM-VPC: An algorithm for multi-robot coverage of known spaces using generalized Voronoi partition (with Spanning Tree Coverage)
* **Provenance:** original
* **Family / adapter:** grid / `grid` with planner-required parameters `{'coarse': True, 'connectivity': 4}`
* **Mode:** staged — stages `coarse_regions → gm_voronoi → stc_per_region`
* **Kind:** deterministic (planning time is repeated and the median reported)
* **Supports:** multi_robot

| Hyperparameter | Type | Default | Range / choices | Meaning |
|---|---|---|---|---|
| `tree` | categorical | `dfs` | dfs, bfs, horizontal, vertical | spanning-tree construction on the coarse grid |
| `direction` | categorical | `ccw` | ccw, cw | circumnavigation direction around the tree |

### `kmeans_tsp`

Balanced k-means over coverage targets and a 2-opt improved tour per cluster.

* **Reference:** Target-based baseline: balanced k-means partition + TSP (NN + 2-opt) per robot
* **Provenance:** baseline — balanced k-means + nearest-neighbour + 2-opt tour
* **Family / adapter:** target / `target` with planner-required parameters `{'lattice': 'square'}`
* **Mode:** black-box
* **Kind:** stochastic (one run per seed)
* **Supports:** multi_robot, return_to_depot
* **Output:** does not distinguish service from transit (all motion counted as coverage)

| Hyperparameter | Type | Default | Range / choices | Meaning |
|---|---|---|---|---|
| `kmeans_iters` | int | `50` | 5 – 300 (log scale) | balanced k-means iterations |
| `balance` | float | `0.5` | 0 – 2 | strength of the cluster-size balancing |
| `two_opt_rounds` | int | `5` | 0 – 30 | 2-opt rounds per tour (0 disables 2-opt) |

### `ls_mcpp`

Local search with grow, deduplicate and exchange operators on sub-cell subgraphs; ESTC paths.

* **Reference:** Tang & Ma (2024), Large-scale multi-robot coverage path planning via local search (AAAI)
* **Provenance:** original
* **Family / adapter:** grid / `grid` with planner-required parameters `{'coarse': 'partial', 'connectivity': 4}`
* **Mode:** staged — stages `decomposed_graph → mfc_initial → local_search`
* **Kind:** stochastic (one run per seed)
* **Supports:** multi_robot, return_to_depot

| Hyperparameter | Type | Default | Range / choices | Meaning |
|---|---|---|---|---|
| `max_iterations` | int | `3000` | 300 – 30000 (log scale) | M, maximum iterations |
| `dedup_step` | int | `100` | 10 – 1000 (log scale) | S, forced deduplication step |
| `pool_weight_decay` | float | `0.01` | 0.001 – 0.2 (log scale) | gamma |

### `mcfs`

Layered isolines, isograph tree cover by MIP, connected Fermat spirals per tree.

* **Reference:** Tang & Ma (2024b), Multi-robot connected Fermat spiral coverage -- MCFS and MCFS+AUG (the MMRTC solution refinement is not implemented)
* **Provenance:** original
* **Family / adapter:** swath / `swath` with planner-required parameters `{'mode': 'offset'}`
* **Mode:** staged — stages `isograph → isograph_mmrtc → unified_cfs`
* **Kind:** deterministic (planning time is repeated and the median reported)
* **Supports:** multi_robot, return_to_depot
* **External dependencies:** MILP solver (HiGHS via SciPy)

| Hyperparameter | Type | Default | Range / choices | Meaning |
|---|---|---|---|---|
| `delta` | int | `4` | 0 – 8 | isograph augmentation level (< 2: MCFS without AUG) |
| `selector` | categorical | `cfs` | cfs, random | stitching tuple selector |

### `mfc`

Rooted min-max tree cover (4-approximation per bound) and STC around every tree.

* **Reference:** Zheng, Koenig, Kempe & Jain (2010), Multirobot forest coverage for weighted and unweighted terrain (rooted tree cover of Even et al., 2004)
* **Provenance:** original
* **Family / adapter:** grid / `grid` with planner-required parameters `{'coarse': True, 'connectivity': 4}`
* **Mode:** staged — stages `terrain_graph → rooted_tree_cover → stc_per_tree`
* **Kind:** deterministic (planning time is repeated and the median reported)
* **Supports:** multi_robot, return_to_depot

No hyperparameters.

### `mip`

Rooted min-max tree cover by mixed-integer programming, STC around every tree.

* **Reference:** Tang & Ma (2023), Mixed integer programming for time-optimal multi-robot coverage path planning with efficient heuristics
* **Provenance:** original
* **Family / adapter:** grid / `grid` with planner-required parameters `{'coarse': True, 'connectivity': 4}`
* **Mode:** staged — stages `terrain_graph → mip_tree_cover → stc_per_tree`
* **Kind:** deterministic (planning time is repeated and the median reported)
* **Supports:** multi_robot, return_to_depot
* **External dependencies:** MILP solver (HiGHS via SciPy)

| Hyperparameter | Type | Default | Range / choices | Meaning |
|---|---|---|---|---|
| `heuristic` | categorical | `auto` | auto, none, prh, srh | model reduction: the paper's automatic selection, none, PRH or SRH |
| `alpha` | float | `0.6` | 0.1 – 1 | PRH parabola width (heuristic = prh) |
| `beta` | float | `0.6` | 0.1 – 1 | SRH removal size (heuristic = srh) |
| `full_model_max_vars` | int | `1200` | 500 – 20000 (log scale) | largest model solved without reduction when heuristic = auto |

### `mstc`

One STC tour over the whole region, split among the robots.

* **Reference:** Hazon & Kaminka (2005), Redundancy, efficiency and robustness in multi-robot coverage
* **Provenance:** original
* **Family / adapter:** grid / `grid` with planner-required parameters `{'coarse': True, 'connectivity': 4}`
* **Mode:** staged — stages `coarse_grid → stc_tour → tour_split`
* **Kind:** deterministic (planning time is repeated and the median reported)
* **Supports:** multi_robot

| Hyperparameter | Type | Default | Range / choices | Meaning |
|---|---|---|---|---|
| `tree` | categorical | `dfs` | dfs, bfs, horizontal, vertical | spanning-tree construction on the coarse grid |
| `direction` | categorical | `ccw` | ccw, cw | circumnavigation direction around the tree |

### `mstc_star`

One Spiral-STC tour around the minimum spanning tree, cut into balanced partitions.

* **Reference:** Tang, Sun, Zhang & Ma (2021), MSTC*: Multi-robot coverage path planning under physical constraints (Balanced-MSTC*)
* **Provenance:** original
* **Family / adapter:** grid / `grid` with planner-required parameters `{'coarse': True, 'connectivity': 4}`
* **Mode:** staged — stages `terrain_graph → mst_tour → balanced_cut`
* **Kind:** deterministic (planning time is repeated and the median reported)
* **Supports:** multi_robot, return_to_depot

No hyperparameters.

### `scopp`

Square cells, Lloyd clustering of cell perimeters, conflict auction, nearest-neighbour paths.

* **Reference:** Collins, Ghassemi, Esfahani, Doermann, Dantu & Chowdhury (2021), Scalable coverage path planning of multi-robot teams for monitoring non-convex areas
* **Provenance:** original
* **Family / adapter:** geometric / `geometric` with planner-required parameters `{'mode': 'polygon'}`
* **Mode:** staged — stages `square_cells → lloyd_auction → nn_path`
* **Kind:** stochastic (one run per seed)
* **Supports:** multi_robot, return_to_depot
* **Output:** does not distinguish service from transit (all motion counted as coverage)

| Hyperparameter | Type | Default | Range / choices | Meaning |
|---|---|---|---|---|
| `bias` | float | `0.5` | 0 – 2 | B, distance bias of the conflict auction |
| `max_iter` | int | `10` | 2 – 100 (log scale) | Lloyd iterations |
| `perimeter_points` | int | `8` | 4 – 32 | points on the perimeter of every cell |

### `stc`

Circumnavigates a spanning tree of 2x2 coarse cells; every sub-cell is visited once.

* **Reference:** Gabriely & Rimon (2001), Spanning-tree based coverage of continuous areas by a mobile robot
* **Provenance:** original
* **Family / adapter:** grid / `grid` with planner-required parameters `{'coarse': True, 'connectivity': 4}`
* **Mode:** black-box
* **Kind:** deterministic (planning time is repeated and the median reported)
* **Supports:** return_to_depot

| Hyperparameter | Type | Default | Range / choices | Meaning |
|---|---|---|---|---|
| `tree` | categorical | `dfs` | dfs, bfs, horizontal, vertical | spanning-tree construction on the coarse grid |
| `direction` | categorical | `ccw` | ccw, cw | circumnavigation direction around the tree |

### `swath_bf`

Parallel swaths ordered greedily, split into balanced contiguous chunks, 2-opt improved.

* **Reference:** Benchmark composition: parallel swaths with headland/skip patterns (cf. the Fields2Cover pipeline, Mier et al. 2023), greedy ordering, balanced contiguous split, 2-opt
* **Provenance:** composition — parallel swaths + greedy / skip order + contiguous split + 2-opt
* **Family / adapter:** swath / `swath` with planner-required parameters `{'mode': 'parallel', 'angle': 'auto'}`
* **Mode:** staged — stages `swath_passes → nn_order → contiguous_split → two_opt`
* **Kind:** deterministic (planning time is repeated and the median reported)
* **Supports:** capacity, heterogeneous, multi_robot, return_to_depot, turning_radius

| Hyperparameter | Type | Default | Range / choices | Meaning |
|---|---|---|---|---|
| `service_links` | bool | `True` | true / false | keep the tool active on short straight links between neighbouring passes |
| `pattern` | categorical | `auto` | auto, nn, skip | pass order: greedy nearest neighbour, or skip pattern (auto: skip with a turning radius) |
| `two_opt` | bool | `True` | true / false | 2-opt improvement of each robot's pass order |
| `two_opt_rounds` | int | `10` | 1 – 50 | maximum number of 2-opt improvement rounds |

### `tmstc_star`

Minimum brick tiling, greedy turn-aware spanning tree, MSTC* split with turn costs.

* **Reference:** Lu, Zeng, Tang, Lam & Wen (2023), TMSTC*: A turn-minimizing algorithm for multi-robot coverage path planning
* **Provenance:** original
* **Family / adapter:** grid / `grid` with planner-required parameters `{'coarse': True, 'connectivity': 4}`
* **Mode:** staged — stages `terrain_graph → brick_tree → turn_aware_cut`
* **Kind:** deterministic (planning time is repeated and the median reported)
* **Supports:** multi_robot, return_to_depot

No hyperparameters.

### Notes on behaviour

* **Validation against published results.** `mfc`, `mstc_star`, `mip` and `ls_mcpp` are run on the instances of
  Tang & Ma (2023) and compared with the published coverage times before they may enter an experiment (see
  [Tuning and validation](tuning-and-validation.md#validation-against-published-results)).
* **STC family and representation loss.** The tree-based planners `stc`, `mstc`, `mstc_star`, `tmstc_star`,
  `mfc`, `mip`, `darp_*` and `gm_vpc` only cover 2×2 coarse cells whose sub-cells are all free (`coarse: true`).
  In cluttered maps a noticeable share of the area is lost. The evaluator reports this loss as *representation
  loss* (`uncovered_representation`), and the adapter's fidelity (ε̄) shows it before any planning; failures on
  representations beyond `eps_max` are attributed to the adapter. `ls_mcpp` uses `coarse: partial` and covers the
  sub-cells of incomplete coarse cells with Extended-STC.
* **Objectives of the tree-cover planners.** `mfc`, `mip`, `mstc_star` and `ls_mcpp` optimise the published
  "cover and return" objective (closed tours from and to the root); the trajectories omit the return when the task
  does not require it.
* **`mip`** solves the rooted min-max tree cover with HiGHS (bundled with SciPy) under the planning time limit,
  warm-started with the MFC cover as in the paper. The paper used Gurobi with 16 threads and up to 24 h; with a
  benchmark time limit the reported MIP results are those of this solver and budget (`stage_metrics` record the
  gap to the lower bound).
* **`mcfs`** implements MCFS without and with isograph augmentation (`delta`); the MMRTC solution refinement of
  the paper is not implemented, so robots whose trees share an isoline both traverse it.
* **`crc`** assumes, as the paper does, a common start; with distinct starts the tour starts at the first robot's
  start vertex and sub-tours are assigned by a bottleneck assignment.
* **DARP and start configurations.** DARP converges to a division that is optimal to within one cell when robots
  start dispersed. With clustered starts it often does not converge within `max_iter`; the best connected
  partition found is used and marked `converged: false` in the stage metrics. This is the start-configuration
  sensitivity the benchmark is designed to expose.
* **Deconfliction.** No built-in planner models inter-robot conflicts itself. In the deconfliction profile a
  planner takes part only when the experiment sets `deconfliction_stage: prioritized_waiting`; the planner is then
  run as the composition *planner + prioritized waiting* and reported as `<planner>+pw*`.
* **`swath_bf` in the agricultural profile.** With a turning radius, swaths are visited in a skip pattern, so that
  consecutive swaths are at least 2r apart, and connected with Dubins paths inside a headland. Headland contours
  are smoothed to the turning radius. Pieces the vehicle cannot drive or reach feasibly are skipped rather than
  driven infeasibly. Depot trips are inserted whenever the next piece would exceed the capacity.
* **Planners that do not distinguish service from transit** (`kmeans_tsp`, `ga_targets`, `scopp`) have all their
  motion labelled as service by the framework; overlap is therefore counted conservatively.
* **Not implemented.** AWSTC (Dong et al., 2020, IEEE T-ASE) and the pattern-based encoding of Kapanoglu et al.
  (2012) could not be reimplemented from their publications (closed access); `ga_targets` is a clearly marked
  baseline in the problem formulation of the latter.

## Adapters

Adapter parameters are fixed per experiment and shared by all planners of the same family. Set them in the
experiment's `adapters:` section; a planner's manifest only declares the structure it needs (e.g. STC coarse
cells). Graph-native instances (SafeLog, published MCPP instances) reach grid planners through the identity
`graph` adapter (`native_graph: true`, the default). The transformation record of every run stores the parameters, the adapter
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
| `coarse` | false | `true`: STC coarse cells of 2×2 sub-cells (complete coarse cells only); `partial`: keep every traversable sub-cell and every coarse cell with at least one (Extended-STC); set by the planners |
| `inflation` | 0.0 | shrink the free space by this many metres before rasterizing |
| `weighted` | false | edge weights from the terrain-cost field |
| `check_edges` | true | collision-check graph edges |

### `graph`: graph-native sources

Passes the original graph (instance metadata of SafeLog and published MCPP instances) through unchanged. With
`coarse: true` every vertex becomes an STC coarse cell; vertex weights of weighted terrain graphs become the
coarse-cell weights.

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

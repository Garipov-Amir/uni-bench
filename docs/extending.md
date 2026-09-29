# Extending uninbench

There are three ways to benchmark your own planner, depending on how much of the pipeline you want to use:

| Approach | You provide | You get |
|----------|-------------|---------|
| **Evaluate output only** | a trajectory JSON per instance | metrics and outcomes (`uninbench evaluate`, see [Evaluation](evaluation.md#evaluating-your-own-planners-output)) |
| **Black-box planner** | a `Planner` class | everything: adapters, admission, timeouts, seeds, tuning, reports, replays |
| **Staged planner** | `Stage` classes composed in a `StagedPlanner` | as above, plus exchangeable stages and separate evaluation of intermediate outputs |

## A black-box planner

A planner is a class with a `manifest` and a `plan(rep, ctx)` method. It receives the representation produced by
its adapter and returns a `PlanResult` holding the trajectories. `examples/custom_planner.py` is a complete
example: a greedy grid planner in about 50 lines.

```python
from uninbench.planners import HyperParam, Manifest, PlanContext, Planner, PlanResult
from uninbench.planners.grid_common import cell_routes_to_trajectories
from uninbench.trajectory import SERVICE, TRANSIT


class GreedyGrid(Planner):
    manifest = Manifest(
        name="greedy_grid", version="0.1",
        reference="Example planner shipped with uninbench",
        family="grid", adapter="grid", adapter_params={"connectivity": 4},
        constraints={"multi_robot"},
        hyperparameters=[HyperParam("tie_break", "categorical", "row", choices=["row", "col"])],
    )

    def plan(self, rep, ctx: PlanContext) -> PlanResult:
        ...                                     # compute native routes: {robot_id: [(cell, label), ...]}
        ts = cell_routes_to_trajectories(rep, routes, self.manifest, ctx)
        return PlanResult(ts, native=routes)
```

### The manifest

| Field | Meaning |
|-------|---------|
| `name`, `version`, `reference`, `description` | identity; the version is part of every run id, so bump it when behaviour changes |
| `family`, `adapter`, `adapter_params` | which representation the planner needs, and the adapter parameters it requires (e.g. `coarse: True` for STC) |
| `task_class` | default `one_shot_direct_coverage`; planners of other task classes are excluded from comparisons |
| `constraints` | supported constraints: `multi_robot`, `heterogeneous`, `capacity`, `turning_radius`, `deconfliction`, `return_to_depot`, `terrain` |
| `max_robots` | e.g. `1` for single-robot planners (larger teams become *not applicable*) |
| `stochastic` | `True`: one run per seed, and your code must use `ctx.rng` (a seeded `numpy.random.Generator`) |
| `hyperparameters` | `HyperParam(name, kind, default, low=…, high=…, choices=…, log=…, doc=…)`, with kind `float`, `int`, `categorical` or `bool`; this is the space the tuner searches |
| `mode`, `stages`, `composition_of` | staged planners (below) |
| `distinguishes_service_transit` | `False` if you label everything as service |
| `tool_active_in_turns` | whether the tool stays on during in-place turns |
| `parallelism`, `external_dependencies`, `core_modules` | reported as implementation-complexity indicators; `core_modules` are paths used for the source-line count |

### What `plan` receives

* `rep`: the representation. Its type depends on the adapter:
  * `GridRepresentation` (`rep.grid`, `rep.cells`, `rep.adj` (sparse graph), `rep.coarse`, `rep.starts`,
    `rep.path(a, b)`, `rep.distances(a)`);
  * `PolygonRepresentation` (`rep.free`, `rep.cells`, `rep.angle`);
  * `SwathRepresentation` (`rep.swaths`, `rep.angle`, `rep.spacing`);
  * `TargetRepresentation` (`rep.targets`, `rep.radius`).

  All of them provide `rep.team` (the robots), `rep.cspace(robot)` (the reachable configuration space) and
  `rep.transit(robot).path(a, b)` (collision-free shortest paths).
* `ctx`: `ctx.seed`, `ctx.rng`, `ctx.time_limit`, `ctx.time_left()`, `ctx.out_of_time(fraction)` (for anytime
  algorithms), `ctx.profiles`, `ctx.return_to_depot`.

### Producing trajectories

* `TrajectoryBuilder(robot_id, x, y, psi, tool_active_in_turns=…)`, then `.move_to(x, y, "service" | "transit")`,
  `.follow(points, label)`, `.follow_curve(samples, label)`, `.turn_to(psi)` and `.wait_for(seconds)`, then
  `.build()`. Turns are inserted automatically whenever the heading changes.
* `cell_routes_to_trajectories(rep, routes, manifest, ctx)` converts grid routes. It also connects each robot's
  start pose to its start cell and handles the return to the depot.
* `uninbench.planners.sweep_common.RouteBuilder` turns ordered passes into trajectories, including
  collision-free transit, Dubins connections and depot trips.
* `PlanResult(trajectories, native=…, layers=…, info=…)`: `layers` are optional intermediate structures shown in
  the replay, for example `{"type": "lines", "coords": [...]}`.

### Using the planner

Name it in an experiment as `package.module:ClassName` (the module must be importable from the workspace, e.g. as a
file next to it or an installed package):

```yaml
planners:
  - examples.custom_planner:GreedyGrid
```

Alternatively, register it so it appears everywhere, including the UIs' Planners tab. Either call
`uninbench.planners.register(GreedyGrid)`, or declare an entry point in your package's `pyproject.toml`:

```toml
[project.entry-points."uninbench.planners"]
greedy_grid = "my_package.planners:GreedyGrid"
```

## A staged planner

Staged planners are compositions of four kinds of stages: `environment_model`, `work_distribution`,
`route_construction` and `optimization`. Each stage declares what it `consumes` and what it `produces`. Common
data types:

* `representation`: the adapter output;
* `model`: a working structure;
* `partition`: a division among robots;
* `tour`: a single route;
* `assignment`: per-robot work;
* `routes`: per-robot routes. The final stage must produce this.

Stages 2 and 3 may appear in either order. For example, MSTC builds a tour first and splits it afterwards. The
composition is type-checked before running.

Intermediate outputs can be evaluated separately. A stage's `metrics()` result is stored in the run record under
`stage_metrics`, for example the balance and convergence of a DARP partition before any route exists.

`examples/staged_planner.py` reuses the DARP stages and replaces route construction by column sweeps:

```python
class ColumnSweeps(Stage):
    kind = "route_construction"
    name = "column_sweeps"
    consumes = ("partition",)
    produces = "routes"

    def run(self, data, rep, ctx, params):
        part = data["partition"]                     # {"nodes", "labels", ...} from DARPDistribution
        ...
        return {"routes": routes}


class DARPColumns(StagedPlanner):
    manifest = Manifest(name="darp_columns", ..., mode="staged",
                        stages=["coarse_regions", "darp", "column_sweeps"],
                        composition_of=["darp", "column sweeps"], hyperparameters=DARP_PARAMS)
    stages = [CoarseRegions(), DARPDistribution(), ColumnSweeps()]

    def finish(self, data, rep, ctx):
        return PlanResult(cell_routes_to_trajectories(rep, data["routes"]["routes"], self.manifest, ctx))
```

Set `composition_of` whenever you combine stages of different methods. Such planners are marked `*` and are never
reported as the original algorithm.

## Validation cases for your planner

Add cases that reproduce worked examples or stated properties of your method, with explicit tolerances:

```python
from uninbench.validation import CASES, Expectation, ValidationCase
from uninbench.validation.cases import _rect

CASES.append(ValidationCase(
    "greedy_open_room", "greedy_grid",
    "Greedy grid covers an open 10x6 room",
    lambda: _rect(10, 6, [(0.5, 0.5, 0.0)]),
    [Expectation("coverage_ratio", 0.98, kind="ge"),
     Expectation("static_collision_time", 0.0, abs_tol=1e-9)],
))
```

Structural checks are functions `check(result, evaluation, rep) -> str | None`, passed in `checks=[...]`. They
return an error message, or `None` if the check passes.

Put the cases in the module that defines the planner. If the planner is registered through an entry point,
`uninbench validate` imports the module and picks up the cases. Otherwise, run them from Python:

```python
from uninbench.validation import validate_planner
report = validate_planner("greedy_grid", out_dir="validation", extra_cases=[my_case])
```

## Adapters and importers

* **Adapter:** subclass `uninbench.adapters.base.Adapter`.
  * Set `name`, `family` and `defaults`.
  * Implement `convert(instance) -> Representation`.
  * Set `rep.reconstruction` to a `GeometryRegion`, `RasterRegion` or `DiscUnionRegion`, which the fidelity check
    uses.
  * Register it in `uninbench.adapters.ADAPTERS`.
* **Importer:** produce `Instance` objects (see [Instances](instances.md#creating-instances-in-python)), save them
  into a directory, and call `uninbench split <dir>`. Store the source resolution of raster data in
  `Environment.source_resolution`, and put everything you vary into `metadata["factors"]`, so that reports can
  stratify and plot against it.

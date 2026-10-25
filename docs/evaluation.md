# Evaluation and metrics

The evaluator computes every metric from two inputs only: the **canonical instance** and the **trajectory set** a
planner produced. It never looks inside a planner. It brings every trajectory into a normal form and re-times it
with the instance's motion model, so planners cannot gain an advantage from their own timing assumptions:

* the robot starts with the declared start heading;
* a motion segment faces its direction of travel (a rectangular footprint is swept in that orientation);
* every heading change is a turn segment and costs `turn_cost` × |Δψ|, whether or not the planner labelled it
  (a warning lists the turns the evaluator had to insert);
* waits keep their declared durations; planner time stamps are only checked for consistency.

## Outcomes

Every run ends with exactly one outcome. All outcomes are stored; failed runs are never dropped.

| Outcome | Meaning |
|---------|---------|
| `success` | the trajectories are valid and cover at least 1 − τ of the coverable area F\* (τ = `task.tolerance`, default 0.02) |
| `incomplete` | valid, but the coverage ratio is below 1 − τ |
| `invalid` | wrong format, discontinuous, not starting at the start poses, not returning to the depot when required, colliding with obstacles, violating the turning radius or capacity (agricultural profile), or conflicting robots (deconfliction profile) |
| `timeout` | a planner call exceeded the time limit (plus `grace`) and was killed |
| `crash` | the planner or the adapter raised an error, or the worker died (e.g. out of memory) |
| `not_applicable` | the planner does not support a constraint this instance needs; not a failure, and excluded from success rates |

Failed runs carry `failure_attribution`: `adapter` when the adapter's representation was flagged by the fidelity
check (ε̄ > `eps_max`) or the adapter itself failed, otherwise `planner`. Reports list adapter-attributed failures
separately and do not use those instances to compare that planner (R3).

The failure reason is stored in the run's `reason` field, and the full traceback of a crash in `traceback`.

## The target area F\*

Not every point of the free space can be swept:

* narrow gaps may be too small for the robot's body;
* a round tool cannot reach sharp corners.

Completeness is therefore measured against the **reachable coverable set**:

```
F* = reach(F_c ⊖ B_body) ⊕ B_tool
```

Erode the free space by the body footprint, keep the components that contain a robot start, then dilate by the
tool footprint. F\* is computed once per instance and team, so all planners are judged against the same area.

## Metrics

All metrics are stored per run under `metrics` in `runs.jsonl`, and flattened in `report/runs.csv`.

| Metric | Definition | Notes |
|--------|------------|-------|
| `coverage_ratio` ρ | μ(C) / μ(F\*), where C are the cells of F\* swept at least once | success needs ρ ≥ 1 − τ |
| `uncovered_area` U | μ(F\* \ C) in m² | = representation + planning loss |
| `uncovered_representation` | uncovered area outside the adapter's reconstruction F̂ | loss caused by the representation; shared by all planners using the same adapter settings |
| `uncovered_planning` | uncovered area inside F̂ | loss caused by the planner |
| `overlap_ratio` ω | ∫ max(n(x) − 1, 0) dx / μ(F\*) | redundant sweeping; n(x) = number of distinct sweeps of cell x |
| `path_length` | total fleet travel (m) | plus `service_length`, `transit_length`, `transit_share` |
| `makespan` T_max | completion time of the last robot (s) | the primary one-shot objective |
| `sum_of_costs` | Σ_j T_j | total fleet time |
| `T_LB` | max(0, μ(F\*) − π Σ b_j²) / (2 Σ b_j v_j) | lower bound for any complete plan; b_j is the footprint radius |
| `normalized_makespan` | T_max / T_LB | dimensionless; comparable across instances of different size |
| `turns`, `heading_change` | number of turns, and Σ \|Δψ\| (rad) | heading changes closer than `turn_merge_distance` count as one turn |
| `workload_imbalance` | T_max / mean(T_j) | 1 = perfectly balanced |
| `static_collision_time` / `_length` | time and length during which a body overlaps an obstacle | any collision makes the run invalid |
| `conflict_instants` | sampled instants at which two bodies overlap | reported in the core profile, enforced in the deconfliction profile |
| `capacity_excess` | worst serviced area between depot visits, minus the capacity | agricultural profile; must be ≤ 0 |

Per-robot times, lengths and turns are stored in `per_robot`. The planning time (`planning_time`, the median over
repetitions), `peak_rss_mb`, `adapter_time` and `eval_time` are stored at the top level of each run record.

* **Planning time** is the wall-clock time of the planner call (plus any framework stage composed with it). Every
  call -- each repetition and each seed -- receives a fresh copy of the adapter output, so structures a planner
  builds lazily (transit graphs, shortest-path trees) are never reused from an earlier call, seed or planner.
* **Peak memory** (`peak_rss_mb`) is the peak resident set size of the worker process at the end of planning. Every
  run executes in a fresh worker process (`max_tasks_per_worker: 1`), so the value belongs to that run; the
  details are under `memory`. Runs that shared a process (`inline`) have no `peak_rss_mb`.

## How sweeps are counted

The workspace is rasterized at the **evaluation resolution** δ_e, by default one tenth of the smallest working
width. For every cell, the evaluator counts the *maximal time intervals* during which the cell lies inside a
robot's tool footprint while the tool is active:

* one continuous pass counts once, however many waypoints it has;
* a later return counts again;
* an inactive in-place turn separates two passes. The tool stays active during turns only if the planner declares
  it (`tool_active_in_turns`).

Rasterization introduces its own error. The evaluator therefore re-evaluates every plan at δ_e/2. If any
raster-dependent metric -- coverage, overlap, uncovered area and its representation/planning split -- changes by more
than `sensitivity_tolerance` (1 % relative; areas relative to the coverable area), it refines δ_e, at most
`max_refinements` times. All other metrics are computed from the trajectories alone and do not depend on δ_e. The
outcome of this check is stored under `sensitivity` in the run record.

## Collisions, conflicts and constraints

* **Static collisions** are checked with the *body* footprint against the true, uninflated obstacles. Numerical
  grazing below 10⁻⁴ working widths is ignored.
* **Inter-robot conflicts** are found by sampling the re-timed trajectories so that no robot moves more than δ_e
  between samples. Robots that have finished stay parked at their last position.
* **Turning radius** (agricultural profile): a path is drivable if every corner can be replaced by a circular
  fillet of radius r_min within its adjacent segments. A turn in place never satisfies this.
* **Capacity** (agricultural profile): the serviced area between two depot visits must not exceed the capacity.

## Evaluating your own planner's output

Any planner, in any language, can be evaluated. Write its plan in the **common trajectory format** and call:

```bash
uninbench evaluate path/to/instance.json my_plan.json
```

This prints the outcome, the failure reasons and all metrics as JSON.

### The trajectory format

```json
{
  "trajectories": [
    {
      "robot_id": "r0",
      "xy": [[0.5, 0.5], [9.5, 0.5], [9.5, 0.5], [9.5, 1.5]],
      "psi": [0.0, 0.0, 1.5708, 1.5708],
      "labels": ["service", "turn", "transit"],
      "active": [true, true, false]
    }
  ]
}
```

| Field | Meaning |
|-------|---------|
| `robot_id` | must match a robot of the instance; every robot needs exactly one trajectory |
| `xy` | waypoints in metres; the first one must be the robot's start position |
| `psi` | heading at every waypoint (radians); optional, derived from the motion if omitted |
| `labels` | one per segment: `service` (tool active, covers), `transit` (tool off), `turn` (in-place heading change), `wait` (stand still) |
| `active` | optional, one per segment: whether the tool is on. The default is on for `service` and off for everything else. |
| `wait` | optional, one per segment: durations of `wait` segments in seconds |
| `t` | optional time stamps; they are only checked for monotonicity, since the evaluator re-times everything |

Rules:

* Consecutive waypoints must be continuous. Turn and wait segments must not move the robot.
* Heading changes may be given as `turn` segments (recommended); otherwise the evaluator inserts them and charges
  their turn time. Reversing is not supported: a segment always faces its direction of travel.
* When the task requires it, the last waypoint must be the depot (or the start).
* Labels may also be given as a string of digits (`0` service, `1` transit, `2` turn, `3` wait), and `active` as
  a string of `0`/`1`. This is the compact form uninbench itself writes.

Planners that cannot tell service from transit should label everything `service`. Overlap is then counted
conservatively.

### From Python

```python
from uninbench.instance import Instance
from uninbench.trajectory import TrajectoryBuilder, TrajectorySet
from uninbench.evaluator import evaluate

inst = Instance.load("data/custom/pillar_hall.json")
b = TrajectoryBuilder("r0", *inst.team[0].start, tool_active_in_turns=True)
b.move_to(29, 1, "service").move_to(29, 2, "service").move_to(1, 2, "service")   # turns are inserted
ts = TrajectorySet()
ts.add(b.build())
ev = evaluate(inst, ts)
print(ev.status, ev.metrics["coverage_ratio"], ev.validity.reasons)
```

To take part in experiments, with timeouts, seeds, tuning and reports, wrap your planner as described in
[Extending uninbench](extending.md).

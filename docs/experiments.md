# Experiments

An experiment is a YAML file. It says which instances to use, which team sizes and start configurations, which
planners in which configurations, how many seeds, under which limits, and which optional profiles. Run it from the
TUI or web UI (*Experiments → Run…*) or headless:

```bash
uninbench run experiments/study.yaml [--workers 8] [--no-resume] [--output results/study-v2]
```

## A complete example

```yaml
name: study                       # used for the default output directory results/study
output: results/study
tuning_dir: results/tuning        # where frozen tuned configurations live

instances:                        # one or more instance sources
  - {path: data/IF5}              # an instance set (directory), a file or a glob
  - {path: data/IF4, split: all}  # override the split for this source
  - generate: {kind: synthetic, count: 3, sizes: [20, 40], densities: [0.05, 0.1], base_seed: 7}
    family: IF5x                  # generated on the fly (not stored under data/)
    split: all
split: test                       # default split for all sources: test | tuning | all

team_sizes: [1, 2, 4, 8]
start_configs: [clustered, dispersed]

planners:
  - stc                           # plain name: default configuration
  - {name: darp_stc, configs: [default, tuned]}
  - {name: swath_bf, params: {two_opt: false}, label: swath_no2opt}   # a custom configuration
  - examples.custom_planner:GreedyGrid                                  # external planner (module:Class)

adapters:                         # experiment-wide adapter parameters, shared by a family
  grid: {rule: center, connectivity: 4}
  swath: {angle: auto, perimeter: true}
factors:
  planning_resolution: [0.5, 1.0] # grid planners: δ_p as multiples of the working width
profiles: [core]
task: {time_limit: 120, memory_limit_mb: 8192, tolerance: 0.02}
evaluation: {resolution_factor: 0.1, sensitivity: true}

seeds: 20                         # stochastic planners: one run per seed
repetitions: 5                    # deterministic planners: timing repetitions (median reported)
workers: 8
```

## Reference

### Top-level keys

| Key | Default | Meaning |
|-----|---------|---------|
| `name` | `experiment` | experiment name, recorded with every run |
| `output` | `results/<name>` | output directory |
| `instances` | *(required)* | list of instance sources (see below) |
| `split` | `test` | default split: `test`, `tuning` or `all` |
| `team_sizes` | `[1]` | team sizes *m* |
| `start_configs` | `[clustered]` | `clustered`, `dispersed` or `given` |
| `planners` | *(required)* | list of planner specifications (see below) |
| `adapters` | `{}` | adapter parameters per adapter name (`grid`, `graph`, `geometric`, `swath`, `target`); they override the planner's defaults for every planner using that adapter |
| `native_graph` | true | grid planners receive graph-native instances (SafeLog, published MCPP instances) through the identity `graph` adapter |
| `factors.planning_resolution` | `[1.0]` | multipliers of the grid resolution (grid planners only); each value becomes its own planner variant, `name@dp0.5` |
| `profiles` | `[core]` | `core`, `agricultural`, `deconfliction`, `heterogeneous` (see [Profiles](#profiles)) |
| `deconfliction_stage` | – | `prioritized_waiting`: planners without their own deconfliction take part in the deconfliction profile as compositions `<planner>+pw*`; without it they are *not applicable* there |
| `profile_params` | `{}` | parameters per profile, e.g. `{agricultural: {capacity: 0.3}}` |
| `task` | `{}` | overrides of the instance task: `tolerance`, `time_limit` (s, per planner call), `memory_limit_mb`, `return_to_depot` |
| `evaluation` | `{}` | overrides of the evaluation settings (see below) |
| `seeds` | 20 | number of seeds for stochastic planners, taken from one fixed list shared by all planners |
| `repetitions` | 5 | timing repetitions for deterministic planners |
| `workers` | 1 | parallel worker processes |
| `eps_max` | 0.05 | adapter fidelity tolerance: runs whose relative reconstruction error exceeds it are *flagged* |
| `tuning_dir` | – | directory of frozen tuned configurations (needed for `configs: [tuned]`) |
| `require_validation` | true | planners enter only after passing validation (R9). A missing report, or one for another planner version, is produced on the spot; failing planners are listed in `excluded.json` |
| `require_published_validation` | true | for planners with published reference results (`mfc`, `mstc_star`, `mip`, `ls_mcpp`) validation includes reproducing them; needs `datasets/MIP-MCPP` (`scripts/fetch_datasets.sh`) |
| `validation_dir` | `<output>/../validation` | where validation reports are read from and written to |
| `store` | all true | `{trajectories, coverage, events}`: which artefacts to store per run (the replay needs all three) |
| `grace` | 5.0 | seconds added to the time limit before a planner call is killed |
| `max_tasks_per_worker` | 1 | runs per worker process before it is replaced; 1 makes the peak memory of every run attributable to it |
| `inline` | false | run in-process instead of in worker processes (debugging; no timeouts, no per-run peak memory) |

### Instance sources

Each entry is either a path string or a mapping:

| Key | Meaning |
|-----|---------|
| `path` | instance set directory, a single instance file, or a glob such as `data/IF1/ac_AC300_*.json` |
| `generate` | generate instances on the fly: `{kind: synthetic, count, sizes, densities, shapes, nonconvexities, base_seed, …}` or `{kind: safelog, rows, cols, densities, count, seed}` |
| `family` | override the family label (families are compared separately in reports) |
| `split` | split for this source (overrides the top-level `split`) |
| `limit` | use at most this many instances |
| `tuning_fraction`, `stratify` | how to split instances that have no split yet (generated or unsplit files); `stratify` defaults to `auto`, the factor the family varies |

### Planner specifications

| Form | Meaning |
|------|---------|
| `stc` | the default configuration θ₀ (the published defaults) |
| `{name: darp_stc, configs: [default, tuned]}` | both θ₀ and the frozen tuned θ\*; the report compares them |
| `{name: swath_bf, params: {…}}` | a custom configuration, labelled `custom` |
| `{…, label: my_variant}` | a display name, e.g. to run the same planner twice with different parameters |
| `package.module:ClassName` | a planner that is not registered, imported by path (see [Extending](extending.md)) |

Admission is checked for every instance:

* A planner whose declared task class differs from the instance's is **excluded** and listed in
  `excluded.json`.
* A planner that lacks a constraint the instance needs is recorded as **not applicable**. Examples: a
  single-robot planner with `m > 1`, or a planner without turning-radius support in the agricultural profile.
* The evaluation resolution must be finer than the planning resolution (R10); a `planning_resolution` factor that
  violates this stops the experiment with an error.

### Evaluation settings

| Key | Default | Meaning |
|-----|---------|---------|
| `resolution` | – | evaluation cell size δ_e in metres (overrides the factor) |
| `resolution_factor` | 0.1 | δ_e = factor × smallest working width |
| `sensitivity` | true | re-evaluate at δ_e/2 and refine if any raster-dependent metric (coverage, overlap, uncovered area and its representation/planning split) changes more than the tolerance |
| `sensitivity_tolerance` | 0.01 | relative tolerance of the sensitivity check |
| `max_refinements` | 1 | how often δ_e may be halved |
| `collision` | `invalidate` | a static collision makes the run invalid |
| `conflicts` | `report` | inter-robot conflicts are reported (`report`) or make the run invalid (`enforce`) |
| `turn_merge_distance` | w/2 | heading changes closer than this count as one turn |
| `metrics` | all | active metric set: only these metrics (plus coverage ratio, coverable area and T_LB) are stored |

## Factor sweeps

The framework varies one factor at a time around a baseline:

| Factor | How |
|--------|-----|
| team size | `team_sizes: [1, 2, 4, 8, 16, 32]` |
| start configuration | `start_configs: [clustered, dispersed]` |
| environment size | synthetic sizes at a constant density (`sizes: [20, 40, 80, 160]`) |
| obstacle density and shape | synthetic `densities`, `shapes`; IF3 furniture pairs |
| graph connectivity | IF4 density sweep |
| planning resolution | `factors: {planning_resolution: [0.5, 0.75, 1.0]}` |

The report plots metrics and planning time against every factor present in the data (see
[Results](results.md#scaling)).

## Profiles

Profiles add domain constraints without touching the core comparison. A planner takes part only if its manifest
declares every constraint of the profile; otherwise its runs are *not applicable*.

| Profile | Adds | Required constraints |
|---------|------|----------------------|
| `core` | nothing (the default) | – |
| `agricultural` | minimum turning radius, capacity with depot trips, rectangular implement, headlands, return to depot | `capacity`, `turning_radius` |
| `deconfliction` | inter-robot conflicts become hard constraints (planners may insert waits); with `deconfliction_stage: prioritized_waiting` planners without own deconfliction run as compositions `+pw*` | `deconfliction` (for teams of two or more) |
| `heterogeneous` | robots differ in speed (and optionally width) | `heterogeneous` |

`profile_params`:

```yaml
profile_params:
  agricultural:
    min_turn_radius: 2.0      # in working widths
    capacity: 0.25            # fraction of the coverable area serviced per depot visit
    tool_length: 0.2          # implement length in working widths
    return_to_depot: true
    headland: null            # metres; default (min_turn_radius + 1) × width
    depot: [0.0, 0.0]         # default: near the start, where a full turning circle fits
  heterogeneous:
    speed_spread: 0.5         # speeds from (1 − 0.25) v to (1 + 0.25) v
    width_spread: 0.0
```

## Outputs and resuming

A run of `experiments/study.yaml` writes to `results/study/`:

| File | Content |
|------|---------|
| `experiment.json` | the resolved configuration, planner manifests, hardware and software description |
| `runs.jsonl` | one JSON record per run, including failures (see [Results](results.md#run-records)) |
| `summary.json` | outcome counts of the last invocation |
| `excluded.json` | planners excluded from the comparison, with reasons |
| `instances/` | every derived instance (with its team) that was run, named `<id>__<hash>.json` |
| `runs/` | per-run trajectories, intermediate structures and coverage maps (for replays and figures) |

**Resuming:** every run has a deterministic `run_id`, computed from the instance hash, planner, version,
configuration, seed, adapter parameters, limits and factors. Running the same experiment again skips the runs
already recorded, so you can interrupt and restart freely, or add planners and team sizes later. `--no-resume`
discards the old records.

## Running at scale

* **Workers:** `workers:` (or `--workers`) sets the number of worker processes. Each planner call runs in a worker
  with its own timeout. A slightly smaller number than the physical core count gives the most stable timings.
* **Timings:** planning time is the wall-clock time of the planner call only, on a fresh copy of the adapter
  output. Adapter and evaluation time are recorded separately. For deterministic planners the median of
  `repetitions` calls is reported.
* **Memory limit:** `memory_limit_mb` is enforced with `RLIMIT_AS` on Linux and by polling the workers' resident
  set size on macOS. Peak memory is recorded per run (one fresh worker process per run).
* **Clusters:** split a large study into several configurations, for example one per family, with separate
  `output` directories. Run each with `uninbench run` in a batch job, then build one report per results
  directory. Pin the software with the Docker image; its tag is recorded in every run.
* **Estimate first:** *Check* in the UIs estimates the number of runs. A quick trial with `limit: 2` on each
  instance source shows the time per run.

## Example configurations

`examples/experiments/` contains ready-made configurations:

| File | Purpose |
|------|---------|
| `quick.yaml` | small end-to-end experiment on generated maps |
| `core.yaml` | the core benchmark over all families with default and tuned configurations |
| `resolution.yaml` | planning-resolution sensitivity of grid planners |
| `scaling.yaml` | environment size × team size |
| `connectivity.yaml` | IF4 graph density sweep with clustered and dispersed starts |
| `furniture.yaml` | IF3 paired empty/furnished rooms |
| `agricultural.yaml` | agricultural profile on IF2 fields |
| `deconfliction.yaml` | conflicts as hard constraints |

Copy one into your workspace's `experiments/` and adjust the paths.

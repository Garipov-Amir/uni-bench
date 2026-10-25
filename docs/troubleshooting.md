# Troubleshooting and FAQ

### Almost every run of STC, MSTC or DARP is "incomplete". Is something broken?

Usually not. STC-type planners cover 2 × 2 coarse cells, each twice the working width, whose sub-cell centres are
valid robot positions. In cluttered maps, much of the area near obstacles cannot be represented that way. The
evaluator reports this share as `uncovered_representation`, and the adapter's fidelity ε̄ in the report shows it
before any planning happens. To see these planners at their best, use grid-like environments such as the IF4
graphs or the IF3 rooms.

You can also:

* raise the tolerance τ (`task: {tolerance: 0.05}`) when you are interested in quality, not completeness;
* vary the planning resolution (`factors: {planning_resolution: [0.5, 1.0]}`) to study the effect.

### A run is "invalid". How do I find out why?

The `reason` column (web: *Results → Runs*; CSV: `report/runs.csv`) says why: a static collision, a discontinuous
trajectory, a turning-radius or capacity violation, or conflicts in the deconfliction profile. To see where it
happens, build a replay for the instance and toggle the reconstruction and intermediate layers, or draw it with
`uninbench figure results/<exp> <run_id>`.

### DARP runs take long, or report `converged: false`

DARP iterates until the regions are balanced and connected. With clustered starts, or in maze-like graphs, it
often cannot reach that within `max_iter` and relaxes its threshold several times. Things that help:

* a smaller `max_iter`;
* `gm_vpc`, whose geodesic Voronoi partition needs no iterations (a different, published method);
* tuning.

A planner call that exceeds `task.time_limit` is recorded as a `timeout`.

### Jobs show state `lost`

The job process disappeared without writing its final status, for example after a reboot or when the process was
killed from outside. The results written so far are kept. Start the experiment again to resume.

### I closed the TUI or stopped the web server. Did my run stop?

No. Jobs run independently of the interface. Open the TUI (*Jobs* tab) or the web UI (*Jobs* page) to follow or
cancel them.

### "Could not start the web UI" / the page does not load

* Another program uses port 8350. Use `uninbench web --port 8400`. From the TUI, `w` tries ports 8350–8369
  automatically.
* A remote machine: use an SSH tunnel (see [The web UI](web.md#remote-access)). With `--host`, you must open the
  page with exactly that host name or IP; other names are rejected with *forbidden host*.
* *missing or invalid session token*: the server was restarted. Reload the page.

### Opening a report from the TUI does nothing

On machines without a desktop (for example over SSH), there is no browser to open. The notification shows the file
path. Copy the file to your computer, or use the web UI through an SSH tunnel.

### Planning-time measurements look noisy

* Use fewer workers than physical cores (`workers:`).
* Avoid running other heavy jobs at the same time.
* Keep `repetitions: 5` for deterministic planners; their reported time is the median.

Compare planning times only within one experiment, since the hardware description is recorded per run.

### Memory limit and peak memory

`memory_limit_mb` is enforced with `RLIMIT_AS` on Linux. On macOS, which ignores `RLIMIT_AS`, the worker pool polls
the resident set size of busy workers and stops a run that exceeds the limit (outcome `crash`, reason
*memory limit exceeded*). Peak memory (`peak_rss_mb`) is measured per run because every run executes in a fresh
worker process; with `inline: true` or `max_tasks_per_worker` > 1 it is left empty.

### A planner is excluded with "published validation data not available"

MFC, MSTC\*, MIP and LS-MCPP must reproduce published results before they may enter an experiment. Run
`scripts/fetch_datasets.sh` (it clones the published instances to `datasets/MIP-MCPP`), or set
`UNINBENCH_MIP_MCPP`. For exploratory runs, `require_published_validation: false` admits them anyway.

### Worker processes cannot start (sockets not permitted)

Workers are forked from a preloaded fork server where available. If the fork server cannot be started (for example
in a sandbox that forbids local sockets), the pool falls back to spawning processes, which only makes starting a
run slower.

### Matplotlib or fontconfig warnings about cache directories

These appear when your home directory is not writable, for example in sandboxes and some containers. Set
`MPLCONFIGDIR` and `XDG_CACHE_HOME` to a writable directory. The warnings are harmless.

### How do I start over?

* Runs: `uninbench run experiments/x.yaml --no-resume`, or delete `results/x/`.
* Jobs: *Clear finished* in the Jobs tab. This never deletes results.
* Tuned configurations: delete `results/tuning/<planner>__<family>.json`.

### Can two experiments run at the same time?

Yes, as long as they have different `output` directories. The same experiment must not be run twice concurrently,
because both runs would append to the same `runs.jsonl`.

### Where is the exact command a UI ran?

At the top of the job's log: `$ uninbench …`. Copy it to reproduce the job headless.

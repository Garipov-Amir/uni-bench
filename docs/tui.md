# The terminal UI

```bash
uninbench                      # in a workspace directory
uninbench tui -w ~/coverage-study
```

The TUI runs in any modern terminal, including over SSH. A window of at least 120 × 36 characters works best. The
mouse works everywhere: click tabs, buttons, rows and fields. Everything is also reachable from the keyboard.

## Layout and keys

```
┌ uninbench — /path/to/workspace ─────────────────────────────── 14:02:11 ┐
│ Dashboard  Instances  Planners  Experiments  Jobs  Results              │  ← tabs
│                                                                         │
│   … content of the active tab …                                         │
│                                                                         │
│ 1 Dashboard 2 Instances 3 Planners 4 Experiments 5 Jobs 6 Results       │  ← key help
│ r Refresh  w Web UI  q Quit                                  ^p palette  │
└─────────────────────────────────────────────────────────────────────────┘
```

| Key | Action |
|-----|--------|
| `1` … `6` | switch to Dashboard, Instances, Planners, Experiments, Jobs, Results |
| `Tab` / `Shift+Tab` | move the focus between widgets |
| `↑` `↓` `PgUp` `PgDn` | move within tables and lists (the details follow the highlighted row) |
| `Enter` | activate the focused button; open the highlighted job from the Dashboard |
| `r` | reload the active tab from disk |
| `w` | start the web UI for this workspace and open it in your browser |
| `i` | initialize the workspace (when the Dashboard says it is not initialized) |
| `Ctrl+S` | save the experiment in the editor (Experiments tab) |
| `Esc` | close a dialog without doing anything |
| `Ctrl+P` | command palette (search for commands, change the theme) |
| `q` | quit; running jobs continue |

When a background job finishes, a notification appears in the corner. It is green for `done`, yellow for
`cancelled` and red for `failed`.

## Dashboard (1)

* **Summary:** the workspace path and the number of instance sets, instances, experiments, result sets and running
  jobs.
* **Quick actions:**
  * **New experiment** opens a template in the Experiments editor.
  * **Generate instances** and **Import dataset** open the corresponding dialogs.
  * **Validate planners** runs the validation cases of every planner.
  * **Run demo** starts the end-to-end demonstration.
  * **Open web UI** does the same as pressing `w`.
* **Recent jobs:** the ten newest jobs with their state and progress. Select a row and press `Enter` to follow it
  in the Jobs tab.

## Instances (2)

* **Instance sets** (left): the directories under `data/`, with the number of instances and how many belong to
  each split. Highlight a set to list its instances.
* **Instance table** (right): id, family, split, number of holes, robots, working width and the instance's
  *factors* (the parameters it was generated with, used for stratification and scaling plots).
* **Instance detail** (below the table): a character map of the highlighted instance, where `█` is an obstacle or
  the outside, `·` is free space and `●` is a robot start. Next to it are the key numbers: free area, coverable
  area F\*, the makespan lower bound T_LB and the evaluation resolution δ_e.
* **Generate…** opens the generator dialog, with two generators:
  * *Synthetic environments (IF5)* uses sizes (widths in m), obstacle densities, obstacle shapes and boundary
    notches. The count is per combination of these factors.
  * *SafeLog-like graphs (IF4)* uses rows × columns and edge densities. Leave the densities empty for a 21-step
    sweep from a spanning tree to the full grid.

  Both take a seed, the tuning fraction and the factor to stratify the split by.
* **Import…** imports a downloaded dataset: `areacoverage`, `fields`, `rooms` or `safelog`. Enter the source
  directory, the output set and optional importer parameters such as `width=3.0`. See
  [Instances and datasets](instances.md).
* **Re-split set…** reassigns the tuning/test split of the highlighted set, for example after adding instances.

## Planners (3)

The table lists every registered planner with its family, mode (black-box or staged), whether it is stochastic,
and its latest validation result. Compositions are marked with `*`.

The detail pane shows the description, reference, adapter, supported constraints, pipeline stages, and a table of
hyperparameters with defaults, ranges and meaning.

* **Validate selected** runs the validation cases of the highlighted planner.
* **Validate all** runs them for every planner. Results appear in the *Validation* column when the job finishes.

## Experiments (4)

The left side lists the configurations in `experiments/`. A `● results` marker means the experiment already has
results. The right side is a YAML editor.

| Button | What it does |
|--------|--------------|
| **New experiment** | loads a commented template into the editor |
| **Save** (`Ctrl+S`) | writes `experiments/<name>.yaml`; the name is the field left of `.yaml` |
| **Check** | parses the configuration and lists errors (unknown planners, missing instance sets, YAML syntax) and warnings (existing results, missing tuned configurations). If valid, it estimates the number of runs. |
| **Tune…** | saves, then starts Bayesian tuning on the tuning split: budget per planner and family, seeds per evaluation |
| **Run…** | checks and saves, then starts the experiment. Choose the worker processes (empty = from the configuration) and whether to *resume*. |

The full configuration reference is in [Experiments](experiments.md).

## Jobs (5)

* The **table** lists all jobs, newest first: state, progress, start time, elapsed time and the last line of
  output.
* The **header** of the highlighted job shows its command line, exit code and the files it produced (reports and
  replays).
* The **progress bar** counts completed runs.
* The **log** follows the job's output live.

Buttons:

* **Cancel** appears for running jobs.
* **Remove** appears for finished jobs.
* **Clear finished** removes all finished jobs, after asking for confirmation.
* **Open result** opens the report or replay produced by the job in your browser.

## Results (6)

The left side lists the result sets in `results/`, with the number of runs and outcomes. For the highlighted set:

* The **outcome counts** across all runs.
* **Success rates** per family, team size (m), configuration and planner, with the count of each failure kind.
  *n/a* counts combinations the planner does not support; they are excluded from the success-rate denominator.
* **Normalized makespan T/T_LB**: median, IQR and 95 % bootstrap confidence interval over instances, computed on
  successful runs only.
* **Build report** creates `results/<name>/report/index.html`. **Open report** opens it in your browser.
* **Replay…** asks for an instance and builds a side-by-side replay. **Open latest replay** opens the newest one.

How to read the report: [Results, reports and replays](results.md).

## Tips

* If a browser cannot be opened (for example on a remote machine), the notification shows the file path. Copy the
  file, or press `w` and open the web UI through an SSH tunnel (see [The web UI](web.md#remote-access)).
* The TUI only *starts* jobs, so heavy experiments are safe to launch from a laptop and follow later from any
  interface.
* Themes and other settings are in the command palette (`Ctrl+P`).

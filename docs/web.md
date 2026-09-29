# The web UI

```bash
uninbench web                         # serve the current workspace on http://localhost:8350/
uninbench web -w ~/coverage-study --port 8400 --open
```

`--open` also opens your browser. From the TUI, press `w` to start the server in the background and open it.
Stop a standalone server with `Ctrl+C`; jobs it started keep running.

The server uses only the Python standard library, and the page needs no internet access.

## Pages

The left sidebar switches between six pages. The page address (for example `#/results/results%2Fquick`) can be
bookmarked and reloaded.

### Dashboard

* Counts of instance sets, instances, experiments and result sets, and the number of running jobs. Click a count to
  open its page.
* If the folder is not yet a workspace, a banner offers **Initialize workspace**.
* **Quick actions:** New experiment, Generate instances, Import dataset, Validate planners, Run the demo.
* **Recent jobs**, with live progress bars.

### Instances

* The left column lists the instance sets. **Generate…**, **Import…** and **Re-split…** open dialogs with the
  same options as in the [TUI](tui.md#instances-2).
* The table lists the instances of the selected set. Type in the filter box to narrow it by id, family, split or
  factor.
* Clicking an instance shows a drawing and the key numbers:
  * grey: obstacles;
  * white: free space;
  * blue tint: the coverable set F\*;
  * dots: robot starts.

### Planners

Each row is a planner, showing its family, adapter, mode, kind, supported constraints and validation status. Click
a row to expand the description, reference, pipeline stages, adapter parameters and the hyperparameter table.
**Validate all** and the per-planner **Validate** buttons start validation jobs.

### Experiments

* The left column lists the configurations. **New experiment** loads a template.
* The editor shows the YAML file; `Tab` inserts two spaces. The file name is the field above the editor.
* The **Quick builder** (click to expand) sets the common fields with checkboxes and inputs: instance sets,
  planners, team sizes, start configurations, split, seeds, repetitions, workers, time limit, tolerance, profiles
  and configurations. **Apply to YAML** merges them into the editor. Everything else stays as you wrote it.
* The buttons:
  * **Save** writes the file.
  * **Check** lists errors and warnings and estimates the number of runs.
  * **Tune…** starts tuning.
  * **Run…** checks, saves, asks for workers and resume, and starts the run.

### Jobs

* The table lists all jobs.
* Selecting a job shows its command, state, progress and full log. The log follows new output while you are
  scrolled to the bottom.
* **Cancel** stops a running job. **Remove** deletes a finished job's record.
* **Open report** and **Open replay** buttons appear for jobs that produced those files.
* **Clear finished** removes all finished jobs.

### Results

* The left column lists the result sets, with a bar showing the share of each outcome.
* For the selected set:
  * the outcome counts;
  * **Open report**, and **Build report** or **Rebuild report**;
  * **Replay**: pick an instance and **Build replay**; links to existing replays;
  * **Success rates** per family, team size, configuration and planner;
  * **Normalized makespan T/T_LB**: median, IQR and 95 % CI over instances (successful runs);
  * **Runs**: individual runs, filtered by planner and outcome, showing coverage, T/T_LB, overlap, planning time
    and the failure reason.

Reports, replays and figures open in a new browser tab. They are self-contained HTML files served from the
workspace.

## Security

The web UI can start processes, so it is locked down by default:

* It **binds to 127.0.0.1**, so only programs on the same machine can connect.
* It checks the **Host** header against the address it serves, which blocks DNS-rebinding attacks from web
  pages.
* Every **state-changing request** (starting, cancelling or removing jobs, saving files) needs a per-session
  token. The token is embedded in the page the server delivers and is regenerated at every start.
* It **only reads and writes inside the workspace**; path traversal is refused. It never deletes results.

## Remote access

To use the web UI of a workspace on a remote server, use an SSH tunnel:

```bash
# on your laptop
ssh -L 8350:localhost:8350 user@server
# on the server (inside the SSH session)
cd ~/coverage-study && uninbench web
```

Then open `http://localhost:8350/` on your laptop.

Binding to another interface (`--host 192.168.1.20`) is possible on trusted networks. The server accepts only
requests addressed to exactly that host name or IP. Anyone who can reach the port can browse the workspace and
start jobs, so prefer the tunnel.

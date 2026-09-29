# Getting started

## Install

uninbench needs Python 3.10 or newer.

```bash
git clone <your copy of the repository> unin-bench && cd unin-bench
python3 -m venv .venv
.venv/bin/pip install -e ".[test]"
```

This installs the `uninbench` command into `.venv/bin`. Either activate the environment
(`source .venv/bin/activate`) or call `.venv/bin/uninbench` directly.

To check the installation, validate the built-in planners:

```bash
uninbench validate
```

Every planner should print `PASS`.

If you prefer containers, the `Dockerfile` builds a versioned environment. The image tag is recorded with every
run:

```bash
docker build -t uninbench:0.1.0 .
docker run --rm -v "$PWD:/work" uninbench:0.1.0 run experiments/quick.yaml   # inside a workspace
```

## Create a workspace

A workspace is an ordinary directory that holds your instances, experiment configurations, results and jobs
(see [Workspaces and jobs](workspace.md)).

```bash
mkdir ~/coverage-study && cd ~/coverage-study
uninbench init
```

`init` creates `data/`, `experiments/`, `results/`, `validation/`, `datasets/` and an example configuration,
`experiments/quick.yaml`.

## Your first experiment in the terminal UI

Start the TUI in the workspace:

```bash
uninbench
```

Then work through these steps:

1. Press **4** for the *Experiments* tab. `quick` is selected. Press **Check**: the line below the toolbar estimates
   how many runs the configuration produces.
2. Press **Run…** and then **Start run**. The TUI switches to the *Jobs* tab and follows the job. Its progress bar
   counts completed runs, and the log shows each run's outcome as it finishes.
3. When the job is done, press **6** for *Results*. Select `quick` to see the success rates and the normalized
   makespan per planner. Then:
   * **Build report** creates the statistical report. **Open report** opens it in your browser.
   * **Replay…** builds a side-by-side animation of all planners on one instance. **Open latest replay** shows it.

Jobs keep running if you quit the TUI (**q**). Start it again later and they appear in *Jobs*.

## The same in the web UI

```bash
uninbench web --open
```

This opens `http://localhost:8350/`. The *Experiments* page has the same **Check**, **Run…** and **Tune…**
buttons, and the *Results* page links to reports and replays. See [The web UI](web.md).

## The same without a UI

```bash
uninbench run experiments/quick.yaml
uninbench report results/quick                         # → results/quick/report/index.html
uninbench replay results/quick                         # → results/quick/replay/<instance>.html
```

## A one-command demonstration

```bash
uninbench demo --out demo
```

The demo generates synthetic maps and small SafeLog-like graphs, runs all seven planners with 1, 2 and 4 robots,
and writes a report and a replay into `demo/results/`.

## Where to go next

* Generate or import instances, including the public AreaCoverage, Fields2Cover and indoor-room datasets:
  [Instances and datasets](instances.md).
* Write your own experiment: [Experiments](experiments.md).
* Understand what the numbers mean: [Evaluation and metrics](evaluation.md) and [Results](results.md).
* Tune planners fairly before comparing them: [Tuning and validation](tuning-and-validation.md).

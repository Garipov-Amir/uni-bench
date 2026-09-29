# Workspaces and jobs

## Layout

The terminal UI, the web UI and the headless commands all work on a **workspace**: the directory you start them in,
or the one given with `-w/--workspace`.

```
workspace/
├── data/                 instance sets, one directory per set
│   └── IF5/              *.json instances + index.json (tuning/test split)
├── experiments/          experiment configurations (*.yaml)
├── results/              one directory per experiment (see results.md)
│   └── tuning/           frozen tuned configurations (<planner>__<family>.json)
├── validation/           planner validation reports (<planner>.json)
├── datasets/             raw downloads (AreaCoverage, room maps, fields)
└── .uninbench/jobs/      background jobs started from a UI
```

`uninbench init [DIR]` creates this layout. It is idempotent, so it is safe to run on an existing workspace. If
you open a UI on an empty directory, the Dashboard offers to initialize it (key **i** in the TUI, a button in the
web UI).

Paths inside configurations are relative to the workspace root. Examples are `data/IF5`, `results/quick` and
`results/tuning`. The UIs refuse to read or write anything outside the workspace.

## Background jobs

Anything that can take more than a moment runs as a **job**:

* experiments;
* tuning;
* validation;
* reports and replays;
* generation, import and re-splitting of instances;
* the evaluator check;
* the demo.

A job is simply the corresponding headless command (for example `uninbench run experiments/quick.yaml`) started in
the background. Therefore:

* **Jobs survive the interface.** Closing the TUI or stopping the web server does not stop them.
* **Every interface sees every job.** A job started in the web UI appears in the TUI's *Jobs* tab, and vice versa.
* **Jobs are reproducible from the command line.** The Jobs view shows the exact command (`$ uninbench …`) at the
  top of each log.

Each job has a directory `.uninbench/jobs/<id>/` with three files:

| File | Content |
|------|---------|
| `job.json` | kind, title, command line, working directory, creation time |
| `status.json` | state, process ids, exit code, start and end times |
| `output.log` | everything the command printed |

Job states:

| State | Meaning |
|-------|---------|
| `queued` | the job was created and its process is starting |
| `running` | the command is executing |
| `done` | the command exited with code 0 |
| `failed` | the command exited with a non-zero code; see the log |
| `cancelled` | you cancelled it; the whole process group was terminated |
| `lost` | the job process disappeared without writing a final status (for example, the machine was rebooted) |

**Progress** is read from the log. Experiment runs print one line per finished run (`[k/n] planner instance
outcome …`), and tuning prints `eval k/budget`.

**Cancel** stops the job, including its worker processes. Finished runs of an experiment remain in `runs.jsonl`,
and running the experiment again resumes where it stopped.

**Remove** deletes a finished job's record and log. **Clear finished** removes all of them. Results written by
the job, such as `results/…` or `data/…`, are never deleted by these buttons.

## Several people or machines

A workspace is plain files, so you can put it on a shared file system and inspect it from several machines. Two
limitations apply:

* Jobs are local processes. Cancelling only works on the machine that started the job.
* Two runs of the *same* experiment must not execute at the same time, because both would append to the same
  `runs.jsonl`.

For clusters, use the headless commands directly (see [Experiments](experiments.md#running-at-scale)).

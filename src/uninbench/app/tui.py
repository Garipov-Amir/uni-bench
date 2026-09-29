"""Terminal user interface (Textual): ``uninbench`` / ``uninbench tui``.

Six tabs mirror the workflow -- Dashboard, Instances, Planners, Experiments,
Jobs, Results.  Long operations run as background jobs (shared with the web UI),
so the interface never blocks and jobs survive closing it.
"""

from __future__ import annotations

import time
import webbrowser
from pathlib import Path

from rich.table import Table
from rich.text import Text
from textual import on, work
from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.containers import Grid, Horizontal, Vertical, VerticalScroll
from textual.screen import ModalScreen
from textual.widgets import (Button, Checkbox, DataTable, Footer, Header, Input, Label, Log, OptionList,
                             ProgressBar, Select, Static, TabbedContent, TabPane, TextArea)
from textual.widgets.option_list import Option

from .. import __version__
from .actions import DATASETS, Actions
from .jobs import JobManager
from .workspace import Workspace

ACTIVE = ("queued", "running")
STATE_STYLE = {"done": "green", "running": "cyan", "queued": "dim", "failed": "red", "cancelled": "yellow",
               "lost": "red", "success": "green", "incomplete": "yellow", "invalid": "red", "timeout": "yellow",
               "crash": "red", "not_applicable": "dim"}


def fmt(v, nd: int = 3) -> str:
    if v is None or v == "":
        return "–"
    if isinstance(v, float):
        if v != v:
            return "–"
        if abs(v) >= 1000:
            return f"{v:.0f}"
        return f"{v:.{nd}g}"
    if isinstance(v, dict):
        return " ".join(f"{k}={fmt(x)}" for k, x in v.items())
    if isinstance(v, (list, tuple)):
        return ", ".join(fmt(x) for x in v)
    return str(v)


def state_text(s: str) -> Text:
    return Text(str(s).replace("_", " "), style=STATE_STYLE.get(s, ""))


def dur(s) -> str:
    if s is None:
        return "–"
    if s < 60:
        return f"{s:.0f} s"
    if s < 3600:
        return f"{int(s // 60)} min {int(s % 60)} s"
    return f"{int(s // 3600)} h {int(s % 3600 // 60)} min"


def ascii_map(detail: dict, cols: int = 56) -> Text:
    """Character rendering of an instance (free space, obstacles, robot starts)."""
    from shapely.geometry import Polygon

    x0, y0, x1, y1 = detail["bounds"]
    cw = max((x1 - x0) / cols, 1e-9)
    ch = 2 * cw  # terminal cells are about twice as tall as wide
    rows = max(1, int((y1 - y0) / ch + 0.5))
    polys = []
    for poly in detail["free_geometry"]:
        polys.append(Polygon(poly[0], poly[1:]))
    import shapely

    xs = [x0 + (j + 0.5) * cw for j in range(cols)]
    starts = {}
    for k, r in enumerate(detail["team"]):
        j = min(cols - 1, max(0, int((r["start"][0] - x0) / cw)))
        i = min(rows - 1, max(0, int((y1 - r["start"][1]) / ch)))
        starts[(i, j)] = k
    text = Text()
    union = shapely.unary_union(polys) if polys else None
    for i in range(rows):
        y = y1 - (i + 0.5) * ch
        inside = shapely.contains_xy(union, xs, [y] * cols) if union is not None else [False] * cols
        for j in range(cols):
            if (i, j) in starts:
                text.append("●", style=["bold cyan", "bold magenta", "bold green", "bold yellow"][starts[(i, j)] % 4])
            elif inside[j]:
                text.append("·", style="dim")
            else:
                text.append("█", style="grey50")
        text.append("\n")
    return text


# ---------------------------------------------------------------------- modals
class FormModal(ModalScreen):
    """A small form; dismisses with a dict of values or None.  Long forms use two columns
    and the buttons stay visible below the (scrollable) fields."""

    DEFAULT_CSS = """
    FormModal { align: center middle; }
    #form { width: 96; height: auto; max-height: 92%; background: $surface; border: round $accent; padding: 1 2; }
    #form .help { color: $text-muted; margin-top: 1; }
    #fields { height: auto; max-height: 30; }
    #fields Grid { grid-size: 2; grid-gutter: 0 2; height: auto; }
    #fields Vertical { height: auto; }
    #fields Label { margin-top: 1; color: $text-muted; }
    #form .buttons { height: auto; margin-top: 1; align-horizontal: right; }
    #form .buttons Button { margin-left: 1; }
    """
    BINDINGS = [Binding("escape", "cancel", "Cancel")]

    def __init__(self, title: str, fields: list[dict], submit: str = "Start", help: str = ""):
        super().__init__()
        self.title_text, self.fields, self.submit, self.help = title, fields, submit, help

    def _field(self, f: dict):
        if f.get("type") == "checkbox":
            yield Checkbox(f["label"], value=bool(f.get("value")), id=f"f-{f['name']}")
            return
        yield Label(f["label"])
        if f.get("type") == "select":
            yield Select([(str(lab), val) for val, lab in f["options"]], value=f.get("value"), allow_blank=False,
                         id=f"f-{f['name']}")
        else:
            yield Input(str(f.get("value", "")), placeholder=f.get("placeholder", ""), id=f"f-{f['name']}")

    def compose(self) -> ComposeResult:
        with Vertical(id="form"):
            yield Label(f"[b]{self.title_text}[/b]")
            if self.help:
                yield Static(self.help, classes="help")
            with VerticalScroll(id="fields"):
                if len(self.fields) > 5:
                    with Grid():
                        for f in self.fields:
                            with Vertical():
                                yield from self._field(f)
                else:
                    for f in self.fields:
                        yield from self._field(f)
            with Horizontal(classes="buttons"):
                yield Button("Cancel", id="cancel")
                yield Button(self.submit, variant="primary", id="submit")

    def values(self) -> dict:
        out = {}
        for f in self.fields:
            w = self.query_one(f"#f-{f['name']}")
            out[f["name"]] = w.value
        return out

    @on(Button.Pressed, "#submit")
    def _submit(self):
        self.dismiss(self.values())

    @on(Input.Submitted)
    def _enter(self):
        self.dismiss(self.values())

    @on(Button.Pressed, "#cancel")
    def action_cancel(self):
        self.dismiss(None)


class ConfirmModal(ModalScreen):
    DEFAULT_CSS = """
    ConfirmModal { align: center middle; }
    #confirm { width: 60; height: auto; background: $surface; border: round $warning; padding: 1 2; }
    #confirm Horizontal { height: auto; margin-top: 1; align-horizontal: right; }
    #confirm Button { margin-left: 1; }
    """

    def __init__(self, question: str):
        super().__init__()
        self.question = question

    def compose(self) -> ComposeResult:
        with Vertical(id="confirm"):
            yield Static(self.question)
            with Horizontal():
                yield Button("No", id="no")
                yield Button("Yes", variant="warning", id="yes")

    @on(Button.Pressed)
    def _answer(self, event: Button.Pressed):
        self.dismiss(event.button.id == "yes")


# ---------------------------------------------------------------------- panes
class Pane(Vertical):
    """Base for tab contents: access to the shared workspace, jobs and actions."""

    @property
    def ws(self) -> Workspace:
        return self.app.ws

    @property
    def jobs(self) -> JobManager:
        return self.app.jobs

    @property
    def actions(self) -> Actions:
        return self.app.actions

    def refresh_data(self):  # overridden
        pass


class DashboardPane(Pane):
    def compose(self) -> ComposeResult:
        yield Static(id="dash-banner")
        yield Static(id="dash-summary")
        with Horizontal(classes="toolbar"):
            yield Button("New experiment", variant="primary", id="dash-new")
            yield Button("Generate instances", id="dash-generate")
            yield Button("Import dataset", id="dash-import")
            yield Button("Validate planners", id="dash-validate")
            yield Button("Run demo", id="dash-demo")
            yield Button("Open web UI", id="dash-web")
        yield Label("[b]Recent jobs[/b]  (Enter opens the job)", classes="section")
        yield DataTable(id="dash-jobs", cursor_type="row", zebra_stripes=True)

    def on_mount(self):
        self.query_one("#dash-jobs", DataTable).add_columns("Job", "Kind", "State", "Progress", "Started", "Elapsed")
        self.refresh_data()

    def refresh_data(self):
        s = self.ws.summary()
        banner = self.query_one("#dash-banner", Static)
        if not s["initialized"]:
            banner.update(Text.from_markup(
                "[b yellow]This folder is not a uninbench workspace yet.[/] Press [b]i[/b] to initialize it "
                "(creates data/, experiments/, results/ and an example experiment)."))
            banner.display = True
        else:
            banner.display = False
        t = Table.grid(padding=(0, 3))
        for _ in range(6):
            t.add_column()
        running = len(self.jobs.running())
        t.add_row(*[Text(str(v), style="bold") for v in (s["instance_sets"], s["instances"], s["experiments"],
                                                         s["results"], running)])
        t.add_row(*[Text(v, style="dim") for v in ("instance sets", "instances", "experiments", "result sets",
                                                   "jobs running")])
        head = Text.from_markup(f"[b]Workspace[/b] {s['root']}   [dim]uninbench {__version__}[/]\n")
        from rich.console import Group

        self.query_one("#dash-summary", Static).update(Group(head, t))
        tbl = self.query_one("#dash-jobs", DataTable)
        tbl.clear()
        for j in self.jobs.list(limit=10):
            p = j.get("progress")
            tbl.add_row(j["title"], j["kind"], state_text(j["state"]),
                        f"{p['done']}/{p['total']}" if p else "–",
                        time.strftime("%H:%M:%S", time.localtime(j.get("started") or j["created"])),
                        dur(j.get("elapsed")), key=j["id"])

    @on(DataTable.RowSelected, "#dash-jobs")
    def _open_job(self, event: DataTable.RowSelected):
        self.app.show_job(event.row_key.value)

    @on(Button.Pressed, "#dash-new")
    def _new(self):
        self.app.action_show_tab("experiments")
        self.app.query_one(ExperimentsPane).new_experiment()

    @on(Button.Pressed, "#dash-generate")
    def _gen(self):
        self.app.generate_dialog()

    @on(Button.Pressed, "#dash-import")
    def _imp(self):
        self.app.import_dialog()

    @on(Button.Pressed, "#dash-validate")
    def _val(self):
        self.app.start(lambda: self.actions.validate())

    @on(Button.Pressed, "#dash-demo")
    def _demo(self):
        self.app.start(lambda: self.actions.demo("demo"))

    @on(Button.Pressed, "#dash-web")
    def _web(self):
        self.app.action_web()


class InstancesPane(Pane):
    def compose(self) -> ComposeResult:
        with Horizontal():
            with Vertical(classes="left"):
                yield Label("[b]Instance sets[/b]", classes="section")
                yield OptionList(id="sets")
                with Horizontal(classes="toolbar"):
                    yield Button("Generate…", id="inst-generate")
                    yield Button("Import…", id="inst-import")
                yield Button("Re-split set…", id="inst-split")
            with Vertical(classes="right"):
                yield Label("", id="inst-title", classes="section")
                yield DataTable(id="inst-table", cursor_type="row", zebra_stripes=True)
                with Horizontal(id="inst-detail-row"):
                    yield Static(id="inst-map")
                    yield Static(id="inst-detail")

    def on_mount(self):
        self.query_one("#inst-table", DataTable).add_columns("Instance", "Family", "Split", "Holes", "Robots",
                                                            "Width", "Factors")
        self.current_set = None
        self.refresh_data()

    def refresh_data(self):
        sets = self.ws.instance_sets()
        ol = self.query_one("#sets", OptionList)
        keep = self.current_set
        ol.clear_options()
        for s in sets:
            ol.add_option(Option(Text.from_markup(f"[b]{s['name']}[/b]  [dim]{s['count']} · {fmt(s['splits'])}[/]"),
                                 id=s["path"]))
        if not sets:
            self.query_one("#inst-title", Label).update("No instance sets yet — Generate… or Import… one.")
            return
        paths = [s["path"] for s in sets]
        target = keep if keep in paths else paths[0]
        ol.highlighted = paths.index(target)
        self.load_set(target)

    @on(OptionList.OptionHighlighted, "#sets")
    def _set(self, event: OptionList.OptionHighlighted):
        if event.option.id != self.current_set:
            self.load_set(event.option.id)

    def load_set(self, path: str):
        self.current_set = path
        rows = self.ws.instances(path)
        self.query_one("#inst-title", Label).update(f"[b]{path}[/b]  [dim]{len(rows)} instances · Enter shows "
                                                     f"the instance[/]")
        tbl = self.query_one("#inst-table", DataTable)
        tbl.clear()
        for r in rows:
            tbl.add_row(r["id"], r["family"] or "–", r["split"] or "–", str(r["holes"]), str(r["robots"]),
                        fmt(r["width"]), fmt(r["factors"]), key=r["file"])

    @on(DataTable.RowHighlighted, "#inst-table")
    def _inst(self, event: DataTable.RowHighlighted):
        if event.row_key is not None:
            self.load_detail(event.row_key.value)

    @work(thread=True, exclusive=True, group="instance-detail")
    def load_detail(self, file: str):
        try:
            d = self.ws.instance_detail(file)
            m = ascii_map(d, cols=52)
        except Exception as e:  # show the problem instead of crashing the UI
            self.app.call_from_thread(self.query_one("#inst-detail", Static).update, f"[red]{e}[/red]")
            return
        t = Table.grid(padding=(0, 2))
        t.add_column(style="dim")
        t.add_column()
        for k, v in (("instance", d["id"]), ("family", d["family"]), ("split", d["split"]), ("hash", d["hash"]),
                     ("robots", d["robots"]), ("working width", f"{fmt(d['working_width'])} m"),
                     ("extent", f"{fmt(d['extent'][0])} × {fmt(d['extent'][1])} m"),
                     ("free area", f"{fmt(d['free_area'])} m²"), ("coverable F*", f"{fmt(d['coverable_area'])} m²"),
                     ("holes", d["holes"]), ("T_LB", f"{fmt(d['T_LB'])} s"), ("δ_e", f"{fmt(d['delta_e'])} m"),
                     ("factors", fmt(d["factors"])), ("source", d["metadata"].get("source"))):
            t.add_row(k, str(v if v is not None else "–"))
        self.app.call_from_thread(self.query_one("#inst-map", Static).update, m)
        self.app.call_from_thread(self.query_one("#inst-detail", Static).update, t)

    @on(Button.Pressed, "#inst-generate")
    def _gen(self):
        self.app.generate_dialog()

    @on(Button.Pressed, "#inst-import")
    def _imp(self):
        self.app.import_dialog()

    @on(Button.Pressed, "#inst-split")
    def _split(self):
        if not self.current_set:
            return self.app.notify("Select an instance set first", severity="warning")
        s = self.current_set

        def done(v):
            if v:
                self.app.start(lambda: self.actions.split(s, float(v["fraction"] or 0.2), v["stratify"] or None))

        self.app.push_screen(FormModal(f"Re-split {s}", [
            {"name": "fraction", "label": "Tuning fraction", "value": 0.2},
            {"name": "stratify", "label": "Stratify by factor (optional)", "value": ""}], "Assign splits"), done)


class PlannersPane(Pane):
    def compose(self) -> ComposeResult:
        with Horizontal():
            with Vertical(classes="left wide"):
                yield DataTable(id="pl-table", cursor_type="row", zebra_stripes=True)
                with Horizontal(classes="toolbar"):
                    yield Button("Validate selected", id="pl-validate")
                    yield Button("Validate all", variant="primary", id="pl-validate-all")
            with VerticalScroll(classes="right"):
                yield Static(id="pl-detail")

    def on_mount(self):
        self.query_one("#pl-table", DataTable).add_columns("Planner", "Family", "Mode", "Kind", "Validation")
        self.refresh_data()

    def refresh_data(self):
        self.manifests = {m["name"]: m for m in self.actions.planners()}
        tbl = self.query_one("#pl-table", DataTable)
        tbl.clear()
        for name, m in self.manifests.items():
            v = m.get("validation")
            val = Text(v["summary"], style="green" if v["passed"] else "red") if v else Text("not validated",
                                                                                           style="dim")
            tbl.add_row(name + (" *" if m.get("composition_of") else ""), m["family"], m["mode"],
                        "stochastic" if m["stochastic"] else "deterministic", val, key=name)

    @on(DataTable.RowHighlighted, "#pl-table")
    def _show(self, event: DataTable.RowHighlighted):
        if event.row_key is None:
            return
        m = self.manifests[event.row_key.value]
        self.selected = m["name"]
        t = Table(title=None, expand=True, show_edge=False)
        for c in ("hyperparameter", "type", "default", "range / choices", "meaning"):
            t.add_column(c)
        for hp in m["hyperparameters"]:
            if hp["kind"] == "bool":
                rng = "true, false"
            else:
                rng = ", ".join(map(str, hp.get("choices") or [])) or \
                    f"{fmt(hp.get('low'))} – {fmt(hp.get('high'))}" + (" (log)" if hp.get("log") else "")
            t.add_row(hp["name"], hp["kind"], fmt(hp.get("default")), rng, hp.get("doc", ""))
        from rich.console import Group

        head = Text.from_markup(
            f"[b]{m['name']}[/b] {m['version']}" + (" [yellow](staged composition)[/]" if m.get("composition_of")
                                                   else "") +
            f"\n{m['description']}\n[dim]{m['reference']}[/]\n\n"
            f"[dim]family[/] {m['family']}   [dim]adapter[/] {m['adapter']} {m['adapter_params']}\n"
            f"[dim]constraints[/] {', '.join(m['constraints']) or '–'}\n"
            + (f"[dim]stages[/] {' → '.join(m['stages'])}\n" if m["stages"] else "") + "\n")
        self.query_one("#pl-detail", Static).update(Group(head, t))

    @on(Button.Pressed, "#pl-validate")
    def _val(self):
        name = getattr(self, "selected", None)
        if name:
            self.app.start(lambda: self.actions.validate([name]))

    @on(Button.Pressed, "#pl-validate-all")
    def _val_all(self):
        self.app.start(lambda: self.actions.validate())


class ExperimentsPane(Pane):
    def compose(self) -> ComposeResult:
        with Horizontal():
            with Vertical(classes="left narrow"):
                yield Label("[b]Experiments[/b]", classes="section")
                yield OptionList(id="exp-list")
                yield Button("New experiment", variant="primary", id="exp-new")
            with Vertical(classes="right"):
                with Horizontal(classes="toolbar"):
                    yield Label("experiments/", classes="inline")
                    yield Input(id="exp-name", placeholder="name")
                    yield Label(".yaml", classes="inline")
                    yield Button("Save", id="exp-save")
                    yield Button("Check", id="exp-check")
                    yield Button("Tune…", id="exp-tune")
                    yield Button("Run…", variant="primary", id="exp-run")
                yield Static(id="exp-msgs")
                yield self._editor()

    def _editor(self) -> TextArea:
        try:
            return TextArea.code_editor("", language="yaml", id="exp-text")
        except Exception:  # syntax highlighting unavailable (no tree-sitter): plain editor
            return TextArea.code_editor("", id="exp-text")

    def on_mount(self):
        self.current = None
        self.refresh_data()

    def refresh_data(self):
        exps = self.ws.experiments()
        ol = self.query_one("#exp-list", OptionList)
        ol.clear_options()
        for e in exps:
            badge = " [cyan]● results[/]" if e.get("has_results") else ""
            d = f"⚠ {e['error']}" if e.get("error") else f"{len(e.get('planners') or [])} planners · m ∈ " \
                                                          f"{{{', '.join(map(str, e.get('team_sizes') or []))}}}"
            ol.add_option(Option(Text.from_markup(f"[b]{Path(e['path']).stem}[/b]{badge}\n[dim]{d}[/]"),
                                 id=e["path"]))
        paths = [e["path"] for e in exps]
        if self.current in paths:
            ol.highlighted = paths.index(self.current)
        elif paths and self.current is None:
            ol.highlighted = 0
            self.load(paths[0])

    @on(OptionList.OptionHighlighted, "#exp-list")
    def _pick(self, event: OptionList.OptionHighlighted):
        if event.option.id != self.current:
            self.load(event.option.id)

    def load(self, path: str):
        self.current = path
        self.query_one("#exp-name", Input).value = Path(path).stem
        self.query_one("#exp-text", TextArea).text = self.ws.read_text(path)
        self.query_one("#exp-msgs", Static).update("")

    def new_experiment(self):
        name = "my_experiment"
        self.current = ""
        self.query_one("#exp-name", Input).value = name
        self.query_one("#exp-text", TextArea).text = self.ws.new_experiment_text(name)
        self.query_one("#exp-msgs", Static).update("[dim]New experiment — edit and press Save (ctrl+s).[/]")
        self.query_one("#exp-text", TextArea).focus()

    @on(Button.Pressed, "#exp-new")
    def _new(self):
        self.new_experiment()

    def save(self) -> str:
        path = self.ws.write_experiment(self.query_one("#exp-name", Input).value,
                                        self.query_one("#exp-text", TextArea).text)
        self.current = path
        self.refresh_data()
        self.app.notify(f"Saved {path}")
        return path

    @on(Button.Pressed, "#exp-save")
    def _save(self):
        self.save()

    def check(self) -> dict:
        r = self.actions.check_experiment(self.query_one("#exp-text", TextArea).text)
        lines = [f"[red]✗ {e}[/]" for e in r["errors"]] + [f"[yellow]! {w}[/]" for w in r.get("warnings", [])]
        if r["ok"]:
            lines.append(f"[green]✓[/] {r['instances']} instances × m ∈ {{{', '.join(map(str, r['team_sizes']))}}} "
                         f"× {len(r['start_configs'])} start configs × {len(r['planners'])} planners ≈ "
                         f"[b]{r['estimated_runs']}[/b] runs (+{r['not_applicable']} not applicable) → {r['output']}")
        self.query_one("#exp-msgs", Static).update(Text.from_markup("\n".join(lines)))
        return r

    @on(Button.Pressed, "#exp-check")
    def _check(self):
        self.check()

    @on(Button.Pressed, "#exp-run")
    def _run(self):
        r = self.check()
        if not r["ok"]:
            return self.app.notify("Fix the configuration errors first", severity="error")
        path = self.save()

        def done(v):
            if v:
                w = int(v["workers"]) if str(v["workers"]).strip().isdigit() else None
                self.app.start(lambda: self.actions.run_experiment(path, w, bool(v["resume"])))

        self.app.push_screen(FormModal(f"Run {path}", [
            {"name": "workers", "label": "Worker processes (empty: from the configuration)", "value": ""},
            {"name": "resume", "label": "Resume: skip runs already recorded", "type": "checkbox", "value": True}],
            "Start run", help=f"About {r['estimated_runs']} runs; results go to {r['output']}."), done)

    @on(Button.Pressed, "#exp-tune")
    def _tune(self):
        path = self.save()

        def done(v):
            if v:
                self.app.start(lambda: self.actions.tune(path, int(v["budget"] or 20), None,
                                                         int(v["tuning_seeds"] or 3)))

        self.app.push_screen(FormModal(f"Tune {path}", [
            {"name": "budget", "label": "Budget (planner evaluations per planner and family)", "value": 20},
            {"name": "tuning_seeds", "label": "Seeds per evaluation (stochastic planners)", "value": 3}],
            "Start tuning", help="Bayesian optimisation on the tuning split; the best configuration is frozen to "
                                 "the tuning directory and used by `configs: [tuned]`."), done)


class JobsPane(Pane):
    def compose(self) -> ComposeResult:
        yield DataTable(id="jobs-table", cursor_type="row", zebra_stripes=True)
        with Horizontal(classes="toolbar"):
            yield Button("Cancel", variant="error", id="job-cancel")
            yield Button("Remove", id="job-remove")
            yield Button("Clear finished", id="job-clear")
            yield Button("Open result", id="job-open")
        yield Static(id="job-head")
        yield ProgressBar(id="job-progress", total=100, show_eta=False)
        yield Log(id="job-log", highlight=False)

    def on_mount(self):
        self.query_one("#jobs-table", DataTable).add_columns("Job", "Kind", "State", "Progress", "Started",
                                                            "Elapsed", "Last output")
        self.selected = None
        self._log_len = 0
        self.refresh_data()

    def refresh_data(self):
        tbl = self.query_one("#jobs-table", DataTable)
        jobs = self.jobs.list(limit=100)
        cur = tbl.cursor_row
        tbl.clear()
        for j in jobs:
            p = j.get("progress")
            tbl.add_row(j["title"], j["kind"], state_text(j["state"]), f"{p['done']}/{p['total']}" if p else "–",
                        time.strftime("%H:%M:%S", time.localtime(j.get("started") or j["created"])),
                        dur(j.get("elapsed")), (j.get("last_line") or "")[:70], key=j["id"])
        if jobs:
            ids = [j["id"] for j in jobs]
            row = ids.index(self.selected) if self.selected in ids else min(cur or 0, len(ids) - 1)
            tbl.move_cursor(row=row)
            self.show(ids[row])

    def select_job(self, jid: str):
        self.selected = None
        self._log_len = 0
        self.refresh_data()
        tbl = self.query_one("#jobs-table", DataTable)
        try:
            tbl.move_cursor(row=tbl.get_row_index(jid))
        except Exception:
            pass
        self.show(jid)

    @on(DataTable.RowHighlighted, "#jobs-table")
    def _hl(self, event: DataTable.RowHighlighted):
        if event.row_key is not None and event.row_key.value != self.selected:
            self.show(event.row_key.value)

    def show(self, jid: str):
        if jid != self.selected:
            self.selected = jid
            self._log_len = -1
        try:
            j = self.jobs.get(jid, log_bytes=400_000)
        except KeyError:
            return
        self.current_job = j
        head = Text.from_markup(f"[b]{j['title']}[/b]  ")
        head.append_text(state_text(j["state"]))
        head.append(f"   {dur(j.get('elapsed'))}   ", style="dim")
        head.append(f"$ uninbench {' '.join(j['argv'])}", style="dim")
        if j.get("returncode") is not None:
            head.append(f"   exit {j['returncode']}", style="dim")
        if j.get("artifacts"):
            head.append("   → " + ", ".join(f"{a['kind']}: {a['path']}" for a in j["artifacts"]), style="cyan")
        self.query_one("#job-head", Static).update(head)
        bar = self.query_one("#job-progress", ProgressBar)
        p = j.get("progress")
        bar.update(total=100, progress=100 * (p["fraction"] if p else (1.0 if j["state"] == "done" else 0.0)))
        log = self.query_one("#job-log", Log)
        text = j.get("log", "")
        if self._log_len < 0 or len(text) < self._log_len:
            log.clear()
            log.write(text)
        elif len(text) > self._log_len:
            log.write(text[self._log_len:])
        self._log_len = len(text)
        for b, show in (("#job-cancel", j["state"] in ACTIVE), ("#job-remove", j["state"] not in ACTIVE),
                        ("#job-open", bool(j.get("artifacts")))):
            self.query_one(b, Button).display = show

    def poll(self):
        if self.selected:
            self.show(self.selected)

    @on(Button.Pressed, "#job-cancel")
    def _cancel(self):
        if self.selected:
            self.jobs.cancel(self.selected)
            self.app.notify("Job cancelled", severity="warning")
            self.refresh_data()

    @on(Button.Pressed, "#job-remove")
    def _remove(self):
        if self.selected:
            try:
                self.jobs.remove(self.selected)
            except ValueError as e:
                return self.app.notify(str(e), severity="error")
            self.selected = None
            self.refresh_data()

    @on(Button.Pressed, "#job-clear")
    def _clear(self):
        def done(ok):
            if ok:
                n = self.jobs.clear_finished()
                self.app.notify(f"Removed {n} finished jobs")
                self.selected = None
                self.refresh_data()

        self.app.push_screen(ConfirmModal("Remove all finished jobs and their logs?"), done)

    @on(Button.Pressed, "#job-open")
    def _open(self):
        arts = getattr(self, "current_job", {}).get("artifacts") or []
        if arts:
            self.app.open_file(arts[-1]["path"])


class ResultsPane(Pane):
    def compose(self) -> ComposeResult:
        with Horizontal():
            with Vertical(classes="left narrow"):
                yield Label("[b]Result sets[/b]", classes="section")
                yield OptionList(id="res-list")
            with VerticalScroll(classes="right"):
                yield Static(id="res-head")
                with Horizontal(classes="toolbar"):
                    yield Button("Build report", id="res-report")
                    yield Button("Open report", variant="primary", id="res-open")
                    yield Button("Replay…", id="res-replay")
                    yield Button("Open latest replay", id="res-open-replay")
                yield Label("[b]Success rates[/b]", classes="section")
                yield DataTable(id="res-success", zebra_stripes=True)
                yield Label("[b]Normalized makespan T/T_LB[/b] (successful runs)", classes="section")
                yield DataTable(id="res-metric", zebra_stripes=True)

    def on_mount(self):
        self.query_one("#res-success", DataTable).add_columns("Family", "m", "Config", "Planner", "Runs",
                                                             "Success", "Incompl.", "Invalid", "Timeout", "Crash",
                                                             "n/a")
        self.query_one("#res-metric", DataTable).add_columns("Family", "m", "Planner", "Instances", "Median",
                                                            "IQR", "95 % CI")
        self.current = None
        self.summary = None
        self.refresh_data()

    def refresh_data(self):
        res = self.ws.results()
        ol = self.query_one("#res-list", OptionList)
        ol.clear_options()
        for r in res:
            oc = " ".join(f"{k[:4]}={v}" for k, v in r["outcomes"].items())
            ol.add_option(Option(Text.from_markup(f"[b]{r['name']}[/b]{' [cyan]● report[/]' if r['report'] else ''}"
                                                  f"\n[dim]{r['runs']} runs · {oc}[/]"), id=r["path"]))
        paths = [r["path"] for r in res]
        if not paths:
            self.query_one("#res-head", Static).update("No results yet — run an experiment first.")
            return
        target = self.current if self.current in paths else paths[0]
        ol.highlighted = paths.index(target)
        self.load(target)

    @on(OptionList.OptionHighlighted, "#res-list")
    def _pick(self, event: OptionList.OptionHighlighted):
        if event.option.id != self.current:
            self.load(event.option.id)

    @work(thread=True, exclusive=True, group="results")
    def load(self, path: str):
        self.current = path
        s = self.ws.results_summary(path)
        self.app.call_from_thread(self._fill, s)

    def _fill(self, s: dict):
        self.summary = s
        head = Text.from_markup(f"[b]{s['name']}[/b]  [dim]{s['path']} · {s['runs']} runs[/]\n")
        for k, v in s["outcomes"].items():
            head.append(f"{v} ", style="bold")
            head.append(f"{k.replace('_', ' ')}   ", style=STATE_STYLE.get(k, ""))
        self.query_one("#res-head", Static).update(head)
        t = self.query_one("#res-success", DataTable)
        t.clear()
        for r in s["success"]:
            t.add_row(r["family"], str(r["team_size"]), r["config"], r["planner"], str(r["runs"]),
                      fmt(r["success_rate"]), *[str(r.get(k, 0)) for k in ("incomplete", "invalid", "timeout",
                                                                             "crash", "not_applicable")])
        t = self.query_one("#res-metric", DataTable)
        t.clear()
        for r in s["descriptives"]:
            if r["metric"] == "normalized_makespan":
                t.add_row(r["family"], str(r["team_size"]), r["planner"], str(r["n"]), fmt(r["median"]),
                          f"{fmt(r['q1'])} – {fmt(r['q3'])}", f"{fmt(r['ci_lo'])} – {fmt(r['ci_hi'])}")

    @on(Button.Pressed, "#res-report")
    def _report(self):
        if self.current:
            self.app.start(lambda: self.actions.report(self.current))

    @on(Button.Pressed, "#res-open")
    def _open(self):
        if self.summary and self.summary.get("report"):
            self.app.open_file(self.summary["report"])
        else:
            self.app.notify("No report yet — press Build report", severity="warning")

    @on(Button.Pressed, "#res-open-replay")
    def _open_replay(self):
        if self.summary and self.summary.get("replays"):
            self.app.open_file(self.summary["replays"][-1])
        else:
            self.app.notify("No replay yet — press Replay…", severity="warning")

    @on(Button.Pressed, "#res-replay")
    def _replay(self):
        if not self.summary or not self.summary["replay_instances"]:
            return self.app.notify("No stored trajectories in this result set", severity="warning")
        opts = [(i, i) for i in self.summary["replay_instances"]]
        cur = self.current

        def done(v):
            if v:
                self.app.start(lambda: self.actions.replay(cur, v["instance"]))

        self.app.push_screen(FormModal("Build replay", [
            {"name": "instance", "label": "Instance", "type": "select", "options": opts, "value": opts[0][0]}],
            "Build replay", help="Side-by-side HTML replay of every planner on the instance."), done)


# ---------------------------------------------------------------------- app
class UninbenchApp(App):
    TITLE = "uninbench"
    CSS = """
    Screen { layout: vertical; }
    TabPane { padding: 0 1; }
    .toolbar { height: auto; margin: 1 0 0 0; }
    .toolbar Button { margin-right: 1; min-width: 10; }
    .toolbar Input { width: 28; }
    .inline { padding: 1 0 0 0; }
    .section { margin: 1 0 0 0; }
    .left { width: 44; margin-right: 2; }
    .left.narrow { width: 36; }
    .left.wide { width: 74; }
    .right { width: 1fr; }
    #sets, #exp-list, #res-list { height: 1fr; border: round $panel; }
    #inst-table { height: 1fr; min-height: 8; }
    #inst-detail-row { height: auto; max-height: 30; margin-top: 1; }
    #inst-map { width: auto; margin-right: 2; }
    #inst-detail { width: 1fr; }
    #pl-table { height: 1fr; }
    #exp-text { height: 1fr; }
    #exp-msgs { height: auto; margin: 1 0; }
    #jobs-table { height: 12; }
    #job-head { margin: 1 0 0 0; height: auto; }
    #job-progress { margin: 0 0 1 0; }
    #job-log { height: 1fr; border: round $panel; }
    #dash-summary { height: auto; margin: 1 0; }
    #dash-banner { height: auto; margin-top: 1; }
    #dash-jobs { height: 1fr; }
    #res-success, #res-metric { height: auto; max-height: 20; }
    """
    BINDINGS = [
        Binding("1", "show_tab('dashboard')", "Dashboard"),
        Binding("2", "show_tab('instances')", "Instances"),
        Binding("3", "show_tab('planners')", "Planners"),
        Binding("4", "show_tab('experiments')", "Experiments"),
        Binding("5", "show_tab('jobs')", "Jobs"),
        Binding("6", "show_tab('results')", "Results"),
        Binding("r", "refresh", "Refresh"),
        Binding("w", "web", "Web UI"),
        Binding("i", "init", "Init workspace", show=False),
        Binding("ctrl+s", "save", "Save", show=False),
        Binding("q", "quit", "Quit"),
    ]

    def __init__(self, workspace: str | Path = "."):
        super().__init__()
        self.ws = Workspace(workspace)
        self.jobs = JobManager(self.ws)
        self.actions = Actions(self.ws, self.jobs)
        self.web = None
        self._seen_states: dict = {}
        self.sub_title = str(self.ws.root)

    def compose(self) -> ComposeResult:
        yield Header(show_clock=True)
        with TabbedContent(initial="dashboard", id="tabs"):
            with TabPane("Dashboard", id="dashboard"):
                yield DashboardPane()
            with TabPane("Instances", id="instances"):
                yield InstancesPane()
            with TabPane("Planners", id="planners"):
                yield PlannersPane()
            with TabPane("Experiments", id="experiments"):
                yield ExperimentsPane()
            with TabPane("Jobs", id="jobs"):
                yield JobsPane()
            with TabPane("Results", id="results"):
                yield ResultsPane()
        yield Footer()

    def on_mount(self):
        self.set_interval(1.5, self._tick)

    # ------------------------------------------------------------------ helpers
    def _tick(self):
        """Poll jobs: refresh the visible job, and notify when jobs finish."""
        changed = False
        for j in self.jobs.list(limit=30):
            old = self._seen_states.get(j["id"])
            if old is not None and old in ACTIVE and j["state"] not in ACTIVE:
                sev = "information" if j["state"] == "done" else ("warning" if j["state"] == "cancelled" else "error")
                self.notify(f"{j['title']}: {j['state']}", severity=sev, timeout=6)
                changed = True
            self._seen_states[j["id"]] = j["state"]
        active = self.query_one(TabbedContent).active
        if active == "jobs":
            pane = self.query_one(JobsPane)
            pane.poll()
            if changed:
                pane.refresh_data()
        elif active == "dashboard":
            self.query_one(DashboardPane).refresh_data()
        if changed and active in ("instances", "results", "planners"):
            self._pane(active).refresh_data()

    def _pane(self, tab: str) -> Pane:
        return self.query_one(PANES[tab])

    def start(self, fn):
        """Start a background job and switch to the Jobs tab."""
        try:
            job = fn()
        except Exception as e:
            self.notify(str(e), severity="error", timeout=8)
            return None
        self._seen_states[job["id"]] = job["state"]
        self.notify(f"Started: {job['title']}")
        self.show_job(job["id"])
        return job

    def show_job(self, jid: str):
        self.action_show_tab("jobs")
        self.query_one(JobsPane).select_job(jid)

    def open_file(self, rel: str):
        path = self.ws.resolve(rel)
        url = path.as_uri()
        opened = False
        try:
            opened = webbrowser.open(url)
        except Exception:
            opened = False
        self.notify(("Opened " if opened else "Open in a browser: ") + str(path), timeout=10)

    def generate_dialog(self):
        def done(v):
            if v:
                self.start(lambda: self.actions.generate(
                    v["kind"], v["out"], int(v["count"] or 5), int(v["seed"] or 0), v["sizes"].split(),
                    v["densities"].split(), v["shapes"].split(), v["nonconvexity"].split(), int(v["rows"] or 20),
                    int(v["cols"] or 20), float(v["fraction"] or 0.2), v["stratify"] or None))

        self.push_screen(FormModal("Generate instances", [
            {"name": "kind", "label": "Generator", "type": "select", "value": "synthetic",
             "options": [("synthetic", "Synthetic environments (IF5)"), ("safelog", "SafeLog-like graphs (IF4)")]},
            {"name": "out", "label": "Output set", "value": "data/IF5"},
            {"name": "count", "label": "Instances per combination", "value": 5},
            {"name": "seed", "label": "Seed", "value": 0},
            {"name": "sizes", "label": "Sizes (width, m) — synthetic", "value": "20 40"},
            {"name": "densities", "label": "Obstacle / edge densities", "value": "0.05 0.1"},
            {"name": "shapes", "label": "Shapes (rect polygon circle mixed)", "value": "mixed"},
            {"name": "nonconvexity", "label": "Boundary notches — synthetic", "value": "0"},
            {"name": "rows", "label": "Grid rows — safelog", "value": 20},
            {"name": "cols", "label": "Grid columns — safelog", "value": 20},
            {"name": "fraction", "label": "Tuning fraction", "value": 0.2},
            {"name": "stratify", "label": "Stratify split by factor", "value": "obstacle_density"}], "Generate"),
            done)

    def import_dialog(self):
        def done(v):
            if v:
                params = dict(p.split("=", 1) for p in v["params"].split() if "=" in p)
                self.start(lambda: self.actions.import_dataset(v["dataset"], v["src"], v["out"], params,
                                                               float(v["fraction"] or 0.2), v["stratify"] or None))

        self.push_screen(FormModal("Import a dataset", [
            {"name": "dataset", "label": "Dataset", "type": "select", "value": "areacoverage",
             "options": [(k, f"{k} — {d}") for k, d in DATASETS.items()]},
            {"name": "src", "label": "Source directory", "value": "datasets/AreaCoverage-dataset"},
            {"name": "out", "label": "Output set", "value": "data/IF1"},
            {"name": "params", "label": "Importer parameters (k=v …)", "value": "", "placeholder": "width=3.0"},
            {"name": "fraction", "label": "Tuning fraction", "value": 0.2},
            {"name": "stratify", "label": "Stratify split by factor", "value": ""}], "Import",
            help="Download the public datasets first with scripts/fetch_datasets.sh (see docs/instances.md)."), done)

    # ------------------------------------------------------------------ actions
    def action_show_tab(self, tab: str):
        self.query_one(TabbedContent).active = tab
        pane = self._pane(tab)
        pane.refresh_data()
        # Textual re-activates the tab that contains the focused widget: move focus into the new tab
        target = pane.query(FOCUS[tab]).first() if pane.query(FOCUS[tab]) else None
        if target is not None:
            target.focus()

    def action_refresh(self):
        self._pane(self.query_one(TabbedContent).active).refresh_data()
        self.notify("Refreshed", timeout=2)

    def action_save(self):
        if self.query_one(TabbedContent).active == "experiments":
            self.query_one(ExperimentsPane).save()

    def action_init(self):
        made = self.ws.init()
        self.notify("Workspace initialized: " + (", ".join(made) if made else "nothing to do"))
        self.action_refresh()

    def action_web(self):
        if self.web is None:
            from .server import serve

            last = None
            for port in range(8350, 8370):
                try:
                    self.web = serve(self.ws.root, "127.0.0.1", port, open_browser=False, block=False)
                    break
                except OSError as e:
                    last = e
            if self.web is None:
                return self.notify(f"Could not start the web UI: {last}", severity="error")
        url = self.web[1]
        try:
            webbrowser.open(url)
        except Exception:
            pass
        self.notify(f"Web UI running at {url}", timeout=10)


FOCUS = {"dashboard": "#dash-jobs", "instances": "#sets", "planners": "#pl-table", "experiments": "#exp-list",
         "jobs": "#jobs-table", "results": "#res-list"}
PANES = {"dashboard": DashboardPane, "instances": InstancesPane, "planners": PlannersPane,
         "experiments": ExperimentsPane, "jobs": JobsPane, "results": ResultsPane}


def run_tui(workspace: str | Path = "."):
    UninbenchApp(workspace).run()

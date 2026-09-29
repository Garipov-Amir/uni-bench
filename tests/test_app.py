import json
import time

import pytest

from uninbench.app.actions import Actions
from uninbench.app.jobs import JobManager, progress_of
from uninbench.app.server import Api, compose_experiment
from uninbench.app.workspace import Workspace, WorkspaceError


@pytest.fixture
def ws(tmp_path):
    w = Workspace(tmp_path / "ws")
    w.init()
    return w


def _wait(jobs, jid, timeout=120):
    t0 = time.time()
    while time.time() - t0 < timeout:
        j = jobs.get(jid, log_bytes=100_000)
        if j["state"] not in ("queued", "running"):
            return j
        time.sleep(0.3)
    raise AssertionError("job did not finish")


def test_workspace_layout_and_safety(ws):
    assert ws.initialized and (ws.experiments_dir / "quick.yaml").exists()
    assert ws.summary()["experiments"] == 1
    with pytest.raises(WorkspaceError):
        ws.resolve("../outside.txt")
    assert ws.resolve("data/x") == ws.root / "data" / "x"


def test_progress_parsing():
    assert progress_of("[3/10] a\n[4/10] b")["done"] == 4
    assert progress_of("  [swath_bf/IF5] eval 2/8: objective")["total"] == 8
    assert progress_of("nothing") is None


def test_jobs_generate_and_run(ws):
    jobs = JobManager(ws)
    acts = Actions(ws, jobs)
    j = _wait(jobs, acts.generate("synthetic", "data/IF5", count=1, sizes=[14], densities=[0.05])["id"])
    assert j["state"] == "done", j["log"]
    assert ws.instance_sets()[0]["count"] == 1
    rows = ws.instances("data/IF5")
    detail = ws.instance_detail(rows[0]["file"])
    assert detail["coverable_area"] > 0 and detail["free_geometry"]
    text = compose_experiment(ws.read_text("experiments/quick.yaml"),
                              {"sets": ["data/IF5"], "planners": ["swath_bf"], "team_sizes": "1", "split": "all",
                               "repetitions": "1", "workers": "1", "time_limit": "30"})
    check = acts.check_experiment(text)
    assert check["ok"] and check["estimated_runs"] == 1
    path = ws.write_experiment("tiny", text)
    j = _wait(jobs, acts.run_experiment(path, workers=1)["id"])
    assert j["state"] == "done" and j["progress"]["done"] == 1
    res = ws.results()
    assert res and res[0]["runs"] == 1
    summary = ws.results_summary(res[0]["path"])
    assert summary["runs"] == 1 and summary["replay_instances"]
    j = _wait(jobs, acts.report(res[0]["path"])["id"])
    assert j["artifacts"] and j["artifacts"][0]["kind"] == "report"


def test_job_cancel(ws):
    jobs = JobManager(ws)
    j = jobs.start("validate", ["validate", "--out", "validation"], "validate")
    time.sleep(1.0)
    assert jobs.cancel(j["id"])["state"] == "cancelled"
    time.sleep(1.0)
    assert jobs.get(j["id"])["state"] == "cancelled"
    jobs.remove(j["id"])
    assert j["id"] not in jobs.ids()


def test_config_check_reports_errors(ws):
    acts = Actions(ws)
    bad = acts.check_experiment("name: x\nplanners: [nope]\ninstances: [{path: data/missing}]")
    assert not bad["ok"] and len(bad["errors"]) == 2
    assert not acts.check_experiment("name: [x")["ok"]


def test_api_dispatch_security(ws):
    api = Api(ws, token="secret")
    st, ctype, body = api.handle("GET", "/", {}, None)
    assert st == 200 and b"secret" in body
    st, _, body = api.handle("GET", "/api/summary", {}, None)
    assert st == 200 and json.loads(body)["experiments"] == 1
    assert api.handle("POST", "/api/validate", {}, {}, "wrong")[0] == 403
    assert api.handle("GET", "/files/../../etc/hosts", {}, None)[0] in (400, 404)
    assert api.handle("GET", "/static/../server.py", {}, None)[0] == 404
    assert api.handle("GET", "/api/nope", {}, None)[0] == 404
    st, _, body = api.handle("GET", "/api/planners", {}, None)
    assert {p["name"] for p in json.loads(body)} >= {"stc", "darp_stc", "swath_bf"}


def test_tui_smoke(ws):
    import asyncio

    from uninbench.app.tui import JobsPane, UninbenchApp

    async def run():
        app = UninbenchApp(ws.root)
        async with app.run_test(size=(140, 44)) as pilot:
            for key, tab in (("2", "instances"), ("3", "planners"), ("4", "experiments"), ("5", "jobs"),
                             ("6", "results"), ("1", "dashboard")):
                await pilot.press(key)
                await pilot.pause(0.2)
                assert app.query_one("TabbedContent").active == tab
            await pilot.click("#dash-validate")
            await pilot.pause(0.5)
            assert app.query_one("TabbedContent").active == "jobs"
            assert app.query_one(JobsPane).selected
            app.jobs.cancel(app.query_one(JobsPane).selected)

    asyncio.run(run())

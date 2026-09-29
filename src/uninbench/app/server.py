"""Local web interface: ``uninbench web``.

A dependency-free HTTP server (standard library) that serves a single-page app
and a small JSON API over the workspace.  It binds to 127.0.0.1 by default;
requests must carry a matching ``Host`` header (DNS-rebinding protection) and
state-changing requests must carry the per-session token embedded in the page.
"""

from __future__ import annotations

import json
import mimetypes
import secrets
import threading
import traceback
import webbrowser
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, unquote, urlparse

from .. import __version__
from ..util import to_jsonable
from .actions import DATASETS, Actions
from .jobs import JobManager
from .workspace import Workspace, WorkspaceError

WEB = Path(__file__).with_name("web")


class ApiError(Exception):
    def __init__(self, status: int, message: str):
        super().__init__(message)
        self.status = status


class Api:
    """Transport-independent request dispatcher (used by the HTTP handler and tests)."""

    def __init__(self, workspace: Workspace, token: str | None = None):
        self.ws = workspace
        self.jobs = JobManager(workspace)
        self.actions = Actions(workspace, self.jobs)
        self.token = token or secrets.token_urlsafe(24)

    # ------------------------------------------------------------------ dispatch
    def handle(self, method: str, path: str, query: dict, body: dict | None, token: str | None = None):
        """Returns ``(status, content_type, payload_bytes)``."""
        try:
            if path in ("/", "/index.html"):
                html = (WEB / "index.html").read_text(encoding="utf8").replace("__TOKEN__", self.token)
                return 200, "text/html; charset=utf-8", html.encode()
            if path.startswith("/static/"):
                return self._static(path[len("/static/"):])
            if path.startswith("/files/"):
                return self._file(unquote(path[len("/files/"):]))
            if not path.startswith("/api/"):
                raise ApiError(404, "not found")
            if method == "POST" and token != self.token:
                raise ApiError(403, "missing or invalid session token")
            route = getattr(self, f"{method.lower()}_{path[5:].replace('/', '_').replace('-', '_')}", None)
            if route is None:
                raise ApiError(404, f"no such endpoint: {method} {path}")
            q = {k: v[-1] for k, v in query.items()}
            result = route(q, body or {}) if method == "POST" else route(q)
            return 200, "application/json", json.dumps(to_jsonable(result)).encode()
        except ApiError as e:
            return e.status, "application/json", json.dumps({"error": str(e)}).encode()
        except (WorkspaceError, ValueError, KeyError, FileNotFoundError) as e:
            return 400, "application/json", json.dumps({"error": f"{type(e).__name__}: {e}"}).encode()
        except Exception as e:  # pragma: no cover - reported to the UI
            return 500, "application/json", json.dumps({"error": repr(e),
                                                        "traceback": traceback.format_exc()}).encode()

    def _static(self, rel: str):
        p = (WEB / rel).resolve()
        if WEB.resolve() not in p.parents or not p.is_file():
            raise ApiError(404, "not found")
        ctype = mimetypes.guess_type(p.name)[0] or "application/octet-stream"
        return 200, ctype, p.read_bytes()

    def _file(self, rel: str):
        p = self.ws.resolve(rel)
        if not p.is_file():
            raise ApiError(404, f"no such file: {rel}")
        if p.name.endswith(".gz"):
            return 200, "application/gzip", p.read_bytes()
        ctype = mimetypes.guess_type(p.name)[0] or "application/octet-stream"
        if ctype.startswith("text/") or ctype in ("application/json",):
            ctype += "; charset=utf-8"
        return 200, ctype, p.read_bytes()

    # ------------------------------------------------------------------ GET
    def get_summary(self, q):
        s = self.ws.summary()
        s.update(version=__version__, jobs=self.jobs.list(limit=8), running=len(self.jobs.running()),
                 datasets=DATASETS)
        return s

    def get_instance_sets(self, q):
        return self.ws.instance_sets()

    def get_instances(self, q):
        return self.ws.instances(q["set"])

    def get_instance(self, q):
        return self.ws.instance_detail(q["file"])

    def get_planners(self, q):
        return self.actions.planners()

    def get_experiments(self, q):
        return self.ws.experiments()

    def get_experiment(self, q):
        return {"path": q["path"], "text": self.ws.read_text(q["path"])}

    def get_experiment_template(self, q):
        return {"text": self.ws.new_experiment_text(q.get("name") or "experiment")}

    def get_results(self, q):
        return self.ws.results()

    def get_results_summary(self, q):
        return self.ws.results_summary(q["dir"])

    def get_runs(self, q):
        return self.ws.runs(q["dir"], q.get("planner") or None, q.get("outcome") or None,
                            int(q.get("limit", 500)))

    def get_jobs(self, q):
        return self.jobs.list(limit=int(q.get("limit", 50)))

    def get_job(self, q):
        return self.jobs.get(q["id"], log_bytes=int(q.get("log", 200_000)))

    # ------------------------------------------------------------------ POST
    def post_init(self, q, b):
        return {"created": self.ws.init(example=b.get("example", True))}

    def post_experiment_save(self, q, b):
        return {"path": self.ws.write_experiment(b["name"], b["text"])}

    def post_experiment_compose(self, q, b):
        return {"text": compose_experiment(b.get("text") or "", b.get("fields") or {})}

    def post_experiment_check(self, q, b):
        return self.actions.check_experiment(b["text"])

    def post_run(self, q, b):
        return self.actions.run_experiment(b["config"], _int(b.get("workers")), bool(b.get("resume", True)))

    def post_tune(self, q, b):
        return self.actions.tune(b["config"], _int(b.get("budget")) or 20, b.get("planners") or None,
                                 _int(b.get("tuning_seeds")) or 3, _int(b.get("workers")))

    def post_validate(self, q, b):
        return self.actions.validate(b.get("planners") or None)

    def post_report(self, q, b):
        return self.actions.report(b["dir"])

    def post_replay(self, q, b):
        return self.actions.replay(b["dir"], b.get("instance") or None, b.get("planners") or None)

    def post_generate(self, q, b):
        return self.actions.generate(
            b.get("kind", "synthetic"), b["out"], _int(b.get("count")) or 5, _int(b.get("seed")) or 0,
            _nums(b.get("sizes")), _nums(b.get("densities")), _words(b.get("shapes")), _nums(b.get("nonconvexity")),
            _int(b.get("rows")) or 20, _int(b.get("cols")) or 20, float(b.get("tuning_fraction") or 0.2),
            b.get("stratify") or None)

    def post_import(self, q, b):
        params = {}
        for item in _words(b.get("params")):
            k, _, v = item.partition("=")
            if k and v:
                params[k] = v
        return self.actions.import_dataset(b["dataset"], b["src"], b["out"], params,
                                           float(b.get("tuning_fraction") or 0.2), b.get("stratify") or None)

    def post_split(self, q, b):
        return self.actions.split(b["set"], float(b.get("tuning_fraction") or 0.2), b.get("stratify") or None)

    def post_check_evaluator(self, q, b):
        return self.actions.check_evaluator(b["src"])

    def post_demo(self, q, b):
        return self.actions.demo(b.get("out") or "demo")

    def post_job_cancel(self, q, b):
        return self.jobs.cancel(b["id"])

    def post_job_remove(self, q, b):
        self.jobs.remove(b["id"])
        return {"removed": b["id"]}

    def post_jobs_clear(self, q, b):
        return {"removed": self.jobs.clear_finished()}


def compose_experiment(text: str, f: dict) -> str:
    """Merge quick-builder fields into an experiment configuration (YAML text)."""
    import yaml

    cfg = yaml.safe_load(text) or {}
    if f.get("sets"):
        cfg["instances"] = [{"path": s} for s in f["sets"]]
    if f.get("planners"):
        configs = _words(f.get("configs")) or ["default"]
        cfg["planners"] = [p if configs == ["default"] else {"name": p, "configs": configs} for p in f["planners"]]
    if _words(f.get("team_sizes")):
        cfg["team_sizes"] = [int(float(x)) for x in _words(f["team_sizes"])]
    if _words(f.get("start_configs")):
        cfg["start_configs"] = _words(f["start_configs"])
    if _words(f.get("profiles")):
        cfg["profiles"] = _words(f["profiles"])
    if f.get("split"):
        cfg["split"] = f["split"]
    for key in ("seeds", "repetitions", "workers"):
        if _int(f.get(key)):
            cfg[key] = _int(f[key])
    task = dict(cfg.get("task") or {})
    if f.get("time_limit") not in (None, ""):
        task["time_limit"] = float(f["time_limit"])
    if f.get("tolerance") not in (None, ""):
        task["tolerance"] = float(f["tolerance"])
    if task:
        cfg["task"] = task
    return yaml.safe_dump(cfg, sort_keys=False, default_flow_style=None)


def _int(v):
    try:
        return int(v) if v not in (None, "") else None
    except (TypeError, ValueError):
        return None


def _words(v) -> list[str]:
    if v is None:
        return []
    if isinstance(v, list):
        return [str(x) for x in v if str(x).strip()]
    return [w for w in str(v).replace(",", " ").split() if w]


def _nums(v) -> list[float]:
    out = []
    for w in _words(v):
        try:
            out.append(float(w))
        except ValueError:
            raise ValueError(f"not a number: {w!r}") from None
    return out


# ---------------------------------------------------------------------- HTTP
def make_handler(api: Api, allowed_hosts: set[str]):
    class Handler(BaseHTTPRequestHandler):
        server_version = f"uninbench/{__version__}"

        def log_message(self, fmt, *args):  # quiet
            pass

        def _host_ok(self) -> bool:
            host = (self.headers.get("Host") or "").lower()
            return host in allowed_hosts

        def _send(self, status, ctype, payload: bytes):
            self.send_response(status)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(payload)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.end_headers()
            self.wfile.write(payload)

        def _dispatch(self, method):
            if not self._host_ok():
                return self._send(HTTPStatus.FORBIDDEN, "text/plain", b"forbidden host")
            url = urlparse(self.path)
            body = None
            if method == "POST":
                n = int(self.headers.get("Content-Length") or 0)
                if n > 5_000_000:
                    return self._send(413, "text/plain", b"request too large")
                try:
                    body = json.loads(self.rfile.read(n) or b"{}")
                except ValueError:
                    return self._send(400, "application/json", b'{"error": "invalid JSON"}')
            status, ctype, payload = api.handle(method, url.path, parse_qs(url.query), body,
                                                self.headers.get("X-Uninbench-Token"))
            self._send(status, ctype, payload)

        def do_GET(self):
            self._dispatch("GET")

        def do_POST(self):
            self._dispatch("POST")

    return Handler


def serve(workspace: str | Path = ".", host: str = "127.0.0.1", port: int = 8350, open_browser: bool = False,
          block: bool = True):
    ws = Workspace(workspace)
    api = Api(ws)
    names = {host, "localhost", "127.0.0.1", "[::1]"} if host in ("127.0.0.1", "localhost", "::1") else {host}
    httpd = ThreadingHTTPServer((host, port), None)
    port = httpd.server_address[1]
    allowed = {f"{n}:{port}".lower() for n in names}
    httpd.RequestHandlerClass = make_handler(api, allowed)
    url = f"http://{'localhost' if host in ('127.0.0.1', '::1') else host}:{port}/"
    if open_browser:
        threading.Timer(0.5, lambda: webbrowser.open(url)).start()
    if not block:
        t = threading.Thread(target=httpd.serve_forever, daemon=True)
        t.start()
        return httpd, url
    print(f"uninbench web UI for workspace {ws.root}\n  → {url}   (Ctrl+C to stop)", flush=True)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        httpd.server_close()
    return httpd, url

"""Background jobs shared by the TUI and the web UI.

A job runs one headless ``uninbench`` command (``run``, ``tune``, ``report`` ...)
in a detached process group, so it keeps running when the interface that
started it is closed, and every interface can observe or cancel it.  Each job
lives in ``<workspace>/.uninbench/jobs/<id>/``::

    job.json      what to run (kind, title, argv, created)
    status.json   state, pids, exit code, timestamps (written by the job wrapper)
    output.log    combined stdout/stderr of the command

The wrapper is ``python -m uninbench.app.jobs <jobdir>``.
"""

from __future__ import annotations

import json
import os
import re
import signal
import subprocess
import sys
import time
import uuid
from pathlib import Path

from .workspace import Workspace

STATES = ("queued", "running", "done", "failed", "cancelled", "lost")
_PROGRESS = [re.compile(r"\[(\d+)/(\d+)\]"), re.compile(r"eval (\d+)/(\d+)")]
_ARTIFACT = re.compile(r"^(report|replay|figure):\s+(\S.*)$", re.M)


def _write(path: Path, data: dict):
    tmp = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    tmp.write_text(json.dumps(data, indent=1), encoding="utf8")
    os.replace(tmp, path)


def _read(path: Path) -> dict:
    try:
        return json.loads(path.read_text(encoding="utf8"))
    except (OSError, ValueError):
        return {}


def _alive(pid) -> bool:
    if not pid:
        return False
    try:
        os.kill(int(pid), 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def tail(path: Path, max_bytes: int = 64_000) -> str:
    try:
        with open(path, "rb") as fh:
            fh.seek(0, os.SEEK_END)
            size = fh.tell()
            fh.seek(max(0, size - max_bytes))
            data = fh.read().decode("utf8", errors="replace")
    except OSError:
        return ""
    if size > max_bytes:
        data = data.split("\n", 1)[-1]
    # the runner uses carriage returns for live progress on terminals
    return "\n".join(line.split("\r")[-1] for line in data.split("\n"))


def progress_of(text: str):
    best = None
    for rx in _PROGRESS:
        for m in rx.finditer(text):
            best = (int(m.group(1)), int(m.group(2)))
    if best and best[1] > 0:
        return {"done": best[0], "total": best[1], "fraction": min(1.0, best[0] / best[1])}
    return None


class JobManager:
    def __init__(self, workspace: Workspace):
        self.ws = workspace

    @property
    def root(self) -> Path:
        return self.ws.jobs_dir

    def start(self, kind: str, argv: list[str], title: str | None = None) -> dict:
        self.root.mkdir(parents=True, exist_ok=True)
        jid = time.strftime("%Y%m%d-%H%M%S-") + uuid.uuid4().hex[:6]
        d = self.root / jid
        d.mkdir()
        job = {"id": jid, "kind": kind, "title": title or f"{kind} {' '.join(argv[1:3])}".strip(),
               "argv": [str(a) for a in argv], "cwd": str(self.ws.root), "created": time.time()}
        _write(d / "job.json", job)
        _write(d / "status.json", {"state": "queued"})
        env = dict(os.environ, PYTHONUNBUFFERED="1", UNINBENCH_PROGRESS_EVERY="1", MPLBACKEND="Agg")
        subprocess.Popen([sys.executable, "-m", "uninbench.app.jobs", str(d)], cwd=self.ws.root, env=env,
                         stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                         start_new_session=True)
        return self.get(jid)

    def ids(self) -> list[str]:
        if not self.root.is_dir():
            return []
        return sorted((p.name for p in self.root.iterdir() if (p / "job.json").exists()), reverse=True)

    def get(self, jid: str, log_bytes: int = 0) -> dict:
        d = self.root / jid
        if not (d / "job.json").exists():
            raise KeyError(jid)
        job = _read(d / "job.json")
        st = _read(d / "status.json")
        state = st.get("state", "queued")
        if state == "running" and not _alive(st.get("pid")):
            state = "lost"
        if state == "queued" and time.time() - job.get("created", 0) > 30:
            state = "lost"
        log = tail(d / "output.log", max(log_bytes, 16_000))
        job.update(state=state, returncode=st.get("returncode"), started=st.get("started"),
                   finished=st.get("finished"), progress=progress_of(log),
                   artifacts=[{"kind": k, "path": self.ws.rel(Path(self.ws.root, p.strip()))}
                              for k, p in _ARTIFACT.findall(log)])
        job["elapsed"] = ((st.get("finished") or time.time()) - st["started"]) if st.get("started") else None
        if log_bytes:
            job["log"] = tail(d / "output.log", log_bytes)
        else:
            job["last_line"] = next((ln for ln in reversed(log.splitlines()) if ln.strip()), "")
        return job

    def list(self, limit: int = 50) -> list[dict]:
        out = []
        for jid in self.ids()[:limit]:
            try:
                out.append(self.get(jid))
            except KeyError:
                continue
        return out

    def running(self) -> list[dict]:
        return [j for j in self.list() if j["state"] in ("queued", "running")]

    def cancel(self, jid: str) -> dict:
        d = self.root / jid
        st = _read(d / "status.json")
        pid = st.get("pid")
        if pid and _alive(pid):
            try:
                os.killpg(os.getpgid(int(pid)), signal.SIGTERM)
            except (ProcessLookupError, PermissionError):
                pass
        st.update(state="cancelled", finished=time.time())
        _write(d / "status.json", st)
        return self.get(jid)

    def remove(self, jid: str):
        d = self.root / jid
        if self.get(jid)["state"] in ("queued", "running"):
            raise ValueError("cancel the job before removing it")
        for f in d.iterdir():
            f.unlink()
        d.rmdir()

    def clear_finished(self) -> int:
        n = 0
        for j in self.list(limit=10_000):
            if j["state"] not in ("queued", "running"):
                self.remove(j["id"])
                n += 1
        return n


# ---------------------------------------------------------------------- wrapper
def _run(jobdir: Path) -> int:
    job = _read(jobdir / "job.json")
    status = {"state": "running", "pid": os.getpid(), "started": time.time()}
    _write(jobdir / "status.json", status)
    cancelled = {"flag": False}

    def on_term(signum, frame):
        cancelled["flag"] = True

    signal.signal(signal.SIGTERM, on_term)
    with open(jobdir / "output.log", "ab", buffering=0) as log:
        log.write(f"$ uninbench {' '.join(job['argv'])}\n".encode())
        proc = subprocess.Popen([sys.executable, "-m", "uninbench.cli", *job["argv"]], cwd=job.get("cwd"),
                                stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL)
        status["child_pid"] = proc.pid
        _write(jobdir / "status.json", status)
        while True:
            try:
                rc = proc.wait(timeout=0.5)
                break
            except subprocess.TimeoutExpired:
                if cancelled["flag"]:
                    proc.terminate()
    if cancelled["flag"] or _read(jobdir / "status.json").get("state") == "cancelled":
        state = "cancelled"
    else:
        state = "done" if rc == 0 else "failed"
    status.update(state=state, returncode=rc, finished=time.time())
    _write(jobdir / "status.json", status)
    return rc


if __name__ == "__main__":  # pragma: no cover - exercised through JobManager.start
    sys.exit(_run(Path(sys.argv[1])))

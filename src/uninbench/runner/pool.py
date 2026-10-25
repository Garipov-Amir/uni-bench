"""A pool of worker processes with per-planner-call timeouts and a memory limit.

* Planning time is limited per planner call: the worker announces the start of
  each call, and the parent terminates (and replaces) a worker whose call runs
  longer than the limit plus a grace period.  Adapter and evaluation time are
  not limited by the planning time limit.
* By default every worker executes a single run and is then replaced by a
  fresh process (``max_tasks_per_worker = 1``), so the peak memory of a run is
  not inflated by earlier runs.  Workers are forked from a preloaded fork
  server where available, which makes the replacement cheap.
* Where the operating system does not enforce ``RLIMIT_AS`` (macOS), the
  parent polls the resident set size of busy workers and terminates a worker
  that exceeds the memory limit; the run is recorded as a crash.
"""

from __future__ import annotations

import multiprocessing as mp
import sys
import time
import traceback
from multiprocessing.connection import wait

from .memory import current_rss_mb, rlimit_enforced
from .worker import RunTask, execute, failure_attribution, worker_main

MEMORY_POLL_S = 0.25


def _context():
    methods = mp.get_all_start_methods()
    if "forkserver" in methods and sys.platform != "win32":
        ctx = mp.get_context("forkserver")
        try:
            ctx.set_forkserver_preload(["uninbench.runner.worker"])
        except Exception:  # pragma: no cover
            pass
        return ctx
    return mp.get_context("spawn")


class _Worker:
    def __init__(self, ctx, memory_limit_mb):
        self.parent, child = ctx.Pipe()
        self.proc = ctx.Process(target=worker_main, args=(child, memory_limit_mb), daemon=True)
        self.proc.start()
        child.close()
        self.task: RunTask | None = None
        self.partial: dict | None = None
        self.deadline: float | None = None
        self.planning_started: float | None = None
        self.done = 0

    def submit(self, task: RunTask):
        self.task = task
        self.partial = None
        self.deadline = None
        self.parent.send(task)

    def kill(self):
        try:
            self.proc.kill()
        except Exception:
            pass
        self.proc.join(timeout=5)
        try:
            self.parent.close()
        except Exception:
            pass


class WorkerPool:
    def __init__(self, workers: int = 1, memory_limit_mb: int | None = None, grace: float = 5.0,
                 max_tasks_per_worker: int | None = 1):
        self.ctx = _context()
        self.memory_limit_mb = memory_limit_mb
        self.grace = grace
        self.max_tasks = max_tasks_per_worker
        self.poll_memory = bool(memory_limit_mb) and not rlimit_enforced()
        try:
            first = _Worker(self.ctx, memory_limit_mb)
        except OSError:  # no fork server (e.g. local sockets not permitted): plain spawned processes
            self.ctx = mp.get_context("spawn")
            first = _Worker(self.ctx, memory_limit_mb)
        self.workers = [first] + [_Worker(self.ctx, memory_limit_mb) for _ in range(max(1, workers) - 1)]

    def run(self, tasks, on_result, on_progress=None):
        """Execute tasks; ``on_result(record)`` is called for every finished run."""
        queue = list(tasks)
        queue.reverse()
        busy: list[_Worker] = []
        idle = list(self.workers)
        done = 0
        total = len(queue)
        last_poll = 0.0

        def finish(w, rec, replace: bool):
            nonlocal done
            on_result(rec)
            done += 1
            busy.remove(w)
            w.task = None
            if replace:
                w.kill()
                self._replace(w, idle)
            else:
                idle.append(w)
            if on_progress:
                on_progress(done, total, rec)

        while queue or busy:
            while queue and idle:
                w = idle.pop()
                w.submit(queue.pop())
                busy.append(w)
            now = time.time()
            timeouts = [w.deadline - now for w in busy if w.deadline is not None]
            wait_for = max(0.05, min(timeouts)) if timeouts else 1.0
            if self.poll_memory:
                wait_for = min(wait_for, MEMORY_POLL_S)
            ready = wait([w.parent for w in busy], timeout=wait_for)
            check_memory = self.poll_memory and time.time() - last_poll >= MEMORY_POLL_S
            if check_memory:
                last_poll = time.time()
            for w in list(busy):
                if w.parent in ready:
                    try:
                        msg = w.parent.recv()
                    except (EOFError, OSError):
                        finish(w, _crash(w, "worker process died (killed or out of memory)"), replace=True)
                        continue
                    kind = msg[0]
                    if kind == "partial":
                        w.partial = msg[1]
                    elif kind == "planning":
                        w.planning_started = time.time()
                        w.deadline = w.planning_started + float(msg[1]) + self.grace
                    elif kind == "planned":
                        w.deadline = None
                    elif kind == "result":
                        w.done += 1
                        retire = self.max_tasks is not None and w.done >= self.max_tasks
                        finish(w, msg[1], replace=retire)
                elif w.deadline is not None and time.time() > w.deadline:
                    finish(w, _timeout(w, time.time() - (w.planning_started or time.time())), replace=True)
                elif not w.proc.is_alive():
                    finish(w, _crash(w, f"worker exited with code {w.proc.exitcode}"), replace=True)
                elif check_memory:
                    rss = current_rss_mb(w.proc.pid)
                    if rss is not None and rss > float(self.memory_limit_mb):
                        finish(w, _crash(w, f"memory limit exceeded ({rss:.0f} MB > {self.memory_limit_mb} MB)"),
                               replace=True)

    def _replace(self, w, idle):
        self.workers.remove(w)
        nw = _Worker(self.ctx, self.memory_limit_mb)
        self.workers.append(nw)
        idle.append(nw)

    def close(self):
        for w in self.workers:
            try:
                w.parent.send(None)
            except Exception:
                pass
        for w in self.workers:
            w.proc.join(timeout=2)
            if w.proc.is_alive():
                w.kill()


def _stub(w: _Worker) -> dict:
    """The record of a run the parent had to stop: what the worker reported so far."""
    if w.partial is not None:
        return dict(w.partial)
    from ..planners import get_planner
    from .worker import _instance, base_record

    task = w.task
    try:
        return base_record(task, _instance(task.instance), get_planner(task.planner))
    except Exception:
        return {"run_id": task.run_id, "planner": task.planner_label, "seed": task.seed}


def _timeout(w: _Worker, elapsed: float) -> dict:
    rec = _stub(w)
    rec.update(outcome="timeout",
               reason=f"planner call exceeded {w.task.time_limit} s (killed after {elapsed:.1f} s)")
    rec["failure_attribution"] = failure_attribution(rec)
    return rec


def _crash(w: _Worker, reason: str) -> dict:
    rec = _stub(w)
    rec.update(outcome="crash", reason=reason)
    rec["failure_attribution"] = failure_attribution(rec)
    return rec


def run_inline(tasks, on_result, on_progress=None):
    """Run tasks in-process (debugging, tests); timeouts are detected post hoc and
    peak memory is not attributable to single runs."""
    total = len(tasks)
    for k, task in enumerate(tasks):
        try:
            rec = execute(task, lambda msg: None)
        except Exception as e:
            from .worker import _instance, base_record
            from ..planners import get_planner

            try:
                rec = base_record(task, _instance(task.instance), get_planner(task.planner))
            except Exception:
                rec = {"run_id": task.run_id, "planner": task.planner_label, "seed": task.seed}
            rec.update(outcome="crash", reason=repr(e), traceback=traceback.format_exc())
            rec["failure_attribution"] = failure_attribution(rec)
        on_result(rec)
        if on_progress:
            on_progress(k + 1, total, rec)

"""A pool of long-lived worker processes with per-planner-call timeouts.

Planning time is limited per planner call: the worker announces the start of
each call, and the parent terminates (and replaces) a worker whose call runs
longer than the limit plus a grace period.  Adapter and evaluation time are
not limited by the planning time limit.
"""

from __future__ import annotations

import multiprocessing as mp
import time
import traceback
from multiprocessing.connection import wait

from .worker import RunTask, execute, worker_main


class _Worker:
    def __init__(self, ctx, memory_limit_mb):
        self.parent, child = ctx.Pipe()
        self.proc = ctx.Process(target=worker_main, args=(child, memory_limit_mb), daemon=True)
        self.proc.start()
        child.close()
        self.task: RunTask | None = None
        self.deadline: float | None = None
        self.planning_started: float | None = None

    def submit(self, task: RunTask):
        self.task = task
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
    def __init__(self, workers: int = 1, memory_limit_mb: int | None = None, grace: float = 5.0):
        self.ctx = mp.get_context("spawn")
        self.memory_limit_mb = memory_limit_mb
        self.grace = grace
        self.workers = [_Worker(self.ctx, memory_limit_mb) for _ in range(max(1, workers))]

    def run(self, tasks, on_result, on_progress=None):
        """Execute tasks; ``on_result(record)`` is called for every finished run."""
        queue = list(tasks)
        queue.reverse()
        busy: list[_Worker] = []
        idle = list(self.workers)
        done = 0
        total = len(queue)
        while queue or busy:
            while queue and idle:
                w = idle.pop()
                w.submit(queue.pop())
                busy.append(w)
            now = time.time()
            timeouts = [w.deadline - now for w in busy if w.deadline is not None]
            wait_for = max(0.05, min(timeouts)) if timeouts else 1.0
            ready = wait([w.parent for w in busy], timeout=wait_for)
            for w in list(busy):
                if w.parent in ready:
                    try:
                        msg = w.parent.recv()
                    except (EOFError, OSError):
                        rec = _crash(w.task, "worker process died (killed or out of memory)")
                        on_result(rec)
                        done += 1
                        busy.remove(w)
                        w.kill()
                        self._replace(w, idle)
                        continue
                    kind = msg[0]
                    if kind == "planning":
                        w.planning_started = time.time()
                        w.deadline = w.planning_started + float(msg[1]) + self.grace
                    elif kind == "planned":
                        w.deadline = None
                    elif kind == "result":
                        on_result(msg[1])
                        done += 1
                        busy.remove(w)
                        w.task = None
                        idle.append(w)
                        if on_progress:
                            on_progress(done, total, msg[1])
                elif w.deadline is not None and time.time() > w.deadline:
                    rec = _timeout(w.task, time.time() - (w.planning_started or time.time()))
                    on_result(rec)
                    done += 1
                    busy.remove(w)
                    w.kill()
                    self._replace(w, idle)
                    if on_progress:
                        on_progress(done, total, rec)
                elif not w.proc.is_alive():
                    rec = _crash(w.task, f"worker exited with code {w.proc.exitcode}")
                    on_result(rec)
                    done += 1
                    busy.remove(w)
                    self._replace(w, idle)

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


def _stub(task: RunTask) -> dict:
    from .worker import _instance, base_record
    from ..planners import get_planner

    try:
        return base_record(task, _instance(task.instance), get_planner(task.planner))
    except Exception:
        return {"run_id": task.run_id, "planner": task.planner_label, "seed": task.seed}


def _timeout(task: RunTask, elapsed: float) -> dict:
    rec = _stub(task)
    rec.update(outcome="timeout", reason=f"planner call exceeded {task.time_limit} s (killed after {elapsed:.1f} s)")
    return rec


def _crash(task: RunTask, reason: str) -> dict:
    rec = _stub(task)
    rec.update(outcome="crash", reason=reason)
    return rec


def run_inline(tasks, on_result, on_progress=None):
    """Run tasks in-process (debugging, tests); timeouts are detected post hoc."""
    total = len(tasks)
    for k, task in enumerate(tasks):
        try:
            rec = execute(task, lambda msg: None)
        except Exception as e:
            rec = _crash(task, repr(e))
            rec["traceback"] = traceback.format_exc()
        on_result(rec)
        if on_progress:
            on_progress(k + 1, total, rec)

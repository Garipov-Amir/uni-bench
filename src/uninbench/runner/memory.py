"""Memory measurement and enforcement for benchmark runs.

* :func:`maxrss_mb` -- peak resident set size of the calling process.  Runs
  execute in a fresh worker process (one task per process by default), so the
  value measured at the end of a planner call belongs to that run only.
* :func:`current_rss_mb` -- current resident set size of any process; the
  worker pool polls it to enforce the memory limit on platforms that do not
  enforce ``RLIMIT_AS`` (macOS).
"""

from __future__ import annotations

import ctypes
import ctypes.util
import resource
import sys


def maxrss_mb() -> float:
    r = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    return r / (1024 * 1024) if sys.platform == "darwin" else r / 1024


class _ProcTaskInfo(ctypes.Structure):
    _fields_ = [("pti_virtual_size", ctypes.c_uint64), ("pti_resident_size", ctypes.c_uint64),
                ("pti_total_user", ctypes.c_uint64), ("pti_total_system", ctypes.c_uint64),
                ("pti_threads_user", ctypes.c_uint64), ("pti_threads_system", ctypes.c_uint64),
                ("pti_policy", ctypes.c_int32), ("pti_faults", ctypes.c_int32), ("pti_pageins", ctypes.c_int32),
                ("pti_cow_faults", ctypes.c_int32), ("pti_messages_sent", ctypes.c_int32),
                ("pti_messages_received", ctypes.c_int32), ("pti_syscalls_mach", ctypes.c_int32),
                ("pti_syscalls_unix", ctypes.c_int32), ("pti_csw", ctypes.c_int32),
                ("pti_threadnum", ctypes.c_int32), ("pti_numrunning", ctypes.c_int32),
                ("pti_priority", ctypes.c_int32)]


_LIBPROC = None
_PROC_PIDTASKINFO = 4


def _libproc():
    global _LIBPROC
    if _LIBPROC is None:
        try:
            _LIBPROC = ctypes.CDLL(ctypes.util.find_library("proc") or "/usr/lib/libproc.dylib", use_errno=True)
        except OSError:
            _LIBPROC = False
    return _LIBPROC


def current_rss_mb(pid: int) -> float | None:
    """Current resident set size of process ``pid`` in MiB (None if unavailable)."""
    if sys.platform.startswith("linux"):
        try:
            with open(f"/proc/{pid}/statm", encoding="ascii") as fh:
                pages = int(fh.read().split()[1])
            return pages * resource.getpagesize() / 2 ** 20
        except (OSError, ValueError, IndexError):
            return None
    if sys.platform == "darwin":
        lib = _libproc()
        if not lib:
            return None
        info = _ProcTaskInfo()
        n = lib.proc_pidinfo(int(pid), _PROC_PIDTASKINFO, ctypes.c_uint64(0), ctypes.byref(info),
                             ctypes.sizeof(info))
        return info.pti_resident_size / 2 ** 20 if n == ctypes.sizeof(info) else None
    return None


def rlimit_enforced() -> bool:
    """Whether ``RLIMIT_AS`` limits memory on this platform."""
    return sys.platform.startswith("linux")

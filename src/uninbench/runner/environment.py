"""Description of the hardware and software environment stored with every run."""

from __future__ import annotations

import os
import platform
import subprocess
import sys
from functools import lru_cache
from importlib import metadata
from pathlib import Path

from .. import __version__


def _mem_total_bytes():
    try:
        return int(os.sysconf("SC_PAGE_SIZE") * os.sysconf("SC_PHYS_PAGES"))
    except (ValueError, OSError, AttributeError):
        pass
    try:
        out = subprocess.run(["sysctl", "-n", "hw.memsize"], capture_output=True, text=True, timeout=2)
        return int(out.stdout.strip())
    except Exception:
        return None


def _git_revision() -> str | None:
    root = Path(__file__).resolve().parents[3]
    if not (root / ".git").exists():
        return None
    try:
        out = subprocess.run(["git", "-C", str(root), "rev-parse", "HEAD"], capture_output=True, text=True, timeout=2)
        rev = out.stdout.strip()
        dirty = subprocess.run(["git", "-C", str(root), "status", "--porcelain"], capture_output=True, text=True,
                               timeout=2).stdout.strip()
        return rev + ("-dirty" if dirty else "") if rev else None
    except Exception:
        return None


def _pkg(name: str):
    try:
        return metadata.version(name)
    except metadata.PackageNotFoundError:
        return None


@lru_cache(maxsize=1)
def describe_environment() -> dict:
    return {
        "hardware": {
            "platform": platform.platform(),
            "machine": platform.machine(),
            "processor": platform.processor() or None,
            "cpu_count": os.cpu_count(),
            "memory_bytes": _mem_total_bytes(),
            "node": platform.node(),
        },
        "software": {
            "uninbench": __version__,
            "git": _git_revision(),
            "python": sys.version.split()[0],
            "numpy": _pkg("numpy"),
            "scipy": _pkg("scipy"),
            "shapely": _pkg("shapely"),
            "networkx": _pkg("networkx"),
            "container_image": os.environ.get("UNINBENCH_IMAGE") or ("docker" if Path("/.dockerenv").exists()
                                                                      else None),
        },
    }

"""Tuning/test splits, stratified by the varied factor (R8)."""

from __future__ import annotations

import hashlib
import math
from collections import defaultdict
from pathlib import Path

from ..util import dump_json, load_json
from .model import Instance


def _stratum(meta: dict, key: str | None):
    if not key:
        return "all"
    fac = meta.get("factors", {})
    v = fac.get(key, meta.get(key, "all"))
    return str(v)


def assign_splits(instances: list[Instance], tuning_fraction: float = 0.2, stratify: str | None = None,
                  seed: int = 0) -> dict[str, str]:
    """Deterministically assign each instance to 'tuning' or 'test'.

    Paired variants (instances sharing ``metadata["pair"]``, e.g. the empty and
    furnished version of a room) are always assigned to the same split.
    """
    groups: dict[str, list[Instance]] = defaultdict(list)
    for inst in instances:
        groups[str(inst.metadata.get("pair", inst.id))].append(inst)
    strata = defaultdict(list)
    for gkey, members in groups.items():
        first = members[0]
        strata[(first.family, _stratum(first.metadata, stratify))].append((gkey, members))
    out = {}
    for _, units in sorted(strata.items()):
        units = sorted(units, key=lambda u: hashlib.sha256(f"{seed}:{u[0]}".encode()).hexdigest())
        n_tune = int(math.floor(tuning_fraction * len(units) + 0.5))
        for k, (_, members) in enumerate(units):
            for inst in members:
                out[inst.id] = "tuning" if k < n_tune else "test"
    return out


def split_directory(directory: str | Path, tuning_fraction: float = 0.2, stratify: str | None = None,
                    seed: int = 0) -> dict[str, str]:
    """Assign splits to every instance JSON in a directory and write index.json."""
    directory = Path(directory)
    paths = sorted(p for p in directory.glob("*.json") if p.name != "index.json")
    insts = [Instance.load(p) for p in paths]
    splits = assign_splits(insts, tuning_fraction, stratify, seed)
    index = []
    for p, inst in zip(paths, insts):
        inst.metadata["split"] = splits[inst.id]
        inst.save(p)
        index.append({"id": inst.id, "file": p.name, "family": inst.family, "hash": inst.hash,
                      "split": splits[inst.id], "factors": inst.metadata.get("factors", {})})
    dump_json({"tuning_fraction": tuning_fraction, "stratify": stratify, "seed": seed, "instances": index},
              directory / "index.json")
    return splits


def load_instances(pattern: str | Path, split: str | None = None) -> list[Instance]:
    """Load instances from a file, a directory or a glob pattern."""
    pattern = Path(pattern)
    if pattern.is_dir():
        paths = sorted(p for p in pattern.glob("*.json") if p.name != "index.json")
    elif any(ch in str(pattern) for ch in "*?["):
        paths = sorted(p for p in Path(".").glob(str(pattern)) if p.name != "index.json")
    else:
        paths = [pattern]
    out = []
    for p in paths:
        inst = Instance.load(p)
        if split and inst.metadata.get("split", "test") != split:
            continue
        out.append(inst)
    return out


def read_index(directory: str | Path):
    p = Path(directory) / "index.json"
    return load_json(p) if p.exists() else None

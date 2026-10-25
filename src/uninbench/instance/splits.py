"""Tuning/test splits, stratified by the varied factor (R8)."""

from __future__ import annotations

import hashlib
import math
from collections import defaultdict
from pathlib import Path

from ..util import dump_json, load_json
from .model import Instance


# the factor each instance family varies (Table: instance families of the benchmark)
DEFAULT_STRATIFY = {
    "IF1": "holes",
    "IF2": "holes",
    "IF3": "furniture",  # empty/furnished variants of a room are paired and always share a split
    "IF4": "graph_density",
    "IF5": "width,obstacle_density,obstacle_shape,nonconvexity",
}


def _factor(meta: dict, key: str):
    fac = meta.get("factors", {})
    return fac.get(key, meta.get(key))


def _is_number(v) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def _hash(seed, key) -> str:
    return hashlib.sha256(f"{seed}:{key}".encode()).hexdigest()


def assign_splits(instances: list[Instance], tuning_fraction: float = 0.2, stratify: str | None = "auto",
                  seed: int = 0) -> dict[str, str]:
    """Deterministically assign each instance to 'tuning' or 'test', stratified by the varied factor.

    ``stratify`` is a factor name, several comma-separated names, ``"auto"``
    (the factor each family varies, :data:`DEFAULT_STRATIFY`) or None.  A single
    numeric factor is stratified by systematic sampling along its sorted values,
    so that every part of its range is represented in both splits even when
    every value occurs once (e.g. the 21 SafeLog densities); categorical or
    combined factors are split within every stratum.  Paired variants (instances
    sharing ``metadata["pair"]``) are always assigned to the same split.
    """
    groups: dict[str, list[Instance]] = defaultdict(list)
    for inst in instances:
        groups[str(inst.metadata.get("pair", inst.id))].append(inst)
    by_family = defaultdict(list)
    for gkey, members in groups.items():
        by_family[members[0].family].append((gkey, members))
    out = {}
    for fam, units in sorted(by_family.items()):
        key = DEFAULT_STRATIFY.get(fam) if stratify == "auto" else stratify
        names = [k.strip() for k in key.split(",")] if key else []
        vals = [tuple(_factor(m[0].metadata, k) for k in names) for _, m in units]
        units = [u for u, _ in sorted(zip(units, vals), key=lambda t: _hash(seed, t[0][0]))]
        vals = [tuple(_factor(m[0].metadata, k) for k in names) for _, m in units]
        if len(names) == 1 and all(_is_number(v[0]) for v in vals) and len(set(vals)) > 1:
            order = sorted(range(len(units)), key=lambda i: (vals[i][0], _hash(seed, units[i][0])))
            n_tune = int(math.floor(tuning_fraction * len(units) + 0.5))
            offset = int(_hash(seed, fam)[:8], 16) / 16 ** 8
            pick = {order[min(len(order) - 1, int(math.floor((k + offset) * len(order) / n_tune)))]
                    for k in range(n_tune)} if n_tune else set()
            chosen = {i for i in range(len(units)) if i in pick}
        else:
            strata = defaultdict(list)
            for i, v in enumerate(vals):
                strata[str(v)].append(i)
            chosen = set()
            for _, idx in sorted(strata.items()):
                n_tune = int(math.floor(tuning_fraction * len(idx) + 0.5))
                chosen.update(idx[:n_tune])
        for i, (_, members) in enumerate(units):
            for inst in members:
                out[inst.id] = "tuning" if i in chosen else "test"
    return out


def split_directory(directory: str | Path, tuning_fraction: float = 0.2, stratify: str | None = "auto",
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

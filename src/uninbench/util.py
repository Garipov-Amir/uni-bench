"""Small shared helpers: hashing, JSON conversion, angles."""

from __future__ import annotations

import base64
import gzip
import hashlib
import json
import math
import os
import uuid
import zlib
from pathlib import Path
from typing import Any

import numpy as np


def to_jsonable(obj: Any) -> Any:
    """Recursively convert numpy scalars/arrays and tuples into JSON types."""
    if isinstance(obj, dict):
        return {str(k): to_jsonable(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [to_jsonable(v) for v in obj]
    if isinstance(obj, np.ndarray):
        return to_jsonable(obj.tolist())
    if isinstance(obj, (np.integer,)):
        return int(obj)
    if isinstance(obj, (np.floating,)):
        v = float(obj)
        return v if math.isfinite(v) else (None if math.isnan(v) else ("inf" if v > 0 else "-inf"))
    if isinstance(obj, float):
        if math.isfinite(obj):
            return obj
        return None if math.isnan(obj) else ("inf" if obj > 0 else "-inf")
    if isinstance(obj, (np.bool_,)):
        return bool(obj)
    if isinstance(obj, Path):
        return str(obj)
    if isinstance(obj, (set, frozenset)):
        return sorted(to_jsonable(v) for v in obj)
    return obj


def canonical_json(obj: Any) -> str:
    return json.dumps(to_jsonable(obj), sort_keys=True, separators=(",", ":"))


def content_hash(obj: Any, n: int = 16) -> str:
    return hashlib.sha256(canonical_json(obj).encode("utf8")).hexdigest()[:n]


def wrap_angle(a):
    """Wrap angle(s) to [-pi, pi)."""
    return (np.asarray(a) + np.pi) % (2 * np.pi) - np.pi


def dump_json(obj: Any, path: str | Path, indent: int | None = 1) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    data = json.dumps(to_jsonable(obj), indent=indent)
    if path.suffix == ".gz":
        with gzip.open(path, "wt", encoding="utf8") as fh:
            fh.write(data)
    else:
        # unique temporary name: several worker processes may write the same file concurrently
        tmp = path.with_name(f".{path.name}.{os.getpid()}.{uuid.uuid4().hex[:8]}.tmp")
        tmp.write_text(data, encoding="utf8")
        os.replace(tmp, path)


def load_json(path: str | Path) -> Any:
    path = Path(path)
    if path.suffix == ".gz":
        with gzip.open(path, "rt", encoding="utf8") as fh:
            return json.load(fh)
    return json.loads(path.read_text(encoding="utf8"))


def pack_mask(mask: np.ndarray) -> dict:
    """Compact, JSON-safe encoding of a boolean raster."""
    mask = np.asarray(mask, dtype=bool)
    raw = np.packbits(mask.ravel()).tobytes()
    return {"shape": list(mask.shape), "bits": base64.b64encode(zlib.compress(raw, 6)).decode("ascii")}


def unpack_mask(d: dict) -> np.ndarray:
    shape = tuple(d["shape"])
    raw = zlib.decompress(base64.b64decode(d["bits"]))
    bits = np.unpackbits(np.frombuffer(raw, dtype=np.uint8))[: int(np.prod(shape))]
    return bits.astype(bool).reshape(shape)


def pack_array(arr: np.ndarray) -> dict:
    arr = np.ascontiguousarray(arr)
    return {
        "dtype": str(arr.dtype),
        "shape": list(arr.shape),
        "data": base64.b64encode(zlib.compress(arr.tobytes(), 6)).decode("ascii"),
    }


def unpack_array(d: dict) -> np.ndarray:
    raw = zlib.decompress(base64.b64decode(d["data"]))
    return np.frombuffer(raw, dtype=np.dtype(d["dtype"])).reshape(tuple(d["shape"])).copy()


def seed_list(n: int, base: int = 20240917) -> list[int]:
    """The fixed list of seeds shared by all stochastic planners (R11)."""
    ss = np.random.SeedSequence(base)
    return [int(s) for s in ss.generate_state(max(n, 1), dtype=np.uint32)][:n]

"""Representation adapters, one per representation family (Chapter: families)."""

from .base import DEFAULT_EPS_MAX, Adapter, AdapterRecord, Representation, fidelity
from .geometric import GeometricAdapter, PolygonRepresentation
from .graph import GraphAdapter
from .grid import GridAdapter, GridRepresentation
from .swath import SwathAdapter, SwathRepresentation
from .target import TargetAdapter, TargetRepresentation

ADAPTERS = {
    "grid": GridAdapter,
    "graph": GraphAdapter,
    "geometric": GeometricAdapter,
    "swath": SwathAdapter,
    "target": TargetAdapter,
}


def make_adapter(name: str, **params) -> Adapter:
    try:
        cls = ADAPTERS[name]
    except KeyError as e:
        raise KeyError(f"unknown adapter {name!r}; available: {sorted(ADAPTERS)}") from e
    return cls(**params)


__all__ = [
    "ADAPTERS", "DEFAULT_EPS_MAX", "Adapter", "AdapterRecord", "Representation", "fidelity", "make_adapter",
    "GeometricAdapter", "PolygonRepresentation", "GraphAdapter", "GridAdapter", "GridRepresentation",
    "SwathAdapter", "SwathRepresentation", "TargetAdapter", "TargetRepresentation",
]

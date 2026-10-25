"""MCPP instances of Tang & Ma (2023), used to validate the tree-cover planners against
published results (R9).

The instances are distributed with the authors' code (github.com/reso1/MIP-MCPP,
``data/instances/*.istc``) as pickled NetworkX grid graphs with root vertices.
They are read with a restricted unpickler that only resolves the few classes the
files contain, and embedded like the SafeLog graphs: every terrain-graph vertex
becomes a coarse cell of 2 x 2 unit sub-cells (robot width 1), missing edges
between neighbouring vertices become thin walls, and every robot starts in the
lower-left sub-cell of its root vertex and has to return there.

``PUBLISHED`` holds the coverage times (in the paper's unit: the sum of the
visited decomposed-vertex weights) reported in Table III of the paper for MFC,
MSTC* and the MIP-based planner.
"""

from __future__ import annotations

import copy
import importlib
import pickle
from pathlib import Path

from .safelog import graph_to_instance

# Tang & Ma (2023), Table III: coverage time ("ct") per instance and planner
PUBLISHED = {
    "floor_small": {"mfc": 23.0, "mstc_star": 21.0, "mip": 16.0, "k": 4, "weighted": False},
    "maze_small": {"mfc": 14.0, "mstc_star": 36.0, "mip": 11.0, "k": 6, "weighted": False},
    "terrain_small": {"mfc": 36.12, "mstc_star": 36.98, "mip": 28.41, "k": 8, "weighted": True},
    "floor_medium": {"mfc": 55.0, "mstc_star": 47.5, "mip": 29.0, "k": 12, "weighted": False},
    "terrain_medium": {"mfc": 368.2, "mstc_star": 269.5, "mip": 248.8, "k": 4, "weighted": True},
    "maze_medium": {"mfc": 79.0, "mstc_star": 65.0, "mip": 52.5, "k": 6, "weighted": False},
    "floor_large": {"mfc": 294.0, "mstc_star": 212.5, "mip": 208.0, "k": 4, "weighted": False},
    "maze_large": {"mfc": 105.0, "mstc_star": 139.5, "mip": 91.5, "k": 8, "weighted": False},
    "terrain_large_1": {"mfc": 597.4, "mstc_star": 468.6, "mip": 436.7, "k": 4, "weighted": True},
    "terrain_large_2": {"mfc": 575.9, "mstc_star": 487.7, "mip": 454.4, "k": 4, "weighted": True},
}

_ALLOWED = {
    ("networkx.classes.graph", "Graph"), ("networkx.classes.coreviews", "AdjacencyView"),
    ("networkx.classes.reportviews", "NodeView"), ("networkx.classes.reportviews", "EdgeView"),
    ("networkx.classes.reportviews", "DegreeView"), ("builtins", "dict"), ("builtins", "set"),
    ("builtins", "list"), ("builtins", "tuple"), ("numpy", "dtype"), ("numpy.core.numeric", "_frombuffer"),
    ("numpy._core.numeric", "_frombuffer"), ("numpy.core.multiarray", "scalar"),
    ("numpy._core.multiarray", "scalar"),
}


class _Record:
    """Stand-in for MIP_MCPP.instance.Instance (attributes only)."""


class _Unpickler(pickle.Unpickler):
    def find_class(self, module, name):
        if (module, name) == ("MIP_MCPP.instance", "Instance"):
            return _Record
        if (module, name) not in _ALLOWED:
            raise pickle.UnpicklingError(f"refusing to load {module}.{name}")
        if module.startswith("numpy.core"):
            try:
                return getattr(importlib.import_module(module.replace("numpy.core", "numpy._core")), name)
            except (ImportError, AttributeError):
                pass
        return getattr(importlib.import_module(module), name)


def read_istc(path):
    with open(path, "rb") as fh:
        return _Unpickler(fh).load()


def key_of(path: Path) -> str:
    return Path(path).stem.split("-")[0]


def istc_to_instance(path, cell_size: float = 2.0):
    rec = read_istc(path)
    G = rec.G
    pos = {v: tuple(int(c) for c in d["pos"]) for v, d in G.nodes(data=True)}
    cols = max(x for x, _ in pos.values()) + 1
    rows = max(y for _, y in pos.values()) + 1
    vid = {v: y * cols + x for v, (x, y) in pos.items()}
    edges = sorted({(min(vid[u], vid[v]), max(vid[u], vid[v])) for u, v in G.edges()})
    weights = {vid[v]: float(d.get("terrain_weight", 1.0)) for v, d in G.nodes(data=True)}
    key = key_of(path)
    inst = graph_to_instance(f"mipmcpp_{key}", rows, cols, edges, cell_size, robot_width=cell_size / 2,
                             vertices=sorted(vid.values()), source="MIP-MCPP", dataset="Tang & Ma (2023)",
                             published=PUBLISHED.get(key), instance_file=Path(path).name)
    inst.metadata["graph"]["vertex_weights"] = {str(k): v for k, v in weights.items()}
    template = inst.team[0]
    team = []
    for k, r in enumerate(rec.R):
        x, y = pos[r]
        robot = copy.deepcopy(template)
        robot.id = f"r{k}"
        robot.start = ((x + 0.25) * cell_size, (y + 0.25) * cell_size, 0.0)
        robot.depot = (robot.start[0], robot.start[1])
        team.append(robot)
    inst.team = team
    inst.task.return_to_depot = True
    inst.family = "published"
    inst.__dict__.pop("hash", None)
    return inst


def import_mip_mcpp(src, out_dir=None):
    """Import every ``*.istc`` file of the MIP-MCPP repository (``data/instances``)."""
    src = Path(src)
    files = sorted(src.glob("*.istc")) or sorted(src.glob("data/instances/*.istc"))
    out = []
    for f in files:
        inst = istc_to_instance(f)
        out.append(inst)
        if out_dir is not None:
            inst.save(Path(out_dir))
    return out


__all__ = ["PUBLISHED", "import_mip_mcpp", "istc_to_instance", "read_istc"]

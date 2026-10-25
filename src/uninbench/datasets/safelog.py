"""IF4: SafeLog graphs (Rybecky & Kulich, 2021) and graph-to-geometry embedding.

SafeLog maps are subgraphs of a 4-connected grid over a common vertex set.
Each vertex is embedded as a square cell and every missing edge between two
neighbouring vertices becomes a thin wall on the boundary between the two
cells.  The original graph is kept in the metadata so that graph-native
planners receive it unchanged through the identity (``graph``) adapter.

Accepted inputs:

* the published dataset (Zenodo, doi:10.5281/zenodo.4439404): a directory
  with ``test_nodes.txt`` (``id x y`` per line) and ``test_edges<X>.txt``
  (``id id`` per line, both directions listed), X = 0 (spanning tree) ... 20;
* plain edge lists, one edge per line (``#`` comments allowed): ``u v`` with
  row-major vertex ids, or ``r1 c1 r2 c2``.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
from shapely import unary_union
from shapely.geometry import box

from ..geometry.footprint import Footprint
from ..instance.model import Environment, Instance, Robot


def lattice_edges(rows: int, cols: int) -> list[tuple[int, int]]:
    out = []
    for r in range(rows):
        for c in range(cols):
            v = r * cols + c
            if c + 1 < cols:
                out.append((v, v + 1))
            if r + 1 < rows:
                out.append((v, v + cols))
    return out


def graph_to_instance(iid: str, rows: int, cols: int, edges, cell_size: float = 2.0, robot_width: float | None = None,
                      wall: float = 0.02, vertices=None, **meta) -> Instance:
    """Embed a 4-grid subgraph as cells with thin walls for missing edges."""
    vertices = sorted(set(range(rows * cols)) if vertices is None else set(int(v) for v in vertices))
    vset = set(vertices)
    eset = {tuple(sorted((int(a), int(b)))) for a, b in edges}
    t = wall * cell_size
    cells = [box((v % cols) * cell_size, (v // cols) * cell_size, (v % cols + 1) * cell_size,
                 (v // cols + 1) * cell_size) for v in vertices]
    boundary = unary_union(cells)
    walls = []
    for a, b in lattice_edges(rows, cols):
        if a in vset and b in vset and (a, b) not in eset:
            ra, ca = divmod(a, cols)
            if b == a + 1:  # vertical wall between columns ca and ca+1
                x = (ca + 1) * cell_size
                walls.append(box(x - t / 2, ra * cell_size, x + t / 2, (ra + 1) * cell_size))
            else:  # horizontal wall between rows ra and ra+1
                y = (ra + 1) * cell_size
                walls.append(box(ca * cell_size, y - t / 2, (ca + 1) * cell_size, y + t / 2))
    env = Environment(boundary=boundary, holes=walls)
    w = robot_width if robot_width is not None else cell_size / 2
    v0 = vertices[0]
    start = ((v0 % cols + 0.25) * cell_size, (v0 // cols + 0.25) * cell_size, 0.0)
    # the body is narrower than the tool by the wall thickness so that the sub-cells next to
    # a (thin but finite) wall remain drivable; the tool keeps the nominal width
    robot = Robot(id="r0", start=start, tool=Footprint("disc", w), body=Footprint("disc", w - 2 * t))
    density = len(eset) / max(1, len(lattice_edges(rows, cols)))
    meta = {"source": "safelog", "graph": {"rows": rows, "cols": cols, "cell_size": cell_size, "origin": [0.0, 0.0],
                                           "vertices": vertices, "edges": sorted(eset)},
            "factors": {"graph_density": round(density, 4)}, **meta}
    return Instance(id=iid, environment=env, team=[robot], family="IF4", metadata=meta)


def parse_edges(text: str, cols: int) -> list[tuple[int, int]]:
    out = []
    for line in text.splitlines():
        line = line.split("#", 1)[0].strip().replace(",", " ")
        if not line:
            continue
        v = [int(float(x)) for x in line.split()]
        if len(v) == 2:
            out.append((v[0], v[1]))
        elif len(v) == 4:
            out.append((v[0] * cols + v[1], v[2] * cols + v[3]))
        else:
            raise ValueError(f"cannot parse edge line {line!r}")
    return out


def read_nodes(path) -> dict[int, tuple[int, int]]:
    """``id x y`` lines -> {id: (row, col)} with row = y and col = x."""
    out = {}
    for line in Path(path).read_text().splitlines():
        line = line.split("#", 1)[0].strip()
        if not line:
            continue
        i, x, y = (int(float(v)) for v in line.replace(",", " ").split()[:3])
        out[i] = (y, x)
    return out


def import_safelog(src: str | Path, out_dir: str | Path | None = None, rows: int = 20, cols: int = 20,
                   cell_size: float = 2.0, robot_width: float | None = None, pattern: str | None = None):
    """Import SafeLog maps: the published directory layout (``test_nodes.txt`` +
    ``test_edges<X>.txt``) or plain edge lists."""
    src = Path(src)
    nodes_file = (src / "test_nodes.txt") if src.is_dir() else None
    if nodes_file is not None and not nodes_file.exists():
        nested = sorted(src.glob("*/test_nodes.txt"))
        if nested:
            src, nodes_file = nested[0].parent, nested[0]
    out = []
    if nodes_file is not None and nodes_file.exists():
        nodes = read_nodes(nodes_file)
        rows = max(r for r, _ in nodes.values()) + 1
        cols = max(c for _, c in nodes.values()) + 1
        # the 21 published maps test_edges0 (spanning tree) ... test_edges20; the extra unnumbered
        # test_edges.txt (complete grid) is included only on request (pattern="test_edges*.txt")
        files = sorted(src.glob(pattern or "test_edges[0-9]*.txt"), key=_edge_file_key)
        for f in files:
            edges = set()
            for u, v in parse_edges(f.read_text(), cols):
                (ru, cu), (rv, cv) = nodes[u], nodes[v]
                if abs(ru - rv) + abs(cu - cv) != 1:
                    raise ValueError(f"{f.name}: edge ({u},{v}) is not between 4-neighbours")
                a, b = ru * cols + cu, rv * cols + cv
                edges.add((min(a, b), max(a, b)))
            level = _edge_file_key(f)[0]
            inst = graph_to_instance(f"safelog_{f.stem}", rows, cols, sorted(edges), cell_size, robot_width,
                                     source_file=f.name, source="safelog", dataset="doi:10.5281/zenodo.4439404",
                                     map_level=level)
            out.append(inst)
            if out_dir is not None:
                inst.save(Path(out_dir))
        return out
    files = [src] if src.is_file() else sorted(src.glob(pattern or "*.txt"))
    for f in files:
        inst = graph_to_instance(f"safelog_{f.stem}", rows, cols, parse_edges(f.read_text(), cols), cell_size,
                                 robot_width, source_file=f.name)
        out.append(inst)
        if out_dir is not None:
            inst.save(Path(out_dir))
    return out


def _edge_file_key(f: Path):
    """test_edges0 ... test_edges20 in density order; the unnumbered file is the full grid."""
    digits = "".join(ch for ch in f.stem if ch.isdigit())
    return (int(digits) if digits else 21, f.name)


def random_connected_subgraph(rows: int, cols: int, density: float, rng) -> list[tuple[int, int]]:
    """Spanning tree plus random extra lattice edges up to the requested edge density."""
    all_edges = lattice_edges(rows, cols)
    n = rows * cols
    order = rng.permutation(len(all_edges))
    parent = list(range(n))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    tree, rest = [], []
    for k in order:
        a, b = all_edges[k]
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[ra] = rb
            tree.append((a, b))
        else:
            rest.append((a, b))
    target = max(len(tree), int(round(density * len(all_edges))))
    return tree + rest[: max(0, target - len(tree))]


def synthetic_safelog(out_dir=None, rows: int = 20, cols: int = 20, densities=None, count: int = 1, seed: int = 0,
                      cell_size: float = 2.0, robot_width: float | None = None):
    """SafeLog-like connectivity sweep: random connected subgraphs of a rows x cols grid."""
    if densities is None:
        n_tree = rows * cols - 1
        full = len(lattice_edges(rows, cols))
        densities = list(np.round(np.linspace(n_tree / full, 1.0, 21), 4))
    out = []
    for k, d in enumerate(densities):
        for i in range(count):
            rng = np.random.default_rng([seed, k, i])
            edges = random_connected_subgraph(rows, cols, float(d), rng)
            inst = graph_to_instance(f"safelog_syn_d{d:.3f}_{i}", rows, cols, edges, cell_size, robot_width,
                                     generator={"kind": "safelog_synthetic", "seed": [seed, k, i],
                                                "density": float(d)})
            out.append(inst)
            if out_dir is not None:
                inst.save(Path(out_dir))
    return out


def graph_density(inst: Instance) -> float:
    g = inst.metadata["graph"]
    return len(g["edges"]) / max(1, len(lattice_edges(g["rows"], g["cols"])))


__all__ = ["graph_to_instance", "import_safelog", "synthetic_safelog", "random_connected_subgraph", "parse_edges",
           "lattice_edges", "graph_density"]

"""Replay simulator: a self-contained HTML player built from stored run records.

The simulator reads the same stored records as the reporting protocol -- the
re-timed trajectories, the evaluator's sweep events (entry time of every sweep
of every evaluation cell), the adapter reconstruction and the planners'
intermediate structures -- and never computes metrics itself, so the visual and
the numerical results cannot diverge.  Several planners on the same instance
are shown side by side on a shared clock.
"""

from __future__ import annotations

import base64
import json
from pathlib import Path

import numpy as np
import shapely

from ..geometry.decomposition import polygons_of
from ..geometry.raster import RasterGrid
from ..instance.model import Instance
from ..reconstruction import Reconstruction
from ..runner.experiment import load_runs
from ..util import load_json, unpack_array, unpack_mask

TEMPLATE = Path(__file__).with_name("player.html")


def _b64(arr: np.ndarray) -> str:
    return base64.b64encode(np.ascontiguousarray(arr).tobytes()).decode("ascii")


def _poly_coords(geom, nd=4):
    out = []
    for p in polygons_of(geom):
        out.append([np.round(np.asarray(r.coords), nd).tolist() for r in [p.exterior, *p.interiors]])
    return out


def _display_grid(grid: RasterGrid, max_cells: int):
    f = max(1, int(np.ceil(max(grid.nx, grid.ny) / max_cells)))
    return f, RasterGrid(grid.x0, grid.y0, grid.res * f, int(np.ceil(grid.nx / f)), int(np.ceil(grid.ny / f)))


def _events(cov: dict, max_cells: int):
    grid = RasterGrid.from_dict(cov["grid"])
    f, disp = _display_grid(grid, max_cells)
    ev = cov.get("events")
    if not ev:
        return disp, None
    cell = unpack_array(ev["cell"]).astype(np.int64)
    t = unpack_array(ev["t"]).astype(np.float32)
    rob = unpack_array(ev["robot"]).astype(np.int16)
    i, j = cell // grid.nx, cell % grid.nx
    # nearest-neighbour sampling: keep the evaluation cell containing each display-cell centre
    rep_i = (i // f) * f + f // 2
    rep_j = (j // f) * f + f // 2
    keep = (i == np.minimum(rep_i, grid.ny - 1)) & (j == np.minimum(rep_j, grid.nx - 1))
    di, dj = i[keep] // f, j[keep] // f
    dcell = (di * disp.nx + dj).astype(np.uint32)
    return disp, {"cell": _b64(dcell), "t": _b64(t[keep]), "robot": _b64(rob[keep].astype(np.uint8)),
                  "n": int(keep.sum())}


def _layer(name: str, layer: dict):
    kind = layer.get("type")
    if kind == "raster":
        g = RasterGrid.from_dict(layer["grid"])
        mask = unpack_mask(layer["mask"])
        return {"name": name, "type": "cells", "grid": g.to_dict(), "cells": np.flatnonzero(mask.ravel()).tolist()}
    if kind == "labels":
        g = RasterGrid.from_dict(layer["grid"])
        lab = unpack_array(layer["labels"]).ravel()
        idx = np.flatnonzero(lab >= 0)
        return {"name": name, "type": "labels", "grid": g.to_dict(), "cells": idx.tolist(),
                "labels": lab[idx].astype(int).tolist()}
    if kind == "polygons":
        polys = []
        for w in layer["wkt"]:
            polys += _poly_coords(shapely.from_wkt(w))
        return {"name": name, "type": "polygons", "polys": polys}
    if kind == "lines":
        return {"name": name, "type": "lines", "lines": layer["coords"]}
    if kind == "segments":
        return {"name": name, "type": "lines", "lines": layer["coords"]}
    if kind == "points":
        return {"name": name, "type": "points", "points": layer["coords"]}
    if kind == "point_labels":
        return {"name": name, "type": "points", "points": layer["coords"], "labels": layer["labels"]}
    if kind == "pass_assignment":
        lab = {}
        for k, (rid, idxs) in enumerate(layer["robots"].items()):
            for i in idxs:
                lab[int(i)] = k
        return {"name": name, "type": "lines", "lines": layer["coords"],
                "labels": [lab.get(i, -1) for i in range(len(layer["coords"]))]}
    return None


def _panel(results_dir: Path, rec: dict, max_cells: int) -> dict:
    arts = rec.get("artifacts") or {}
    traj = load_json(results_dir / arts["trajectories"]) if arts.get("trajectories") else {}
    cov = load_json(results_dir / arts["coverage"]) if arts.get("coverage") else None
    robots = []
    for t in (traj.get("retimed") or {}).get("trajectories", []):
        xy = np.asarray(t["xy"], dtype=float)
        robots.append({"id": t["robot_id"], "xy": np.round(xy, 4).tolist(), "t": np.round(t["t"], 4).tolist(),
                       "active": t["active"]})
    layers = []
    for src in ("adapter_layers", "layers"):
        for name, lay in (traj.get(src) or {}).items():
            L = _layer(name, lay)
            if L is not None and not any(x["name"] == name for x in layers):
                layers.append(L)
    if traj.get("reconstruction"):
        rc = Reconstruction.from_dict(traj["reconstruction"])
        g = rc.geometry()
        if g is not None and not g.is_empty:
            layers.append({"name": "reconstruction (F̂)", "type": "polygons", "polys": _poly_coords(g.simplify(1e-3))})
    disp, events = _events(cov, max_cells) if cov else (None, None)
    m = rec.get("metrics") or {}
    shown = {k: m.get(k) for k in ("coverage_ratio", "overlap_ratio", "makespan", "normalized_makespan",
                                   "path_length", "turns", "workload_imbalance", "conflict_instants")}
    shown["planning_time"] = rec.get("planning_time")
    return {"planner": rec.get("planner"), "run_id": rec.get("run_id"), "outcome": rec.get("outcome"),
            "seed": rec.get("seed"), "config": rec.get("config_label"), "metrics": shown, "robots": robots,
            "layers": layers, "grid": disp.to_dict() if disp else None, "events": events,
            "composition": bool(rec.get("composition_of"))}


def build_replay(results_dir, instance_id: str | None = None, planners=None, out=None, max_cells: int = 320,
                 seed: int | None = None) -> Path:
    results_dir = Path(results_dir)
    recs = [r for r in load_runs(results_dir) if (r.get("artifacts") or {}).get("trajectories")]
    if not recs:
        raise ValueError("no stored trajectories in this results directory")
    if instance_id is None:
        counts = {}
        for r in recs:
            counts[r["instance_id"]] = counts.get(r["instance_id"], 0) + 1
        instance_id = max(counts, key=counts.get)
    recs = [r for r in recs if r["instance_id"] == instance_id]
    if planners:
        recs = [r for r in recs if r["planner"] in planners]
    chosen = {}
    for r in sorted(recs, key=lambda r: (r["planner"], r.get("seed", 0))):
        if seed is not None and r.get("seed") != seed and r.get("stochastic"):
            continue
        chosen.setdefault(r["planner"], r)
    if not chosen:
        raise ValueError(f"no runs with trajectories for instance {instance_id}")
    inst = Instance.load(results_dir / next(iter(chosen.values()))["artifacts"]["instance"])
    env = inst.environment
    data = {
        "instance": {"id": inst.id, "family": inst.family, "bounds": list(env.bounds),
                     "free": _poly_coords(inst.free), "radius": [r.body_fp.radius for r in inst.team],
                     "tool": [r.tool.to_dict() for r in inst.team]},
        "panels": [_panel(results_dir, r, max_cells) for r in chosen.values()],
    }
    payload = json.dumps(data, separators=(",", ":")).replace("</", "<\\/")
    html = TEMPLATE.read_text(encoding="utf8").replace("__DATA__", payload)
    html = html.replace("__TITLE__", f"Replay {inst.id}")
    out = Path(out) if out else results_dir / "replay" / f"{instance_id}.html"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(html, encoding="utf8")
    return out

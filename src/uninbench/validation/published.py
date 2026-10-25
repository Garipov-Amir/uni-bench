"""Validation against published results (R9).

The tree-cover planners are run on the instances of Tang & Ma (2023), for which
Tables III of Tang & Ma (2023) and Table 2 of Tang & Ma (2024) report coverage
times of MFC, MSTC*, the MIP planner and LS-MCPP.  Coverage times are compared
in the papers' unit (sum of the visited decomposed-vertex weights, see
:mod:`uninbench.planners.tree_cover`).

Declared tolerances -- they account for tie-breaking in spanning trees, the
choice of the start sub-cell, and solver differences:

* MFC, MSTC*: coverage time at most 10 % above the published value on at least
  80 % of the unweighted instances (a reimplementation must not handicap the
  algorithm; results better than published are reported, not rejected);
* LS-MCPP: within +-10 % of the published mean on at least 75 % of the instances;
* MIP: (a) the published optimal tree covers are feasible in our model and give
  exactly the published coverage times (formulation check, independent of the
  solver), and (b) the full model solves ``maze_small`` to the published optimum.
  The paper solved larger models with Gurobi (16 threads, up to 24 h); HiGHS
  with a benchmark time limit does not reach those values, which is a solver
  difference recorded in the report.

The instances are read from the authors' repository (``scripts/fetch_datasets.sh``
clones it to ``datasets/MIP-MCPP``; ``UNINBENCH_MIP_MCPP`` overrides the path).
"""

from __future__ import annotations

import glob
import os
import time
from pathlib import Path

import numpy as np

from ..datasets.mip_mcpp import PUBLISHED, _Unpickler, istc_to_instance, key_of, read_istc

# Tang & Ma (2024), Table 2: LS-MCPP makespan (mean over 12 seeds) on the instances of Tang & Ma (2023)
PUBLISHED_LS_MCPP = {"floor_small": 16.75, "terrain_medium": 244.5, "maze_medium": 54.3, "floor_large": 192.1,
                     "maze_large": 97.3, "terrain_large_1": 429.6}
UNWEIGHTED = ["floor_small", "maze_small", "floor_medium", "maze_medium", "floor_large", "maze_large"]
PLANNERS_WITH_PUBLISHED = ("mfc", "mstc_star", "mip", "ls_mcpp")


def data_dir(explicit=None) -> Path | None:
    for c in (explicit, os.environ.get("UNINBENCH_MIP_MCPP"), "datasets/MIP-MCPP"):
        if c and (Path(c) / "data" / "instances").is_dir():
            return Path(c)
    return None


def _instance_file(root: Path, key: str) -> str | None:
    f = sorted(glob.glob(str(root / "data" / "instances" / f"{key}-*.istc")))
    return f[0] if f else None


def _run(name: str, key: str, root: Path, params=None, time_limit: float = 120.0, seed: int = 0):
    from ..adapters import make_adapter
    from ..planners import PlanContext, get_planner
    from ..planners.execution import run_planner

    inst = istc_to_instance(_instance_file(root, key))
    man = get_planner(name).manifest
    rep, _ = make_adapter("graph", coarse=True)(inst)
    t0 = time.perf_counter()
    res = run_planner(get_planner(name)(params or {}), rep.fresh(),
                      PlanContext(seed=seed, time_limit=time_limit, return_to_depot=True))
    return float(max(res.info["coverage_time"].values())), time.perf_counter() - t0, res, man


def _table_case(name: str, published: dict, keys, root: Path, rule: str, tol: float, share: float,
                time_limit: float) -> dict:
    rows = []
    for key in keys:
        if key not in published or _instance_file(root, key) is None:
            continue
        ct, dt, _, _ = _run(name, key, root, time_limit=time_limit)
        pub = float(published[key])
        rel = (ct - pub) / pub
        ok = rel <= tol if rule == "not_worse" else abs(rel) <= tol
        rows.append({"instance": key, "value": ct, "published": pub, "rel_diff": rel, "ok": bool(ok),
                     "time": dt})
    passed = bool(rows) and np.mean([r["ok"] for r in rows]) >= share
    return {"case": f"{name}_published_coverage_times", "planner": name, "passed": passed,
            "source": f"published coverage times; rule: {rule} within {tol:.0%} on >= {share:.0%} of instances",
            "results": rows, "errors": [] if rows else ["no published instances found"]}


def _mip_formulation_case(root: Path) -> dict:
    from ..adapters import make_adapter
    from ..planners.mip import RMMTCModel
    from ..planners.tree_cover import coarse_graph, coverage_time, tree_tour

    rows = []
    for key in ("floor_small", "maze_small", "floor_medium"):
        f = _instance_file(root, key)
        sol_files = sorted(glob.glob(str(root / "data" / "solutions" / f"{key}-*.solu")))
        if f is None or not sol_files:
            continue
        rec = read_istc(f)
        inst = istc_to_instance(f)
        rep, _ = make_adapter("graph", coarse=True)(inst)
        cg = coarse_graph(rep)
        pos = {v: tuple(int(c) for c in d["pos"]) for v, d in rec.G.nodes(data=True)}
        cols = max(x for x, _ in pos.values()) + 1
        flat = {v: y * cols + x for v, (x, y) in pos.items()}
        eidx = {(int(u), int(v)): e for e, (u, v) in enumerate(cg.edges)}
        with open(sol_files[0], "rb") as fh:
            sol = _Unpickler(fh).load()
        trees = []
        for i, edges in enumerate(sol):
            te, verts = [], {cg.roots[i]}
            for a, b in edges:
                u, v = cg.index[flat[a]], cg.index[flat[b]]
                te.append(eidx[(min(u, v), max(u, v))])
                verts |= {u, v}
            trees.append((verts, te))
        m = RMMTCModel(cg.n, cg.edges, cg.w_edge, cg.roots)
        x0 = m.start_vector(trees)
        _, cons, _, bnds = m.build()
        ax = cons.A @ x0
        viol = float(max(np.max(np.maximum(cons.lb - ax, 0) + np.maximum(ax - cons.ub, 0)),
                         np.max(np.maximum(bnds.lb - x0, 0)), np.max(np.maximum(x0 - bnds.ub, 0))))
        ct = max(coverage_time(rep, cg, tree_tour(rep, cg, v, e, cg.start_cells[i])) for i, (v, e) in enumerate(trees))
        pub = PUBLISHED[key]["mip"]
        ok = viol < 1e-6 and abs(ct - pub) < 1e-9
        rows.append({"instance": key, "max_violation": viol, "coverage_time": ct, "published": pub, "ok": bool(ok)})
    return {"case": "mip_formulation_published_optima", "planner": "mip", "passed": bool(rows) and all(r["ok"] for r in rows),
            "source": "Tang & Ma (2023): the published optimal tree covers satisfy our MIP and give the published "
                      "coverage times", "results": rows, "errors": [] if rows else ["no published solutions found"]}


def _mip_optimum_case(root: Path) -> dict:
    if _instance_file(root, "maze_small") is None:
        return {"case": "mip_maze_small_optimum", "planner": "mip", "passed": False, "results": [],
                "errors": ["instance maze_small not found"], "source": ""}
    ct, dt, res, _ = _run("mip", "maze_small", root, params={"heuristic": "none"}, time_limit=120.0)
    pub = PUBLISHED["maze_small"]["mip"]
    return {"case": "mip_maze_small_optimum", "planner": "mip", "passed": abs(ct - pub) < 1e-9,
            "source": "Tang & Ma (2023), Table III: optimal coverage time 11 on maze_small",
            "results": [{"instance": "maze_small", "value": ct, "published": pub, "ok": abs(ct - pub) < 1e-9,
                         "time": dt, "solver": res.info.get("mip")}], "errors": []}


def published_cases(planner: str, root=None) -> dict:
    """Run the published-result validation of a planner: {"available", "cases", "passed"}."""
    if planner not in PLANNERS_WITH_PUBLISHED:
        return {"applicable": False}
    root = data_dir(root)
    if root is None:
        return {"applicable": True, "available": False, "passed": False, "cases": [],
                "reason": "published instances not found: run scripts/fetch_datasets.sh "
                          "(clones github.com/reso1/MIP-MCPP to datasets/MIP-MCPP) or set UNINBENCH_MIP_MCPP"}
    if planner in ("mfc", "mstc_star"):
        published = {k: v[planner] for k, v in PUBLISHED.items()}
        cases = [_table_case(planner, published, UNWEIGHTED, root, "not_worse", 0.10, 0.8, 60.0)]
    elif planner == "ls_mcpp":
        keys = [k for k in PUBLISHED_LS_MCPP if k in UNWEIGHTED]
        cases = [_table_case(planner, PUBLISHED_LS_MCPP, keys, root, "two_sided", 0.10, 0.75, 300.0)]
    else:
        cases = [_mip_formulation_case(root), _mip_optimum_case(root)]
    return {"applicable": True, "available": True, "cases": cases, "passed": all(c["passed"] for c in cases),
            "data": str(root)}


__all__ = ["published_cases", "data_dir", "PUBLISHED_LS_MCPP", "PLANNERS_WITH_PUBLISHED", "key_of"]

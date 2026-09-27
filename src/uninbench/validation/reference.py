"""Planner-independent sanity check of the evaluator (R9).

The AreaCoverage dataset ships reference routes computed by its authors.
Re-evaluating them must yield valid, complete plans; a failure points to the
evaluator (or to the interpretation of the dataset), not to any planner.
"""

from __future__ import annotations

import time
from pathlib import Path

from ..datasets.areacoverage import import_areacoverage, reference_trajectories
from ..evaluator import evaluate
from ..util import dump_json


def check_reference_routes(src, subsets=None, limit: int | None = None, flag_convention: str = "from",
                           out: str | Path | None = None, verbose: bool = True) -> dict:
    rows = []
    t0 = time.time()
    for inst in import_areacoverage(src, subsets=subsets, limit=limit, flag_convention=flag_convention):
        d = Path(inst.metadata["source_path"])
        for mem in inst.metadata.get("reference_routes", []):
            try:
                ri, ts = reference_trajectories(inst, d, mem, flag_convention)
                ev = evaluate(ri, ts, sensitivity=False)
                m = ev.metrics
                row = {"instance": inst.id, "solution": mem, "robots": len(ts), "status": ev.status,
                       "reasons": ev.validity.reasons, "coverage_ratio": m.get("coverage_ratio"),
                       "overlap_ratio": m.get("overlap_ratio"), "path_length": m.get("path_length"),
                       "transit_length": m.get("transit_length"), "turns": m.get("turns"),
                       "static_collision_length": m.get("static_collision_length")}
            except Exception as e:  # keep going; report the failure
                row = {"instance": inst.id, "solution": mem, "status": "error", "reasons": [repr(e)]}
            row["passed"] = row["status"] == "success"
            rows.append(row)
            if verbose:
                print(f"{row['instance']:32s} {mem:16s} {row['status']:10s} "
                      f"coverage={row.get('coverage_ratio') or 0:.4f}", flush=True)
    rep = {"check": "AreaCoverage reference routes", "flag_convention": flag_convention, "rows": rows,
           "passed": bool(rows) and all(r["passed"] for r in rows),
           "summary": f"{sum(r['passed'] for r in rows)}/{len(rows)} reference solutions valid and complete",
           "wall_time": time.time() - t0}
    if out is not None:
        dump_json(rep, out)
    return rep

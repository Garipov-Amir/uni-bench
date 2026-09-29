"""Sweep planners in staged mode.

* ``swath_bf`` (swath family): parallel swaths of the swath adapter are ordered
  greedily (boustrophedon on convex regions), split into contiguous balanced
  chunks, optionally improved by 2-opt, and connected by collision-free transit
  -- Dubins headland turns and depot trips when the robots carry a minimum
  turning radius or a capacity (agricultural profile).
* ``bcd`` (geometric family): boustrophedon cellular decomposition (Choset,
  2000), back-and-forth passes inside every cell, cells visited in
  nearest-neighbour order, multi-robot split at pass granularity.
"""

from __future__ import annotations

import numpy as np

from shapely.geometry import LineString

from ..geometry.decomposition import lines_of, offset_swaths, rotate
from ..trajectory import TrajectorySet
from .base import HyperParam, Manifest, PlanResult, Stage, StagedPlanner
from .deconflict import maybe_deconflict
from .sweep_common import Pass, RouteBuilder, nn_order, skip_order, split_contiguous, two_opt


def _mean(robots, attr):
    return float(np.mean([getattr(r, attr) for r in robots]))


class SwathPasses(Stage):
    kind = "environment_model"
    name = "swath_passes"
    consumes = ("representation",)
    produces = "model"

    def run(self, data, rep, ctx, params):
        passes = [Pass(np.asarray(s["line"].coords), bool(s.get("closed")), int(s.get("row", 0)))
                  for s in rep.swaths]
        return {"passes": passes}

    def summary(self, out):
        return {"passes": len(out["passes"])}


class CellPasses(Stage):
    kind = "environment_model"
    name = "cell_passes"
    consumes = ("representation",)
    produces = "model"

    def run(self, data, rep, ctx, params):
        robot = rep.team[0]
        cs = rep.cspace(robot)
        w = min(r.working_width for r in rep.team)
        angle = rep.angle or 0.0
        cells = rep.cells or [rep.free]
        passes = []
        region_cs = cs.buffer(1e-7 * max(1.0, w))
        if params.get("perimeter", True):  # boundary-following pass around every obstacle and the boundary
            for k, s in enumerate(offset_swaths(cs, w, max_rings=1)):
                passes.append(Pass(np.asarray(s["line"].coords), True, -(k + 1)))
            region_cs = region_cs.difference(cs.boundary.buffer(0.5 * w))
        cs_rot = rotate(region_cs, angle)
        # one global lattice of pass offsets (perpendicular to the passes), so that thin
        # cells created by many critical points do not each receive their own pass
        if region_cs.is_empty:
            return {"passes": passes, "cells": len(cells)}
        _, lo, _, hi = rotate(cs, angle).bounds
        n = int(np.ceil((hi - lo) / w - 1e-6)) + 1
        offsets = np.linspace(lo, hi, n) if n > 1 else np.array([(lo + hi) / 2])
        for c, cell in enumerate(cells):
            crot = rotate(cell, angle)
            region = crot.intersection(cs_rot)
            if region.is_empty:
                continue
            x0, y0, x1, y1 = crot.bounds
            mine = offsets[(offsets >= y0) & ((offsets < y1) | (offsets >= hi - 1e-12))]
            if len(mine) == 0:  # a thin cell between two lattice lines: sweep its middle
                mine = np.array([(max(y0, lo) + min(y1, hi)) / 2])
            for y in mine:
                line = LineString([(x0 - 1, y), (x1 + 1, y)])
                for piece in lines_of(line.intersection(region)):
                    if piece.length > 1e-9:
                        pts = np.asarray(rotate(piece, angle, inverse=True).coords)[[0, -1]]
                        passes.append(Pass(pts, False, c))
        return {"passes": passes, "cells": len(cells)}

    def summary(self, out):
        return {"passes": len(out["passes"]), "cells": out["cells"]}


class NNOrder(Stage):
    kind = "route_construction"
    name = "nn_order"
    consumes = ("model",)
    produces = "tour"

    def run(self, data, rep, ctx, params):
        passes = data["model"]["passes"]
        r_min = max((r.min_turn_radius or 0.0) for r in rep.team)
        pattern = params.get("pattern", "auto")
        if pattern == "skip" or (pattern == "auto" and r_min > 0):
            spacing = getattr(rep, "spacing", None) or min(r.working_width for r in rep.team)
            k = int(np.ceil(2 * r_min / spacing)) + 1 if r_min > 0 else 2
            return skip_order(passes, np.asarray(rep.team[0].start[:2], dtype=float), k)
        return nn_order(passes, rep.team[0].start[:2], _mean(rep.team, "speed"), _mean(rep.team, "turn_cost"))


class CellOrder(Stage):
    """Cells in nearest-neighbour order; inside a cell passes alternate direction."""

    kind = "route_construction"
    name = "cell_order"
    consumes = ("model",)
    produces = "tour"

    def run(self, data, rep, ctx, params):
        passes = data["model"]["passes"]
        groups: dict[int, list[int]] = {}
        for k, p in enumerate(passes):
            groups.setdefault(p.group, []).append(k)
        pos = np.asarray(rep.team[0].start[:2], dtype=float)
        left = set(groups)
        seq = []
        while left:
            best = None
            for g in left:
                idx = groups[g]
                for ends in (idx, idx[::-1]):
                    p0 = passes[ends[0]]
                    for rev in (False, True):
                        d = float(np.hypot(*(p0.start(rev) - pos)))
                        if best is None or d < best[0]:
                            best = (d, g, ends, rev)
            _, g, ends, rev = best
            left.discard(g)
            for k, i in enumerate(ends):
                seq.append((i, rev if k % 2 == 0 else not rev))
            pos = passes[seq[-1][0]].end(seq[-1][1])
        return seq


class ContiguousSplit(Stage):
    kind = "work_distribution"
    name = "contiguous_split"
    consumes = ("tour", "model")
    produces = "assignment"

    def run(self, data, rep, ctx, params):
        out, info = split_contiguous(data["tour"], data["model"]["passes"], rep.team)
        return {"routes": out, "info": info}

    def metrics(self, out, rep):
        return {"estimated_makespan": out["info"]["estimated_makespan"],
                "passes_per_robot": [len(v) for v in out["routes"].values()]}


class TwoOptImprove(Stage):
    kind = "optimization"
    name = "two_opt"
    consumes = ("assignment", "model")
    produces = "routes"

    def run(self, data, rep, ctx, params):
        routes = dict(data["assignment"]["routes"])
        if not params.get("two_opt", True) or any(r.min_turn_radius for r in rep.team):
            return routes  # Euclidean 2-opt would break the turning-radius pattern
        passes = data["model"]["passes"]
        for r in rep.team:
            routes[r.id] = two_opt(routes[r.id], passes, r.speed, r.turn_cost, r.start[:2],
                                   max_rounds=int(params.get("two_opt_rounds", 10)),
                                   deadline=lambda: ctx.out_of_time(0.7))
        return routes


class _SweepPlanner(StagedPlanner):
    def finish(self, data, rep, ctx) -> PlanResult:
        passes = data["model"]["passes"]
        ts = TrajectorySet(metadata={"planner": self.manifest.name})
        fallbacks = skipped = 0
        for r in rep.team:
            rb = RouteBuilder(rep, r, self.manifest, ctx, service_links=bool(self.params.get("service_links", True)))
            ts.add(rb.run(data["routes"].get(r.id, []), passes, return_to_depot=ctx.return_to_depot))
            fallbacks += rb.dubins_fallbacks
            skipped += rb.skipped_pieces
        ts, dinfo = maybe_deconflict(ts, rep, ctx)
        layers = rep.layers()
        assign = {r.id: [int(i) for i, _ in data["routes"].get(r.id, [])] for r in rep.team}
        layers["assignment"] = {"type": "pass_assignment",
                                "coords": [np.round(p.pts, 4).tolist() for p in passes], "robots": assign}
        return PlanResult(ts, native=data["routes"], layers=layers,
                          info={"dubins_fallbacks": fallbacks, "skipped_pieces": skipped, **dinfo})


SERVICE_LINKS = HyperParam("service_links", "bool", True,
                           doc="keep the tool active on short straight links between neighbouring passes")
SWEEP_PARAMS = [
    SERVICE_LINKS,
    HyperParam("pattern", "categorical", "auto", choices=["auto", "nn", "skip"],
               doc="pass order: greedy nearest neighbour, or skip pattern (auto: skip with a turning radius)"),
    HyperParam("two_opt", "bool", True, doc="2-opt improvement of each robot's pass order"),
    HyperParam("two_opt_rounds", "int", 10, low=1, high=50, doc="maximum number of 2-opt improvement rounds"),
]


class SwathBF(_SweepPlanner):
    manifest = Manifest(
        name="swath_bf",
        version="1.0",
        reference="Boustrophedon swath coverage with balanced contiguous splitting (cf. Fields2Cover pipeline, "
                  "Mier et al. 2023)",
        family="swath",
        adapter="swath",
        adapter_params={"mode": "parallel", "angle": "auto"},
        constraints={"multi_robot", "heterogeneous", "capacity", "turning_radius", "return_to_depot",
                     "deconfliction"},
        mode="staged",
        stages=["swath_passes", "nn_order", "contiguous_split", "two_opt"],
        hyperparameters=SWEEP_PARAMS,
        core_modules=["uninbench/planners/sweep.py", "uninbench/planners/sweep_common.py"],
        description="Parallel swaths ordered greedily, split into balanced contiguous chunks, 2-opt improved.",
    )
    stages = [SwathPasses(), NNOrder(), ContiguousSplit(), TwoOptImprove()]


class BCD(_SweepPlanner):
    manifest = Manifest(
        name="bcd",
        version="1.0",
        reference="Choset (2000), Coverage of known spaces: the boustrophedon cellular decomposition",
        family="geometric",
        adapter="geometric",
        adapter_params={"mode": "boustrophedon", "angle": "auto"},
        constraints={"multi_robot", "return_to_depot", "deconfliction"},
        mode="staged",
        stages=["cell_passes", "cell_order", "contiguous_split", "two_opt"],
        hyperparameters=[SERVICE_LINKS, HyperParam("perimeter", "bool", True, doc="boundary-following passes"),
                         HyperParam("two_opt", "bool", False, doc="2-opt improvement of each robot's pass order"),
                         HyperParam("two_opt_rounds", "int", 5, low=1, high=50,
                                    doc="maximum number of 2-opt improvement rounds")],
        core_modules=["uninbench/planners/sweep.py", "uninbench/planners/sweep_common.py"],
        description="Boustrophedon cells swept back and forth; cells ordered greedily; split at pass level.",
    )
    stages = [CellPasses(), CellOrder(), ContiguousSplit(), TwoOptImprove()]

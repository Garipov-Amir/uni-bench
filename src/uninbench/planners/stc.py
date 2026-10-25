"""Spanning Tree Coverage (Gabriely & Rimon, 2001) -- single robot, black-box mode."""

from __future__ import annotations

from ..trajectory import SERVICE
from .base import Manifest, PlanContext, Planner, PlanResult
from .grid_common import (DIRECTION_PARAM, TREE_PARAM, cell_routes_to_trajectories, coarse_component, stc_route,
                          tree_layer)


class STC(Planner):
    manifest = Manifest(
        name="stc",
        version="1.1",
        reference="Gabriely & Rimon (2001), Spanning-tree based coverage of continuous areas by a mobile robot",
        family="grid",
        adapter="grid",
        adapter_params={"coarse": True, "connectivity": 4},
        constraints={"return_to_depot"},
        max_robots=1,
        hyperparameters=[TREE_PARAM, DIRECTION_PARAM],
        core_modules=["uninbench/planners/stc.py", "uninbench/planners/grid_common.py"],
        description="Circumnavigates a spanning tree of 2x2 coarse cells; every sub-cell is visited once.",
    )

    def plan(self, rep, ctx: PlanContext) -> PlanResult:
        r = rep.team[0]
        start = rep.starts[r.id]["cell"]
        nx2 = rep.coarse["grid"].nx
        I, J = rep.coarse_of(start)
        root = int(I) * nx2 + int(J)
        nodes = coarse_component(rep, root)
        tour, tree = stc_route(rep, nodes, start, self.params["tree"], self.params["direction"], ctx.rng,
                               close=ctx.return_to_depot)
        routes = {r.id: [(c, SERVICE) for c in tour[1:]]}
        ts = cell_routes_to_trajectories(rep, routes, self.manifest, ctx)
        return PlanResult(ts, native={r.id: tour}, layers={"spanning_tree": tree_layer(rep, tree)},
                          info={"coarse_cells": len(nodes), "tour_cells": len(tour)})

"""Adding a planner: a greedy nearest-uncovered-cell planner in black-box mode.

Register it by naming it in an experiment configuration as
``examples.custom_planner:GreedyGrid`` (run from the repository root), or call
``uninbench.planners.register(GreedyGrid)`` before running an experiment.
"""

from __future__ import annotations

import numpy as np

from uninbench.planners import HyperParam, Manifest, PlanContext, Planner, PlanResult
from uninbench.planners.grid_common import cell_routes_to_trajectories
from uninbench.trajectory import SERVICE, TRANSIT


class GreedyGrid(Planner):
    manifest = Manifest(
        name="greedy_grid",
        version="0.1",
        reference="Example planner shipped with uninbench",
        family="grid",
        adapter="grid",  # receives a GridRepresentation (cells, graph, start cells)
        adapter_params={"connectivity": 4},
        constraints={"multi_robot"},
        hyperparameters=[HyperParam("tie_break", "categorical", "row", choices=["row", "col"])],
        core_modules=["../examples/custom_planner.py"],
        description="Every robot repeatedly moves to its nearest uncovered cell (robots take turns).",
    )

    def plan(self, rep, ctx: PlanContext) -> PlanResult:
        cells = set(int(c) for c in np.flatnonzero(rep.used.ravel()))
        pos = {r.id: rep.starts[r.id]["cell"] for r in rep.team}
        routes = {r.id: [] for r in rep.team}
        covered = set(pos.values())
        while covered < cells:
            progressed = False
            for r in rep.team:
                d = rep.distances(pos[r.id])
                left = [c for c in cells - covered if np.isfinite(d[c])]
                if not left:
                    continue
                key = (lambda c: (d[c], c)) if self.params["tie_break"] == "row" else \
                    (lambda c: (d[c], c % rep.nx, c))
                goal = min(left, key=key)
                path = rep.path(pos[r.id], goal)
                routes[r.id] += [(c, TRANSIT if c in covered else SERVICE) for c in path[1:]]
                covered.update(path)
                pos[r.id] = goal
                progressed = True
            if not progressed:
                break
        ts = cell_routes_to_trajectories(rep, routes, self.manifest, ctx)
        return PlanResult(ts, native=routes)

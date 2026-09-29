"""Adding a planner in staged mode: DARP work distribution + a new route-construction stage.

Use it in an experiment as ``examples.staged_planner:DARPColumns`` (run from the repository root).
"""

from uninbench.planners import Manifest, PlanResult, Stage, StagedPlanner
from uninbench.planners.darp import CoarseRegions, DARPDistribution, DARP_PARAMS, _row_runs_route
from uninbench.planners.grid_common import cell_routes_to_trajectories


class ColumnSweeps(Stage):
    """Route construction: back-and-forth along columns inside every region."""
    kind = "route_construction"
    name = "column_sweeps"
    consumes = ("partition",)
    produces = "routes"

    def run(self, data, rep, ctx, params):
        part = data["partition"]
        nx2 = rep.coarse["grid"].nx
        routes = {}
        for k, robot in enumerate(rep.team):
            subs = set()
            for c in part["nodes"][part["labels"] == k]:
                subs.update(int(s) for s in rep.sub_cells(*divmod(int(c), nx2)))
            routes[robot.id] = _row_runs_route(rep, subs, rep.starts[robot.id]["cell"], "cols")
        return {"routes": routes}


class DARPColumns(StagedPlanner):
    manifest = Manifest(
        name="darp_columns", version="0.1", reference="Composition: DARP + column sweeps",
        family="grid", adapter="grid", adapter_params={"coarse": True},
        constraints={"multi_robot"}, stochastic=True, mode="staged",
        stages=["coarse_regions", "darp", "column_sweeps"], composition_of=["darp", "column sweeps"],
        hyperparameters=DARP_PARAMS, core_modules=[])
    stages = [CoarseRegions(), DARPDistribution(), ColumnSweeps()]

    def finish(self, data, rep, ctx):
        ts = cell_routes_to_trajectories(rep, data["routes"]["routes"], self.manifest, ctx)
        return PlanResult(ts)

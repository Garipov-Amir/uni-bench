"""Genetic algorithm on coverage targets -- benchmark baseline (target family, black-box, stochastic).

Problem formulation of Kapanoglu et al. (2012): the area is represented by
disks (the target adapter's coverage targets), and a visiting sequence of the
disks is sought for every robot so that the completion time of the slowest
robot is minimal.  The pattern-based chromosome of that paper is not
reproduced (its operators are not publicly documented), so this planner is a
*baseline*: a two-part chromosome (a permutation of all targets and k-1
breakpoints that split it among the robots), tournament selection, ordered
crossover and one-point breakpoint crossover, inversion / swap / breakpoint
mutations and elitism.  The initial population contains a nearest-neighbour
tour split into balanced pieces.  Like the target baseline, all segments are
coverage (service is not distinguished from transit).
"""

from __future__ import annotations

import numpy as np
import shapely

from ..trajectory import SERVICE, TrajectoryBuilder, TrajectorySet
from .base import HyperParam, Manifest, PlanContext, Planner, PlanResult
from .targets import nn_tour


def _route_times(order, cuts, P, starts, speed, ret):
    """Completion time of every robot for a permutation and sorted breakpoints."""
    k = len(starts)
    out = np.zeros(k)
    bounds = np.r_[0, cuts, len(order)]
    for r in range(k):
        seq = order[bounds[r]:bounds[r + 1]]
        if len(seq) == 0:
            continue
        Q = P[seq]
        L = np.hypot(*(Q[0] - starts[r])) + np.hypot(*np.diff(Q, axis=0).T).sum()
        if ret:
            L += np.hypot(*(Q[-1] - starts[r]))
        out[r] = L / speed[r]
    return out


def _ox(a, b, rng):
    n = len(a)
    i, j = sorted(rng.choice(n, 2, replace=False)) if n > 1 else (0, 0)
    child = -np.ones(n, dtype=int)
    child[i:j + 1] = a[i:j + 1]
    used = set(child[i:j + 1].tolist())
    fill = [g for g in b if g not in used]
    idx = [t for t in range(n) if child[t] < 0]
    child[idx] = fill
    return child


def ga_plan(P, starts, speed, ret, rng, pop_size=60, generations=400, pc=0.9, pm=0.3, elite=2, deadline=None):
    n, k = len(P), len(starts)
    if n == 0:
        return np.zeros(0, dtype=int), np.zeros(max(k - 1, 0), dtype=int), {"generations": 0}

    def fitness(ind):
        t = _route_times(ind[0], ind[1], P, starts, speed, ret)
        return float(t.max()) + 1e-6 * float(t.sum())

    def rand_cuts():
        return np.sort(rng.integers(0, n + 1, size=k - 1)) if k > 1 else np.zeros(0, dtype=int)

    base = nn_tour(np.asarray(starts[0]), P)
    even = np.array([int(round(n * j / k)) for j in range(1, k)], dtype=int)
    pop = [(base.copy(), even.copy())]
    while len(pop) < pop_size:
        pop.append((rng.permutation(n), rand_cuts()))
    fit = np.array([fitness(i) for i in pop])
    gens = 0
    for g in range(generations):
        gens = g + 1
        order = np.argsort(fit)
        new = [pop[i] for i in order[:elite]]
        while len(new) < pop_size:
            def pick():
                a, b = rng.integers(pop_size, size=2)
                return pop[a] if fit[a] < fit[b] else pop[b]
            p1, p2 = pick(), pick()
            if rng.random() < pc:
                perm = _ox(p1[0], p2[0], rng)
                if k > 1:
                    c = int(rng.integers(0, k))
                    cuts = np.sort(np.r_[p1[1][:c], p2[1][c:]])
                else:
                    cuts = p1[1].copy()
            else:
                perm, cuts = p1[0].copy(), p1[1].copy()
            if rng.random() < pm:
                i, j = sorted(rng.choice(n, 2, replace=False)) if n > 1 else (0, 0)
                perm[i:j + 1] = perm[i:j + 1][::-1]
            if rng.random() < pm and n > 1:
                i, j = rng.choice(n, 2, replace=False)
                perm[[i, j]] = perm[[j, i]]
            if rng.random() < pm and k > 1:
                c = int(rng.integers(0, k - 1))
                cuts[c] = int(np.clip(cuts[c] + rng.integers(-max(1, n // 20), max(1, n // 20) + 1), 0, n))
                cuts = np.sort(cuts)
            new.append((perm, cuts))
        pop = new
        fit = np.array([fitness(i) for i in pop])
        if deadline is not None and deadline():
            break
    best = pop[int(np.argmin(fit))]
    return best[0], best[1], {"generations": gens, "best_fitness": float(fit.min())}


class GATargets(Planner):
    manifest = Manifest(
        name="ga_targets",
        version="1.0",
        reference="Benchmark baseline: genetic algorithm on coverage disks in the problem formulation of Kapanoglu, "
                  "Alikalfa, Ozkan, Yazici & Parlaktuna (2012); the paper's pattern-based encoding is not reproduced",
        family="target",
        adapter="target",
        adapter_params={"lattice": "square"},
        constraints={"multi_robot", "return_to_depot"},
        stochastic=True,
        provenance="baseline",
        composition_of=["two-part chromosome GA (min-max completion time)"],
        distinguishes_service_transit=False,
        hyperparameters=[
            HyperParam("population", "int", 60, low=20, high=300, log=True, doc="population size"),
            HyperParam("generations", "int", 400, low=50, high=5000, log=True, doc="maximum generations"),
            HyperParam("crossover", "float", 0.9, low=0.5, high=1.0, doc="crossover probability"),
            HyperParam("mutation", "float", 0.3, low=0.01, high=0.8, doc="mutation probability (per operator)"),
        ],
        core_modules=["uninbench/planners/ga_targets.py"],
        description="Genetic algorithm over the visiting order and split of the coverage targets.",
    )

    def plan(self, rep, ctx: PlanContext) -> PlanResult:
        P = rep.targets
        robots = rep.team
        starts = np.array([r.start[:2] for r in robots], dtype=float)
        speed = np.array([r.speed for r in robots], dtype=float)
        order, cuts, info = ga_plan(P, starts, speed, ctx.return_to_depot, ctx.rng, int(self.params["population"]),
                                    int(self.params["generations"]), float(self.params["crossover"]),
                                    float(self.params["mutation"]), deadline=lambda: ctx.out_of_time(0.8))
        bounds = np.r_[0, cuts, len(order)]
        ts = TrajectorySet(metadata={"planner": self.manifest.name})
        cs_tol = rep.cspace(robots[0]).buffer(1e-6 * max(1.0, robots[0].working_width))
        shapely.prepare(cs_tol)
        tours = {}
        for i, r in enumerate(robots):
            b = TrajectoryBuilder(r.id, *r.start, tool_active_in_turns=self.manifest.tool_active_in_turns)
            seq = P[order[bounds[i]:bounds[i + 1]]]
            tours[r.id] = seq.tolist()
            tp = None
            for q in seq:
                if cs_tol.covers(shapely.LineString([b.pos, tuple(q)])):
                    b.move_to(q[0], q[1], SERVICE)
                else:
                    tp = tp or rep.transit(r)
                    b.follow(tp.path(b.pos, q), SERVICE)
            if ctx.return_to_depot:
                tp = tp or rep.transit(r)
                depot = r.depot if r.depot is not None else r.start[:2]
                b.follow(tp.path(b.pos, depot), SERVICE)
            ts.add(b.build())
        layers = rep.layers()
        lab = np.full(len(P), -1)
        for i in range(len(robots)):
            lab[order[bounds[i]:bounds[i + 1]]] = i
        layers["clusters"] = {"type": "point_labels", "coords": np.round(P, 4).tolist(), "labels": lab.tolist()}
        return PlanResult(ts, native=tours, layers=layers, info=info)


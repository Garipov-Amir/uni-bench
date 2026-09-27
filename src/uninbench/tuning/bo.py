"""Minimal Bayesian optimisation: Gaussian process (Matern 5/2) + expected improvement.

Kept dependency-free (numpy/scipy) so that every planner is tuned with exactly
the same optimiser and budget (R8).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from scipy.linalg import cho_factor, cho_solve
from scipy.stats import norm


class Space:
    """Encodes planner hyperparameters into the unit hypercube."""

    def __init__(self, hps):
        self.hps = list(hps)
        self.slices = []
        d = 0
        for h in self.hps:
            k = len(h.choices) if h.kind == "categorical" else 1
            self.slices.append(slice(d, d + k))
            d += k
        self.dim = d

    def encode(self, theta: dict) -> np.ndarray:
        u = np.zeros(self.dim)
        for h, sl in zip(self.hps, self.slices):
            v = theta.get(h.name, h.default)
            if h.kind == "categorical":
                u[sl.start + h.choices.index(v)] = 1.0
            elif h.kind == "bool":
                u[sl] = 1.0 if v else 0.0
            else:
                lo, hi = float(h.low), float(h.high)
                if h.log:
                    u[sl] = (math.log(v) - math.log(lo)) / (math.log(hi) - math.log(lo))
                else:
                    u[sl] = (float(v) - lo) / (hi - lo) if hi > lo else 0.0
        return np.clip(u, 0, 1)

    def decode(self, u: np.ndarray) -> dict:
        out = {}
        for h, sl in zip(self.hps, self.slices):
            x = u[sl]
            if h.kind == "categorical":
                out[h.name] = h.choices[int(np.argmax(x))]
            elif h.kind == "bool":
                out[h.name] = bool(x[0] >= 0.5)
            else:
                lo, hi = float(h.low), float(h.high)
                t = float(np.clip(x[0], 0, 1))
                v = math.exp(math.log(lo) + t * (math.log(hi) - math.log(lo))) if h.log else lo + t * (hi - lo)
                out[h.name] = int(round(v)) if h.kind == "int" else float(v)
        return out

    def sample(self, rng: np.random.Generator) -> np.ndarray:
        u = rng.random(self.dim)
        for h, sl in zip(self.hps, self.slices):
            if h.kind == "categorical":
                z = np.zeros(sl.stop - sl.start)
                z[rng.integers(len(z))] = 1.0
                u[sl] = z
            elif h.kind == "bool":
                u[sl] = float(rng.random() < 0.5)
        return u

    def canonical(self, u: np.ndarray) -> np.ndarray:
        """Project onto feasible encodings (one-hot, rounded integers)."""
        return self.encode(self.decode(u))


def _matern52(A, B, ls):
    d = np.sqrt(np.maximum(((A[:, None, :] - B[None, :, :]) ** 2).sum(-1), 0)) / ls
    s5 = math.sqrt(5)
    return (1 + s5 * d + 5.0 / 3.0 * d * d) * np.exp(-s5 * d)


@dataclass
class GP:
    ls: float = 0.3
    noise: float = 1e-4

    def fit(self, X, y):
        self.X = np.asarray(X, float)
        y = np.asarray(y, float)
        self.mu_y, self.sd_y = float(y.mean()), float(y.std() or 1.0)
        self.y = (y - self.mu_y) / self.sd_y
        best = None
        for ls in (0.05, 0.1, 0.2, 0.3, 0.5, 0.8, 1.2, 2.0):
            K = _matern52(self.X, self.X, ls) + self.noise * np.eye(len(self.X))
            try:
                c = cho_factor(K, lower=True)
            except np.linalg.LinAlgError:
                continue
            alpha = cho_solve(c, self.y)
            ll = -0.5 * self.y @ alpha - np.log(np.diag(c[0])).sum()
            if best is None or ll > best[0]:
                best = (ll, ls, c, alpha)
        _, self.ls, self.c, self.alpha = best
        return self

    def predict(self, Xs):
        Ks = _matern52(np.asarray(Xs, float), self.X, self.ls)
        mu = Ks @ self.alpha
        v = cho_solve(self.c, Ks.T)
        var = np.maximum(1.0 - (Ks * v.T).sum(1), 1e-12)
        return mu * self.sd_y + self.mu_y, np.sqrt(var) * self.sd_y


def expected_improvement(mu, sigma, best, xi: float = 0.01):
    imp = best - mu - xi
    z = imp / sigma
    return imp * norm.cdf(z) + sigma * norm.pdf(z)


def bayes_opt(objective, space: Space, budget: int, rng: np.random.Generator, theta0: dict | None = None,
              n_init: int | None = None, n_candidates: int = 2000, log=None):
    """Minimise ``objective(theta)`` with ``budget`` evaluations; theta0 is evaluated first."""
    X, Y, thetas = [], [], []

    def run(u):
        th = space.decode(u)
        y = float(objective(th))
        X.append(space.encode(th))
        Y.append(y)
        thetas.append(th)
        if log:
            log(len(Y), th, y)

    if space.dim == 0 or budget <= 0:
        if theta0 is not None and budget > 0:
            run(space.encode(theta0))
        return thetas, Y
    n_init = n_init or max(2, min(budget // 3, 2 * space.dim + 1))
    if theta0 is not None:
        run(space.encode(theta0))
    while len(Y) < min(n_init, budget):
        run(space.canonical(space.sample(rng)))
    while len(Y) < budget:
        gp = GP().fit(np.array(X), np.array(Y))
        cand = np.array([space.canonical(space.sample(rng)) for _ in range(n_candidates // 2)])
        best_u = X[int(np.argmin(Y))]
        local = np.clip(best_u + 0.1 * rng.standard_normal((n_candidates // 2, space.dim)), 0, 1)
        local = np.array([space.canonical(u) for u in local])
        cand = np.vstack([cand, local])
        mu, sd = gp.predict(cand)
        ei = expected_improvement(mu, sd, min(Y))
        seen = {tuple(np.round(x, 6)) for x in X}
        order = np.argsort(-ei)
        pick = next((cand[k] for k in order if tuple(np.round(cand[k], 6)) not in seen), cand[order[0]])
        run(pick)
    return thetas, Y

"""Statistical tools of the reporting protocol (R11)."""

from __future__ import annotations

import math
from itertools import combinations

import numpy as np
from scipy import stats

# Demsar (2006), Table 5(a): q_0.05 for the Nemenyi test (fallback)
_Q05 = {2: 1.960, 3: 2.343, 4: 2.569, 5: 2.728, 6: 2.850, 7: 2.949, 8: 3.031, 9: 3.102, 10: 3.164}


def bootstrap_ci(values, stat=np.median, n_boot: int = 2000, alpha: float = 0.05, seed: int = 0):
    """Percentile bootstrap CI (resampling the given units, e.g. instances)."""
    v = np.asarray([x for x in values if x is not None and np.isfinite(x)], dtype=float)
    if len(v) == 0:
        return (math.nan, math.nan)
    if len(v) == 1:
        return (float(v[0]), float(v[0]))
    rng = np.random.default_rng(seed)
    idx = rng.integers(0, len(v), size=(n_boot, len(v)))
    boots = np.apply_along_axis(stat, 1, v[idx])
    return float(np.quantile(boots, alpha / 2)), float(np.quantile(boots, 1 - alpha / 2))


def describe(values, seed: int = 0) -> dict:
    v = np.asarray([x for x in values if x is not None and np.isfinite(x)], dtype=float)
    if len(v) == 0:
        return {"n": 0, "median": None, "q1": None, "q3": None, "iqr": None, "mean": None, "ci_lo": None,
                "ci_hi": None}
    q1, med, q3 = np.quantile(v, [0.25, 0.5, 0.75])
    lo, hi = bootstrap_ci(v, seed=seed)
    return {"n": int(len(v)), "median": float(med), "q1": float(q1), "q3": float(q3), "iqr": float(q3 - q1),
            "mean": float(v.mean()), "ci_lo": lo, "ci_hi": hi}


def holm(pvals) -> list[float]:
    """Holm (1979) step-down adjustment."""
    p = np.asarray(pvals, dtype=float)
    n = len(p)
    order = np.argsort(p)
    adj = np.empty(n)
    run = 0.0
    for k, i in enumerate(order):
        run = max(run, (n - k) * p[i])
        adj[i] = min(1.0, run)
    return adj.tolist()


def paired_differences(a, b) -> np.ndarray:
    """Per-instance differences a - b with failures (+inf) kept.

    A failure of one planner is a difference of -inf/+inf in favour of the
    other; a failure of both is a tie.  Instances where a value is missing
    (NaN, e.g. not applicable) are dropped.
    """
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    ok = ~(np.isnan(a) | np.isnan(b))
    a, b = a[ok], b[ok]
    both = np.isinf(a) & np.isinf(b)
    with np.errstate(invalid="ignore"):
        d = np.where(both, 0.0, a - b)
    return d


def wilcoxon_paired(a, b) -> dict:
    """Wilcoxon signed-rank test on paired values; failures (+inf) count as the
    largest possible differences, so a planner cannot look better by failing."""
    d = paired_differences(a, b)
    out = {"n": int(len(d)), "failures_a": int(np.sum(np.isposinf(d))), "failures_b": int(np.sum(np.isneginf(d))),
           "median_diff": float(np.median(d)) if len(d) else None}
    nz = d[d != 0]
    if len(nz) == 0:
        out.update(statistic=None, p=1.0, rank_biserial=0.0)
        return out
    fin = np.abs(nz[np.isfinite(nz)])
    big = (fin.max() if len(fin) else 1.0) * 2 + 1.0
    z = np.where(np.isinf(nz), np.sign(nz) * big, nz)  # ranks only depend on the order of |d|
    r = stats.rankdata(np.abs(z))
    wp, wm = float(r[z > 0].sum()), float(r[z < 0].sum())
    out["rank_biserial"] = (wp - wm) / (wp + wm)
    if len(z) < 2:
        out.update(statistic=None, p=1.0)
        return out
    res = stats.wilcoxon(z, zero_method="wilcox")
    out.update(statistic=float(res.statistic), p=float(res.pvalue))
    return out


def friedman(matrix: np.ndarray) -> dict:
    """Friedman test on per-instance ranks; rows = instances, columns = planners.

    Failures are encoded as +inf and therefore ranked last (ties averaged).
    """
    M = np.asarray(matrix, dtype=float)
    N, k = M.shape
    R = np.vstack([stats.rankdata(np.where(np.isnan(row), np.inf, row)) for row in M]) if N else np.zeros((0, k))
    avg = R.mean(axis=0) if N else np.full(k, math.nan)
    out = {"N": int(N), "k": int(k), "avg_ranks": avg.tolist(), "statistic": None, "p": None}
    if N >= 2 and k >= 3:
        chi2 = 12 * N / (k * (k + 1)) * (np.sum(avg ** 2) - k * (k + 1) ** 2 / 4)
        # Iman-Davenport correction
        denom = N * (k - 1) - chi2
        ff = (N - 1) * chi2 / denom if denom > 0 else math.inf
        out.update(statistic=float(chi2), p=float(stats.chi2.sf(chi2, k - 1)),
                   iman_davenport=float(ff), p_iman_davenport=float(stats.f.sf(ff, k - 1, (k - 1) * (N - 1)))
                   if math.isfinite(ff) else 0.0)
    return out


def nemenyi_cd(k: int, N: int, alpha: float = 0.05) -> float:
    """Critical difference of average ranks (Demsar, 2006)."""
    if k < 2 or N < 1:
        return math.nan
    try:
        q = float(stats.studentized_range.ppf(1 - alpha, k, np.inf)) / math.sqrt(2)
        if not math.isfinite(q):
            raise ValueError
    except Exception:
        q = _Q05.get(k, 3.164 + 0.05 * (k - 10))
    return q * math.sqrt(k * (k + 1) / (6.0 * N))


def pairwise_wilcoxon(matrix: np.ndarray, names: list[str]) -> list[dict]:
    rows = []
    for i, j in combinations(range(len(names)), 2):
        r = wilcoxon_paired(matrix[:, i], matrix[:, j])
        r.update(a=names[i], b=names[j])
        rows.append(r)
    adj = holm([r["p"] if r["p"] is not None else 1.0 for r in rows])
    for r, p in zip(rows, adj):
        r["p_holm"] = p
    return rows


def performance_profile(matrix: np.ndarray, taus: np.ndarray) -> np.ndarray:
    """Dolan & More (2002): fraction of instances with ratio to the best <= tau.

    ``matrix`` rows = instances, columns = planners; failures are +inf.
    Returns an array (planners, len(taus)).
    """
    M = np.asarray(matrix, dtype=float)
    best = np.min(M, axis=1, keepdims=True)
    with np.errstate(invalid="ignore", divide="ignore"):
        ratio = np.where(np.isfinite(M) & np.isfinite(best) & (best > 0), M / best,
                         np.where(np.isfinite(M) & (best == 0) & (M == 0), 1.0, np.inf))
    return np.array([[float(np.mean(ratio[:, p] <= t)) for t in taus] for p in range(M.shape[1])])


def scaling_exponent(x, y) -> dict:
    """Least-squares slope of log(y) against log(x)."""
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    ok = (x > 0) & (y > 0) & np.isfinite(x) & np.isfinite(y)
    if ok.sum() < 2 or len(np.unique(x[ok])) < 2:
        return {"slope": None, "intercept": None, "r2": None, "n": int(ok.sum())}
    lx, ly = np.log(x[ok]), np.log(y[ok])
    slope, intercept, r, _, _ = stats.linregress(lx, ly)
    return {"slope": float(slope), "intercept": float(intercept), "r2": float(r * r), "n": int(ok.sum())}


def pareto_front(points) -> list[int]:
    """Indices of non-dominated points (minimise both coordinates)."""
    pts = [(i, p) for i, p in enumerate(points) if p[0] is not None and p[1] is not None]
    front = []
    for i, (x, y) in pts:
        dominated = any((x2 <= x and y2 <= y) and (x2 < x or y2 < y) for j, (x2, y2) in pts if j != i)
        if not dominated:
            front.append(i)
    return front

import numpy as np
import pytest
from scipy import stats

from uninbench.reporting.stats import (describe, friedman, holm, nemenyi_cd, pareto_front, performance_profile,
                                       scaling_exponent, wilcoxon_paired)
from uninbench.tuning.bo import Space, bayes_opt
from uninbench.planners.base import HyperParam


def test_holm():
    assert holm([0.01, 0.04, 0.03, 0.005]) == pytest.approx([0.03, 0.06, 0.06, 0.02])


def test_friedman_matches_scipy():
    rng = np.random.default_rng(0)
    M = rng.random((12, 4)) + np.array([0, 0.2, 0.4, 0.1])
    ours = friedman(M)
    ref = stats.friedmanchisquare(*M.T)
    assert ours["statistic"] == pytest.approx(ref.statistic)
    assert ours["p"] == pytest.approx(ref.pvalue)


def test_failures_rank_last():
    M = np.array([[1.0, np.inf], [2.0, 1.0], [1.0, np.inf]])
    assert friedman(M)["avg_ranks"] == pytest.approx([1 + 1 / 3, 2 - 1 / 3])


def test_nemenyi_cd_matches_demsar():
    # Demsar (2006): q_0.05 = 2.569 for k = 4
    assert nemenyi_cd(4, 10) == pytest.approx(2.569 * np.sqrt(4 * 5 / 60), rel=1e-3)


def test_wilcoxon_and_effect_size():
    a = np.arange(1, 11, dtype=float)
    r = wilcoxon_paired(a, a + 1.0)
    assert r["median_diff"] == -1.0 and r["rank_biserial"] == -1.0 and r["p"] < 0.01


def test_performance_profile_and_pareto():
    M = np.array([[1.0, 2.0], [2.0, 1.0], [1.0, np.inf]])
    rho = performance_profile(M, np.array([1.0, 2.0, 100.0]))
    assert rho[0].tolist() == pytest.approx([2 / 3, 1.0, 1.0])
    assert rho[1].tolist() == pytest.approx([1 / 3, 2 / 3, 2 / 3])
    assert set(pareto_front([(1, 3), (2, 2), (3, 3), (0.5, 5)])) == {0, 1, 3}


def test_scaling_exponent_and_describe():
    x = np.array([1, 2, 4, 8, 16])
    assert scaling_exponent(x, 3 * x ** 2.0)["slope"] == pytest.approx(2.0)
    d = describe([1, 2, 3, 4, 100])
    assert d["median"] == 3 and d["ci_lo"] <= 3 <= d["ci_hi"]


def test_bayes_opt_finds_minimum():
    space = Space([HyperParam("x", "float", 0.5, low=0.0, high=1.0), HyperParam("c", "categorical", "a",
                                                                               choices=["a", "b"])])
    f = lambda th: (th["x"] - 0.8) ** 2 + (0 if th["c"] == "b" else 0.1)  # noqa: E731
    thetas, ys = bayes_opt(f, space, 20, np.random.default_rng(0), theta0={"x": 0.5, "c": "a"})
    best = thetas[int(np.argmin(ys))]
    assert best["c"] == "b" and abs(best["x"] - 0.8) < 0.1

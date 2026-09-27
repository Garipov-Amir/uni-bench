import json

from uninbench.reporting import build_report
from uninbench.runner import load_runs, run_experiment
from uninbench.simulator import build_replay, plot_run
from uninbench.tuning import tune_experiment


def _cfg(tmp_path, **extra):
    cfg = {
        "name": "t", "output": str(tmp_path / "res"), "tuning_dir": str(tmp_path / "tuning"),
        "instances": [{"generate": {"kind": "synthetic", "count": 2, "sizes": [16], "densities": [0.05]},
                       "family": "IF5", "split": "all"}],
        "team_sizes": [1, 2], "start_configs": ["dispersed"],
        "planners": ["stc", "mstc", "swath_bf", {"name": "darp_stc", "configs": ["default"]}],
        "seeds": 2, "repetitions": 2, "inline": True, "task": {"time_limit": 30},
        "evaluation": {"resolution_factor": 0.2},
    }
    cfg.update(extra)
    return cfg


def test_experiment_report_replay(tmp_path):
    cfg = _cfg(tmp_path)
    s = run_experiment(cfg, progress=False)
    recs = load_runs(tmp_path / "res")
    outcomes = {r["outcome"] for r in recs}
    assert "not_applicable" in outcomes  # STC with two robots
    assert all(r["outcome"] in ("success", "incomplete", "invalid", "timeout", "crash", "not_applicable")
               for r in recs)
    ran = [r for r in recs if r["outcome"] != "not_applicable"]
    assert all("instance_hash" in r and "adapter" in r and "hardware" in r and "software" in r for r in ran)
    darp = [r for r in ran if r["planner"] == "darp_stc"]
    assert len({r["seed"] for r in darp}) == 2  # stochastic: one run per seed
    mstc = [r for r in ran if r["planner"] == "mstc"]
    assert all(len(r["planning_times"]) == 2 for r in mstc)  # deterministic: repeated timing
    # resume skips everything already recorded
    s2 = run_experiment(cfg, progress=False)
    assert s2["runs"] == 0 and s2["skipped_existing"] == s["runs"]
    page = build_report(tmp_path / "res")
    assert page.exists() and (page.parent / "runs.csv").exists()
    html = page.read_text()
    assert "Friedman" in html and "Performance profiles" in html
    replay = build_replay(tmp_path / "res")
    assert "__DATA__" not in replay.read_text()
    fig = plot_run(tmp_path / "res", ran[0]["run_id"])
    assert fig.exists()


def test_tuning_freezes_configuration(tmp_path):
    cfg = _cfg(tmp_path, planners=["swath_bf"], team_sizes=[1])
    cfg["instances"][0]["split"] = "all"
    res = tune_experiment(cfg, budget=2, tuning_seeds=1, inline=True, verbose=False)
    rec = res[("swath_bf", "IF5")]
    assert rec["frozen"] and len(rec["history"]) == 2
    stored = json.loads((tmp_path / "tuning" / "swath_bf__IF5.json").read_text())
    assert stored["theta_star"] == rec["theta_star"]
    cfg2 = _cfg(tmp_path, planners=[{"name": "swath_bf", "configs": ["default", "tuned"]}], team_sizes=[1])
    run_experiment(cfg2, progress=False)
    labels = {r["config_label"] for r in load_runs(tmp_path / "res")}
    assert {"default", "tuned"} <= labels

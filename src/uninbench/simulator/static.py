"""Static figure of a stored run (coverage count map + trajectories), e.g. for papers."""

from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.colors import ListedColormap  # noqa: E402

from ..geometry.decomposition import polygons_of  # noqa: E402
from ..geometry.raster import RasterGrid  # noqa: E402
from ..instance.model import Instance  # noqa: E402
from ..runner.experiment import load_runs  # noqa: E402
from ..util import load_json, unpack_array  # noqa: E402

SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#4a3aa7", "#e34948"]
COUNT_CMAP = ListedColormap(["#fcfcfb", "#b7d3f6", "#5598e7", "#1c5cab"])


def plot_run(results_dir, run_id: str, out=None, show_layers=("swaths", "spanning_tree")) -> Path:
    results_dir = Path(results_dir)
    rec = next(r for r in load_runs(results_dir) if r["run_id"] == run_id)
    arts = rec["artifacts"]
    inst = Instance.load(results_dir / arts["instance"])
    traj = load_json(results_dir / arts["trajectories"])
    fig, ax = plt.subplots(figsize=(7, 7 * (inst.environment.bounds[3] - inst.environment.bounds[1]) /
                                    max(inst.environment.bounds[2] - inst.environment.bounds[0], 1e-9)), dpi=130)
    fig.patch.set_facecolor("#fcfcfb")
    ax.set_facecolor("#c3c2b7")
    if arts.get("coverage"):
        cov = load_json(results_dir / arts["coverage"])
        g = RasterGrid.from_dict(cov["grid"])
        n = unpack_array(cov["counts"]).astype(float)
        ax.imshow(np.minimum(n, 3), origin="lower", extent=(g.x0, g.x0 + g.nx * g.res, g.y0, g.y0 + g.ny * g.res),
                  cmap=COUNT_CMAP, vmin=0, vmax=3, interpolation="nearest", zorder=1)
    for p in polygons_of(inst.free):
        for ring in [p.exterior, *p.interiors]:
            c = np.asarray(ring.coords)
            ax.plot(c[:, 0], c[:, 1], color="#52514e", lw=0.8, zorder=2)
    for name in show_layers:
        lay = (traj.get("layers") or {}).get(name) or (traj.get("adapter_layers") or {}).get(name)
        if lay and lay.get("type") in ("lines", "segments"):
            for ln in lay["coords"]:
                ln = np.asarray(ln)
                ax.plot(ln[:, 0], ln[:, 1], color="#898781", lw=0.5, zorder=3)
    for k, t in enumerate((traj.get("retimed") or traj["trajectories"])["trajectories"]):
        xy = np.asarray(t["xy"])
        c = SERIES[k] if k < len(SERIES) else "#898781"
        ax.plot(xy[:, 0], xy[:, 1], color=c, lw=1.0, zorder=4, label=t["robot_id"])
        ax.plot(xy[0, 0], xy[0, 1], "o", color=c, ms=5, zorder=5)
    ax.set_aspect("equal")
    m = rec.get("metrics") or {}
    ax.set_title(f"{rec['planner']} · {rec['instance_id']}\n{rec['outcome']} · ρ={m.get('coverage_ratio', 0):.3f} · "
                 f"ω={m.get('overlap_ratio', 0):.3f} · T/T_LB={m.get('normalized_makespan', 0):.3f}", fontsize=9,
                 loc="left")
    ax.legend(fontsize=7, frameon=False, loc="upper right")
    out = Path(out) if out else results_dir / "figures" / f"{run_id}.png"
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, bbox_inches="tight")
    plt.close(fig)
    return out

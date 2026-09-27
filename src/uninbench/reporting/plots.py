"""Figures of the reporting protocol (matplotlib, static PNG).

Colours follow the entity (planner), never its rank: every figure of a report
uses the same planner -> colour mapping in a fixed categorical order.  Beyond
eight planners the remaining ones are drawn in muted grey and identified by
direct labels and legends.
"""

from __future__ import annotations

import io
import math

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#4a3aa7", "#e34948"]
OTHER = "#898781"
INK, INK2, MUTED, GRID, AXIS, SURFACE = "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7", "#fcfcfb"


def palette(names) -> dict:
    return {n: (SERIES[k] if k < len(SERIES) else OTHER) for k, n in enumerate(sorted(set(names)))}


def _style(ax, title=None, xlabel=None, ylabel=None):
    ax.set_facecolor(SURFACE)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_color(AXIS)
    ax.tick_params(colors=INK2, labelsize=8, length=3, color=AXIS)
    ax.grid(True, color=GRID, linewidth=0.6)
    ax.set_axisbelow(True)
    if title:
        ax.set_title(title, color=INK, fontsize=10, loc="left")
    if xlabel:
        ax.set_xlabel(xlabel, color=INK2, fontsize=8)
    if ylabel:
        ax.set_ylabel(ylabel, color=INK2, fontsize=8)


def _fig(w=6.4, h=3.6):
    fig, ax = plt.subplots(figsize=(w, h), dpi=130)
    fig.patch.set_facecolor(SURFACE)
    return fig, ax


def to_png(fig) -> bytes:
    buf = io.BytesIO()
    fig.savefig(buf, format="png", bbox_inches="tight", facecolor=fig.get_facecolor())
    plt.close(fig)
    return buf.getvalue()


# ------------------------------------------------------------------ CD diagram
def cd_diagram(names, avg_ranks, cd, title="", colors=None) -> bytes:
    """Critical-difference diagram (Demsar, 2006)."""
    colors = colors or palette(names)
    k = len(names)
    order = np.argsort(avg_ranks)
    ranks = np.asarray(avg_ranks)[order]
    labels = [names[i] for i in order]
    fig, ax = _fig(6.4, 1.2 + 0.28 * k)
    lo, hi = 1, max(k, 2)
    ax.set_xlim(lo - 0.3, hi + 0.3)
    ax.set_ylim(0, 1)
    ax.axis("off")
    y0 = 0.9
    ax.plot([lo, hi], [y0, y0], color=AXIS, lw=1)
    for r in range(lo, hi + 1):
        ax.plot([r, r], [y0, y0 + 0.03], color=AXIS, lw=1)
        ax.text(r, y0 + 0.05, str(r), ha="center", va="bottom", fontsize=8, color=INK2)
    step = 0.75 / max(k, 1)
    half = int(math.ceil(k / 2))
    for i, (lab, r) in enumerate(zip(labels, ranks)):
        left = i < half
        yy = y0 - 0.12 - (i if left else k - 1 - i) * step
        xe = lo - 0.2 if left else hi + 0.2
        ax.plot([r, r, xe], [y0, yy, yy], color=colors.get(lab, OTHER), lw=1.4)
        ax.plot([r], [y0], "o", ms=4, color=colors.get(lab, OTHER))
        ax.text(xe + (-0.05 if left else 0.05), yy, f"{lab} ({r:.2f})", ha="right" if left else "left",
                va="center", fontsize=8, color=INK)
    # cliques: maximal runs of planners whose ranks differ by less than CD
    if np.isfinite(cd):
        cliques = []
        for i in range(k):
            j = i
            while j + 1 < k and ranks[j + 1] - ranks[i] < cd:
                j += 1
            if j > i and not any(a <= i and j <= b for a, b in cliques):
                cliques.append((i, j))
        for n, (a, b) in enumerate(cliques):
            yy = y0 - 0.05 - 0.035 * n
            ax.plot([ranks[a] - 0.03, ranks[b] + 0.03], [yy, yy], color=INK, lw=3, solid_capstyle="round")
        ax.plot([lo, lo + cd], [0.04, 0.04], color=INK, lw=1.2)
        ax.text(lo + cd / 2, 0.07, f"CD = {cd:.2f}", ha="center", fontsize=8, color=INK2)
    if title:
        ax.set_title(title, color=INK, fontsize=10, loc="left")
    return to_png(fig)


# ------------------------------------------------------------------ profiles
def performance_profile_plot(names, taus, rho, title="", xlabel="ratio to best (tau)", colors=None) -> bytes:
    colors = colors or palette(names)
    fig, ax = _fig()
    for n, r in zip(names, rho):
        ax.step(taus, r, where="post", color=colors.get(n, OTHER), lw=2, label=n)
    ax.set_ylim(-0.02, 1.02)
    ax.set_xscale("log", base=2)
    _style(ax, title, xlabel, "fraction of instances")
    ax.legend(fontsize=7, frameon=False, loc="lower right")
    return to_png(fig)


def scaling_plot(rows, title="", xlabel="", ylabel="", logy=False, colors=None) -> bytes:
    names = [r["planner"] for r in rows]
    colors = colors or palette(names)
    fig, ax = _fig()
    for r in rows:
        pts = [p for p in r["points"] if p.get("median") is not None]
        if not pts:
            continue
        x = np.array([p["x"] for p in pts])
        y = np.array([p["median"] for p in pts])
        lo = np.array([p["ci_lo"] for p in pts])
        hi = np.array([p["ci_hi"] for p in pts])
        c = colors.get(r["planner"], OTHER)
        ax.fill_between(x, lo, hi, color=c, alpha=0.15, lw=0)
        ax.plot(x, y, "-o", color=c, lw=2, ms=4, label=r["planner"])
        slope = (r.get("loglog") or {}).get("slope")
        if slope is not None and logy:
            ax.annotate(f"{slope:+.2f}", (x[-1], y[-1]), xytext=(4, 0), textcoords="offset points", fontsize=7,
                        color=INK2, va="center")
    if logy:
        ax.set_yscale("log")
        if all(p["x"] > 0 for r in rows for p in r["points"]):
            ax.set_xscale("log")
    xs = sorted({p["x"] for r in rows for p in r["points"]})
    if xs and len(xs) <= 12:
        ax.set_xticks(xs, [f"{x:g}" for x in xs])
        ax.minorticks_off()
    _style(ax, title, xlabel, ylabel)
    ax.legend(fontsize=7, frameon=False)
    return to_png(fig)


def pareto_plot(rows, quality, cost, title="", colors=None) -> bytes:
    names = [r["planner"] for r in rows]
    colors = colors or palette(names)
    fig, ax = _fig()
    front = sorted([r for r in rows if r["pareto"]], key=lambda r: r[cost])
    if len(front) > 1:
        ax.plot([r[cost] for r in front], [r[quality] for r in front], color=MUTED, lw=1, ls="--", zorder=1)
    for r in rows:
        c = colors.get(r["planner"], OTHER)
        ax.scatter([r[cost]], [r[quality]], s=48, color=c, edgecolor=SURFACE, linewidth=1.5, zorder=3,
                   label=r["planner"])
        ax.annotate(r["planner"], (r[cost], r[quality]), xytext=(5, 3), textcoords="offset points", fontsize=7,
                    color=INK2)
    ax.set_xscale("log")
    _style(ax, title, f"median {cost} [s] (log)", f"median {quality}")
    ax.legend(fontsize=7, frameon=False)
    return to_png(fig)


def outcome_bars(rows, title="", colors=None) -> bytes:
    """Horizontal stacked bars of run outcomes per planner (status-free: success vs failure kinds)."""
    kinds = ["success", "incomplete", "invalid", "timeout", "crash"]
    kc = {"success": SERIES[0], "incomplete": "#86b6ef", "invalid": SERIES[1], "timeout": SERIES[3],
          "crash": SERIES[7]}
    labels = [f"{r['planner']}  m={r['team_size']}" for r in rows]
    fig, ax = _fig(6.4, 0.6 + 0.26 * max(len(rows), 1))
    left = np.zeros(len(rows))
    for kname in kinds:
        vals = np.array([r.get(kname, 0) / r["runs"] if r["runs"] else 0 for r in rows])
        ax.barh(labels, vals, left=left, color=kc[kname], edgecolor=SURFACE, linewidth=1.5, height=0.7,
                label=kname)
        left += vals
    ax.invert_yaxis()
    ax.set_xlim(0, 1)
    _style(ax, None, "share of runs", None)
    ax.grid(False)
    ax.legend(fontsize=7, frameon=False, ncol=5, loc="lower left", bbox_to_anchor=(0.0, 1.0))
    if title:
        ax.set_title(title, color=INK, fontsize=10, loc="left", pad=18)
    return to_png(fig)


def box_by_planner(data: dict, title="", ylabel="", colors=None) -> bytes:
    names = sorted(data)
    colors = colors or palette(names)
    fig, ax = _fig(6.4, 3.2)
    vals = [data[n] for n in names]
    bp = ax.boxplot(vals, patch_artist=True, widths=0.5, showfliers=True,
                    medianprops={"color": INK, "lw": 1.5}, whiskerprops={"color": AXIS},
                    capprops={"color": AXIS}, flierprops={"markersize": 3, "markeredgecolor": MUTED})
    for patch, n in zip(bp["boxes"], names):
        patch.set_facecolor(colors.get(n, OTHER))
        patch.set_alpha(0.8)
        patch.set_edgecolor(SURFACE)
    ax.set_xticks(range(1, len(names) + 1), names, rotation=20, ha="right")
    _style(ax, title, None, ylabel)
    return to_png(fig)

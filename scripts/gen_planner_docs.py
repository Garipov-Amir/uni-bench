"""Regenerate the planner tables of docs/planners.md from the registered manifests.

    .venv/bin/python scripts/gen_planner_docs.py

Replaces everything between "## Built-in planners" and "### Notes on behaviour".
"""

from __future__ import annotations

import sys
from pathlib import Path

from uninbench.planners import PLANNERS

DOC = Path(__file__).resolve().parents[1] / "docs" / "planners.md"
PROV = {"original": "original", "composition": "composition *", "baseline": "baseline *"}


def fmt_range(h: dict) -> str:
    if h["kind"] == "bool":
        return "true / false"
    if h.get("choices"):
        return ", ".join(map(str, h["choices"]))
    return f"{h.get('low'):g} – {h.get('high'):g}" + (" (log scale)" if h.get("log") else "")


def main() -> int:
    order = sorted(PLANNERS, key=lambda n: (PLANNERS[n].manifest.provenance != "original", PLANNERS[n].manifest.family, n))
    head = ["## Built-in planners", "",
            "| Planner | Family | Provenance | Mode | Kind | Idea |",
            "|---------|--------|------------|------|------|------|"]
    for n in order:
        m = PLANNERS[n].manifest
        head.append(f"| `{n}` | {m.family} | {PROV[m.provenance]} | {'staged' if m.mode == 'staged' else 'black-box'} | "
                    f"{'stochastic' if m.stochastic else 'deterministic'} | {m.description} |")
    head += ["",
             "`*` marks results that are not those of an original published algorithm: *compositions* of stages "
             "combined by the benchmark and *baselines* written for it. They are reported with a star and never as "
             "results of an original algorithm. A planner composed with a framework stage (e.g. prioritized-waiting "
             "deconfliction) is marked `+pw*` in reports.",
             "",
             "List them, including anything you registered yourself, with `uninbench planners` (add `--json` for "
             "full manifests) or in the *Planners* tab of either UI.",
             "",
             "## Planner reference",
             "",
             "Generated from the planners' manifests (`scripts/gen_planner_docs.py`). The *adapter parameters* are "
             "what the planner requires; experiment-wide parameters from `adapters:` are applied on top (see "
             "[Experiments](experiments.md#top-level-keys)).",
             ""]
    for n in sorted(PLANNERS):
        m = PLANNERS[n].manifest
        d = m.to_dict()
        head += [f"### `{n}`", "", m.description, "",
                 f"* **Reference:** {m.reference}",
                 f"* **Provenance:** {m.provenance}" + (f" — {' + '.join(m.composition_of)}" if m.composition_of else ""),
                 f"* **Family / adapter:** {m.family} / `{m.adapter}` with planner-required parameters "
                 f"`{m.adapter_params}`",
                 (f"* **Mode:** staged — stages `{' → '.join(m.stages)}`" if m.mode == "staged" else "* **Mode:** black-box"),
                 "* **Kind:** " + ("stochastic (one run per seed)" if m.stochastic else
                                   "deterministic (planning time is repeated and the median reported)"),
                 f"* **Supports:** {', '.join(sorted(m.constraints)) or '–'}"]
        if not m.distinguishes_service_transit:
            head.append("* **Output:** does not distinguish service from transit (all motion counted as coverage)")
        if m.external_dependencies:
            head.append(f"* **External dependencies:** {', '.join(m.external_dependencies)}")
        head.append("")
        if d["hyperparameters"]:
            head += ["| Hyperparameter | Type | Default | Range / choices | Meaning |", "|---|---|---|---|---|"]
            for h in d["hyperparameters"]:
                head.append(f"| `{h['name']}` | {h['kind']} | `{h['default']}` | {fmt_range(h)} | {h.get('doc', '')} |")
        else:
            head.append("No hyperparameters.")
        head.append("")
    text = DOC.read_text(encoding="utf8")
    a = text.index("## Built-in planners")
    b = text.index("### Notes on behaviour")
    DOC.write_text(text[:a] + "\n".join(head) + "\n" + text[b:], encoding="utf8")
    print(f"updated {DOC} ({len(PLANNERS)} planners)")
    return 0


if __name__ == "__main__":
    sys.exit(main())

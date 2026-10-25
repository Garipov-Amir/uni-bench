"""Coarse implementation-complexity indicators of a planner."""

from __future__ import annotations

import io
import tokenize
from pathlib import Path

import uninbench

from ..planners import get_planner


def sloc(path: Path) -> int:
    """Source lines excluding blank lines, comments and docstrings."""
    try:
        src = path.read_text(encoding="utf8")
    except OSError:
        return 0
    lines = set()
    prev = tokenize.INDENT
    try:
        for tok in tokenize.generate_tokens(io.StringIO(src).readline):
            if tok.type == tokenize.STRING and prev in (tokenize.INDENT, tokenize.NEWLINE, tokenize.DEDENT):
                prev = tok.type
                continue  # docstring
            if tok.type not in (tokenize.COMMENT, tokenize.NL, tokenize.NEWLINE, tokenize.INDENT,
                                tokenize.DEDENT, tokenize.ENDMARKER):
                for ln in range(tok.start[0], tok.end[0] + 1):
                    lines.add(ln)
            if tok.type not in (tokenize.COMMENT, tokenize.NL):
                prev = tok.type
    except tokenize.TokenError:
        return sum(1 for ln in src.splitlines() if ln.strip() and not ln.strip().startswith("#"))
    return len(lines)


def complexity(planner: str) -> dict:
    man = get_planner(planner).manifest
    root = Path(uninbench.__file__).resolve().parent.parent
    files = {m: sloc(root / m) for m in man.core_modules}
    return {"planner": man.name, "provenance": man.provenance, "sloc_core": sum(files.values()), "files": files,
            "hyperparameters": len(man.hyperparameters), "stages": len(man.stages) if man.mode == "staged" else 1,
            "mode": man.mode, "external_dependencies": man.external_dependencies, "parallelism": man.parallelism,
            "composition_of": man.composition_of}

"""Validation of (re)implementations before they enter an experiment (R9).

Every planner has validation cases that reproduce worked examples or
properties stated in the original publication, with declared tolerances that
account for numerical, discretisation and implementation differences.
Planners failing validation are corrected or excluded, and the deviation is
documented in the validation report.  :mod:`.published` compares the
tree-cover planners with published coverage times on the authors' instances.
:mod:`.reference` additionally
re-evaluates the reference routes of the AreaCoverage dataset as a
planner-independent sanity check of the evaluator.
"""

from .cases import CASES, Expectation, ValidationCase, run_case, validate_all, validate_planner, validation_passed

__all__ = ["CASES", "Expectation", "ValidationCase", "run_case", "validate_all", "validate_planner",
           "validation_passed"]

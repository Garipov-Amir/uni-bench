"""Unified benchmarking framework for multi-robot coverage path planning.

The package implements the layered pipeline

    canonical instance E_c -> adapter T_i -> representation R_i
        -> planner P_i -> trajectories Pi_i -> evaluator M(E_c, Pi_i)

together with the experiment runner, tuning, statistical reporting and the
replay simulator.  See README.md for the mapping of requirements R1-R12 to
modules.
"""

__version__ = "0.1.0"

TASK_ONE_SHOT = "one_shot_direct_coverage"

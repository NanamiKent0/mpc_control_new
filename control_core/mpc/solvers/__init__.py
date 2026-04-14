"""Compatibility exports for legacy solver import paths."""

from .pair_dock_mpc import PairDockMPC, PairDockMPCOutput
from .scalar_distance_mpc import ScalarDistanceMPC, ScalarDistanceMPCOutput

__all__ = [
    "PairDockMPC",
    "PairDockMPCOutput",
    "ScalarDistanceMPC",
    "ScalarDistanceMPCOutput",
]

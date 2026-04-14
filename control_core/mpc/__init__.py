"""Generic MPC templates used by relation and pair controller families."""

from .cooperate_mpc import CooperateMPC, CooperateMPCOutput
from .pair_dock_mpc import PairDockMPC, PairDockMPCOutput
from .posture_adjust_mpc import PostureAdjustMPC, PostureAdjustMPCOutput
from .scalar_feed_mpc import ScalarFeedMPC, ScalarFeedMPCOutput

__all__ = [
    "CooperateMPC",
    "CooperateMPCOutput",
    "PairDockMPC",
    "PairDockMPCOutput",
    "PostureAdjustMPC",
    "PostureAdjustMPCOutput",
    "ScalarFeedMPC",
    "ScalarFeedMPCOutput",
]

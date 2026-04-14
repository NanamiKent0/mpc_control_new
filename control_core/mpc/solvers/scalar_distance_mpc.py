"""Compatibility wrapper for the generic scalar feed MPC."""

from __future__ import annotations

from ..scalar_feed_mpc import ScalarFeedMPC as ScalarDistanceMPC
from ..scalar_feed_mpc import ScalarFeedMPCOutput as ScalarDistanceMPCOutput

__all__ = ["ScalarDistanceMPC", "ScalarDistanceMPCOutput"]

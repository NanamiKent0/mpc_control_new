"""Compatibility wrapper that re-exports the standalone sim ROS2 bridge."""

from __future__ import annotations

from .sim_backend.ros2_backend import SimRos2BackendBridge as SimRos2Bridge

__all__ = ["SimRos2Bridge"]

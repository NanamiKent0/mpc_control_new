"""Smoke tests for the ROS2-style runtime session path."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mpc_control_new.runtime_integration.runtime_session import build_ros2_runtime_session


class RuntimeSessionRos2SmokeTest(unittest.TestCase):
    """Verify the ROS2 runtime session can be constructed without hardware."""

    def test_ros2_runtime_session_reports_unavailable_provider_without_crashing(self) -> None:
        """A default ROS2 session should degrade cleanly when ROS2 resources are unavailable or disabled."""
        session = build_ros2_runtime_session()

        result = session.step()

        self.assertFalse(result.accepted)
        self.assertEqual(result.reason, "observation_unavailable")
        self.assertEqual(result.diagnostics["provider_kind"], "ros2")
        self.assertEqual(result.diagnostics["dispatcher_kind"], "ros2")
        self.assertEqual(result.diagnostics["input_source"], "runtime_frame:ros2")
        self.assertEqual(result.diagnostics["provider_status"]["ros2_available"], False)


if __name__ == "__main__":
    unittest.main()

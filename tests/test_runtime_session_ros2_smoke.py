"""Smoke tests for the ROS2-style runtime session path."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mpc_control_new.runtime_integration.ros2_runtime_node import Ros2RuntimeNode
from mpc_control_new.runtime_integration.runtime_session import build_ros2_runtime_session


class _FakePublisher:
    """Simple fake ROS2 publisher."""

    def __init__(self) -> None:
        self.payloads: list[object] = []

    def publish(self, payload: object) -> None:
        """Record one payload."""
        self.payloads.append(payload)


class _FakeRos2NodeHandle:
    """Simple fake ROS2 node handle used by runtime-session tests."""

    def __init__(self) -> None:
        self.publisher = _FakePublisher()
        self.subscriptions: list[object] = []

    def create_subscription(self, msg_type: object, topic: str, callback: object, qos: int) -> object:
        """Record one callback."""
        del msg_type, topic, qos
        self.subscriptions.append(callback)
        return callback

    def create_publisher(self, msg_type: object, topic: str, qos: int) -> _FakePublisher:
        """Return the fake publisher."""
        del msg_type, topic, qos
        return self.publisher

    def destroy_node(self) -> None:
        """Compatibility no-op."""
        return None


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
        self.assertFalse(result.diagnostics["legacy_path_used"])

    def test_ros2_runtime_session_runs_with_fake_node_shim(self) -> None:
        """A fake ROS2 node should let the runtime session exercise the new main path."""
        runtime_node = Ros2RuntimeNode(node_handle=_FakeRos2NodeHandle())
        session = build_ros2_runtime_session(runtime_node=runtime_node)

        session.start()
        runtime_node.push_observation(
            {
                "timestamp_ns": 123,
                "modules": [
                    {"module_id": "joint1", "module_type": "joint", "dofs": {"crawl_mm": 0.0}},
                    {"module_id": "tip", "module_type": "tip", "dofs": {"growth_mm": 0.0}},
                    {"module_id": "joint2", "module_type": "joint", "dofs": {"crawl_mm": 0.0}},
                ],
                "pairs": [
                    {
                        "active_module": "joint1",
                        "passive_module": "tip",
                        "relation_type": "tip_joint",
                        "distance_mm": 16.0,
                        "orientation_error_deg": -4.0,
                        "coupled": False,
                        "observation_valid": True,
                    }
                ],
            }
        )

        result = session.step()

        self.assertTrue(result.accepted)
        self.assertEqual(result.provider_kind, "ros2")
        self.assertEqual(result.dispatcher_kind, "ros2")
        self.assertEqual(result.dispatch_target, "ros2")
        self.assertFalse(result.diagnostics["legacy_path_used"])


if __name__ == "__main__":
    unittest.main()

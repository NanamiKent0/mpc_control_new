"""Fake loopback tests for the standalone sim ROS2 backend."""

from __future__ import annotations

import json
import sys
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mpc_control_new.runtime_integration.ros2_runtime_node import (
    LoopbackRos2Broker,
    LoopbackRclpyContext,
)
from mpc_control_new.runtime_integration.sim_backend.ros2_backend import (
    SimRos2BackendBridge,
    SimRos2RuntimeDependencies,
)
from mpc_control_new.runtime_integration.sim_backend.types import SimConfig


class _ArrayMessage:
    def __init__(self, data: list[int]) -> None:
        self.data = list(data)


class SimBackendRos2LoopFakeTest(unittest.TestCase):
    """Verify the standalone sim backend speaks the current GUI ROS2 topic contract."""

    def test_bridge_accepts_gui_command_and_publishes_feedback_and_state(self) -> None:
        broker = LoopbackRos2Broker()
        runtime = SimRos2RuntimeDependencies(
            rclpy_module=LoopbackRclpyContext(),
            node_factory=broker.create_node,
        )
        bridge = SimRos2BackendBridge(
            node_handle=broker.create_node("sim_backend"),
            runtime=runtime,
            config=SimConfig(dt=0.1),
        )
        bridge.start()
        try:
            feedback_frames: list[list[int]] = []
            state_payloads: list[dict[str, object]] = []
            capture_node = broker.create_node("sim_capture")
            capture_node.create_subscription(
                object,
                "joint1/motor_feedback",
                lambda msg: feedback_frames.append(list(msg.data)),
                10,
            )
            capture_node.create_subscription(
                object,
                "sim/state",
                lambda msg: state_payloads.append(json.loads(msg.data)),
                10,
            )
            command_publisher = capture_node.create_publisher(object, "joint1/motor_command", 10)
            command_publisher.publish(_ArrayMessage([1, 0x02, 4550, 0, 0]))
            bridge.step_once()

            snapshot = bridge.snapshot()
            self.assertEqual(snapshot["dispatcher"]["dispatcher_kind"], "sim")
            self.assertEqual(snapshot["provider"]["frame_origin"], "sim")
            self.assertEqual(snapshot["latest_raw_commands"]["joint1"], [1, 2, 4550, 0, 0])
            self.assertGreater(snapshot["backend"]["c1"], 0.0)
            self.assertTrue(feedback_frames)
            self.assertTrue(state_payloads)
            self.assertEqual(state_payloads[-1]["diagnostics"]["state_topic"], "sim/state")
            self.assertIn("joint5", state_payloads[-1]["diagnostics"]["namespaces"])
        finally:
            bridge.stop()

    def test_bridge_accepts_joint5_command_and_publishes_joint5_feedback(self) -> None:
        broker = LoopbackRos2Broker()
        runtime = SimRos2RuntimeDependencies(
            rclpy_module=LoopbackRclpyContext(),
            node_factory=broker.create_node,
        )
        bridge = SimRos2BackendBridge(
            node_handle=broker.create_node("sim_backend_joint5"),
            runtime=runtime,
            config=SimConfig(dt=0.1),
        )
        bridge.start()
        try:
            feedback_frames: list[list[int]] = []
            capture_node = broker.create_node("sim_capture_joint5")
            capture_node.create_subscription(
                object,
                "joint5/motor_feedback",
                lambda msg: feedback_frames.append(list(msg.data)),
                10,
            )
            command_publisher = capture_node.create_publisher(object, "joint5/motor_command", 10)
            command_publisher.publish(_ArrayMessage([1, 0x02, 4550, 0, 0]))
            bridge.step_once()

            snapshot = bridge.snapshot()
            self.assertEqual(snapshot["latest_raw_commands"]["joint5"], [1, 2, 4550, 0, 0])
            self.assertGreater(snapshot["backend"]["c5"], 0.0)
            self.assertTrue(feedback_frames)
            self.assertTrue(any(frame[0] == 1 for frame in feedback_frames))
        finally:
            bridge.stop()


if __name__ == "__main__":
    unittest.main()

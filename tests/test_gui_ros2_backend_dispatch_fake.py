"""Fake GUI backend-dispatch tests covering none/sim/live/both modes."""

from __future__ import annotations

import os
import sys
import unittest
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt5.QtWidgets import QApplication

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mpc_control_new.runtime_integration.gui.gui_ros2 import (
    BACKEND_SOURCE_LIVE,
    BACKEND_SOURCE_SIM,
    MultiJointControlGUI,
    Ros2GuiRuntimeDependencies,
)
from mpc_control_new.runtime_integration.ros2_runtime_node import (
    LoopbackRos2Broker,
    LoopbackRclpyContext,
)


class _ArrayMessage:
    def __init__(self, data: list[int]) -> None:
        self.data = list(data)


class GuiRos2BackendDispatchFakeTest(unittest.TestCase):
    """Verify `gui_ros2.py` is the one GUI entry for backend discovery and fan-out."""

    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QApplication.instance() or QApplication([])

    def test_no_backend_mode_is_explicit_and_safe(self) -> None:
        window = MultiJointControlGUI(
            enable_gamepad_thread=False,
            enable_live_backend=False,
            enable_embedded_sim=False,
        )
        try:
            joint1 = window.joint_map["关节1"]
            joint1.send_single_velocity_command(1)
            self.app.processEvents()

            summary = window.backend_manager.status_summary()
            self.assertEqual(summary["dispatch_mode"], "none")
            self.assertFalse(summary["backends"]["sim"]["online"])
            self.assertFalse(summary["backends"]["live"]["online"])
            self.assertIn("当前无可用 backend", window.log_text.toPlainText())
        finally:
            window.shutdown()
            window.close()

    def test_live_backend_is_auto_connected_and_receives_feedback(self) -> None:
        broker = LoopbackRos2Broker()
        runtime = Ros2GuiRuntimeDependencies(
            rclpy_module=LoopbackRclpyContext(),
            node_factory=broker.create_node,
        )
        captured_commands: list[list[int]] = []
        capture_node = broker.create_node("live_capture")
        capture_node.create_subscription(
            object,
            "joint1/motor_command",
            lambda msg: captured_commands.append(list(msg.data)),
            10,
        )
        window = MultiJointControlGUI(
            ros_runtime=runtime,
            enable_gamepad_thread=False,
            enable_live_backend=True,
            enable_embedded_sim=False,
        )
        try:
            joint1 = window.joint_map["关节1"]
            joint1.set_velocity_physical(1, 5.0)
            broker.publish("joint1/motor_feedback", _ArrayMessage([1, 4550, 1, 1, 1, 0]))
            self.app.processEvents()

            self.assertEqual(window.backend_manager.dispatch_mode(), "live")
            self.assertTrue(any(command[1] == 0x02 for command in captured_commands))
            self.assertIn(1, joint1.ros_system.latest_feedback_by_source["live"])
            self.assertGreater(joint1.get_physical_positions(source="live")[0], 0.0)
        finally:
            window.shutdown()
            window.close()

    def test_both_backends_fan_out_and_keep_feedback_separate(self) -> None:
        live_broker = LoopbackRos2Broker()
        runtime = Ros2GuiRuntimeDependencies(
            rclpy_module=LoopbackRclpyContext(),
            node_factory=live_broker.create_node,
        )
        live_commands: list[list[int]] = []
        live_capture = live_broker.create_node("live_capture")
        live_capture.create_subscription(
            object,
            "joint1/motor_command",
            lambda msg: live_commands.append(list(msg.data)),
            10,
        )
        window = MultiJointControlGUI(
            ros_runtime=runtime,
            enable_gamepad_thread=False,
            enable_live_backend=True,
            enable_embedded_sim=True,
            sim_auto_step_ms=0,
        )
        try:
            sim_backend = window.backend_manager.backend_for_source(BACKEND_SOURCE_SIM)
            self.assertIsNotNone(sim_backend)
            sim_commands: list[list[int]] = []
            sim_capture = sim_backend.broker.create_node("sim_capture")
            sim_capture.create_subscription(
                object,
                "joint1/motor_command",
                lambda msg: sim_commands.append(list(msg.data)),
                10,
            )
            joint1 = window.joint_map["关节1"]
            joint1.set_velocity_physical(1, 5.0)
            window.backend_manager.step_backend(BACKEND_SOURCE_SIM, count=3)
            live_broker.publish("joint1/motor_feedback", _ArrayMessage([1, 9100, 1, 1, 1, 0]))
            self.app.processEvents()

            self.assertEqual(window.backend_manager.dispatch_mode(), "both")
            self.assertTrue(any(command[1] == 0x02 for command in sim_commands))
            self.assertTrue(any(command[1] == 0x02 for command in live_commands))
            self.assertIn(1, joint1.ros_system.latest_feedback_by_source["sim"])
            self.assertIn(1, joint1.ros_system.latest_feedback_by_source["live"])
            self.assertNotEqual(
                joint1.position_status["sim"][1].text(),
                joint1.position_status["live"][1].text(),
            )
            self.assertIn("[sim]", window.log_text.toPlainText())
            self.assertIn("[live]", window.log_text.toPlainText())
        finally:
            window.shutdown()
            window.close()

    def test_global_emergency_stop_broadcasts_to_sim_and_live(self) -> None:
        live_broker = LoopbackRos2Broker()
        runtime = Ros2GuiRuntimeDependencies(
            rclpy_module=LoopbackRclpyContext(),
            node_factory=live_broker.create_node,
        )
        live_commands: list[list[int]] = []
        live_capture = live_broker.create_node("live_estop_capture")
        live_capture.create_subscription(
            object,
            "joint1/motor_command",
            lambda msg: live_commands.append(list(msg.data)),
            10,
        )
        window = MultiJointControlGUI(
            ros_runtime=runtime,
            enable_gamepad_thread=False,
            enable_live_backend=True,
            enable_embedded_sim=True,
            sim_auto_step_ms=0,
        )
        try:
            sim_backend = window.backend_manager.backend_for_source(BACKEND_SOURCE_SIM)
            self.assertIsNotNone(sim_backend)
            sim_commands: list[list[int]] = []
            sim_capture = sim_backend.broker.create_node("sim_estop_capture")
            sim_capture.create_subscription(
                object,
                "joint1/motor_command",
                lambda msg: sim_commands.append(list(msg.data)),
                10,
            )
            window.global_emergency_stop()
            self.app.processEvents()

            self.assertEqual(window.backend_manager.dispatch_mode(), "both")
            self.assertTrue(any(command[1] == 0x01 for command in sim_commands))
            self.assertTrue(any(command[1] == 0x01 for command in live_commands))
        finally:
            window.shutdown()
            window.close()


if __name__ == "__main__":
    unittest.main()

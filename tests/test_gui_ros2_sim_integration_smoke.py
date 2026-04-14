"""Smoke tests for the canonical gui_ros2 + self-contained sim bridge."""

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
    MultiJointControlGUI,
    BACKEND_SOURCE_SIM,
)


class GuiRos2SimIntegrationSmokeTest(unittest.TestCase):
    """Verify the canonical GUI can talk to the self-contained sim through topic semantics."""

    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QApplication.instance() or QApplication([])

    def test_gui_receives_sim_feedback_through_ros2_style_topics(self) -> None:
        window = MultiJointControlGUI(
            enable_gamepad_thread=False,
            enable_embedded_sim=True,
            enable_live_backend=False,
            sim_auto_step_ms=0,
        )
        try:
            joint1_widget = window.joint_map["关节1"]
            sim_backend = window.backend_manager.backend_for_source(BACKEND_SOURCE_SIM)
            self.assertIsNotNone(sim_backend)
            captured_commands: list[list[int]] = []
            capture_node = sim_backend.broker.create_node("sim_capture")
            capture_node.create_subscription(
                object,
                "joint1/motor_command",
                lambda msg: captured_commands.append(list(msg.data)),
                10,
            )
            joint1_widget.set_velocity_physical(1, 5.0)
            window.backend_manager.step_backend(BACKEND_SOURCE_SIM, count=3)
            self.app.processEvents()

            self.assertEqual(window.backend_manager.dispatch_mode(), "sim")
            self.assertTrue(any(command[1] == 0x02 for command in captured_commands))
            self.assertIn(1, joint1_widget.ros_system.latest_feedback_by_source["sim"])
            self.assertGreater(joint1_widget.get_physical_positions(source="sim")[0], 0.0)
            self.assertEqual(sim_backend.bridge.snapshot()["provider"]["frame_origin"], "sim")
            self.assertTrue(sim_backend.bridge.snapshot()["provider"]["ros2_gui_compatible"])
        finally:
            window.shutdown()
            window.close()

    def test_gui_can_drive_joint5_against_embedded_sim_backend(self) -> None:
        window = MultiJointControlGUI(
            enable_gamepad_thread=False,
            enable_embedded_sim=True,
            enable_live_backend=False,
            sim_auto_step_ms=0,
        )
        try:
            joint5_widget = window.joint_map["关节5"]
            sim_backend = window.backend_manager.backend_for_source(BACKEND_SOURCE_SIM)
            self.assertIsNotNone(sim_backend)
            captured_commands: list[list[int]] = []
            capture_node = sim_backend.broker.create_node("sim_capture_joint5")
            capture_node.create_subscription(
                object,
                "joint5/motor_command",
                lambda msg: captured_commands.append(list(msg.data)),
                10,
            )
            joint5_widget.set_velocity_physical(1, 5.0)
            window.backend_manager.step_backend(BACKEND_SOURCE_SIM, count=3)
            self.app.processEvents()

            self.assertTrue(any(command[1] == 0x02 for command in captured_commands))
            self.assertIn(1, joint5_widget.ros_system.latest_feedback_by_source["sim"])
            self.assertGreater(joint5_widget.get_physical_positions(source="sim")[0], 0.0)
        finally:
            window.shutdown()
            window.close()


if __name__ == "__main__":
    unittest.main()

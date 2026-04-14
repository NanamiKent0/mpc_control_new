"""Fake self-containment tests for the canonical gui_ros2 entry."""

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
    ROS2ControlSystem,
    Ros2GuiRuntimeDependencies,
    launch_gui_ros2,
)
from mpc_control_new.runtime_integration.ros2_runtime_node import (
    LoopbackRos2Broker,
    LoopbackRclpyContext,
)


class GuiRos2SelfContainedFakeTest(unittest.TestCase):
    """Verify gui_ros2 can be imported and built headlessly inside the new package."""

    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QApplication.instance() or QApplication([])

    def test_gui_direct_entry_builds_headlessly_without_backends(self) -> None:
        summary = launch_gui_ros2(
            headless=True,
            enable_live_backend=False,
            enable_embedded_sim=False,
            enable_gamepad=False,
        )
        self.assertEqual(summary["dispatch_mode"], "none")
        self.assertEqual(summary["self_contained"], True)
        self.assertIn("关节1", summary["joint_names"])

    def test_gui_builds_headlessly_without_old_project_dependencies(self) -> None:
        window = MultiJointControlGUI(
            enable_gamepad_thread=False,
            enable_live_backend=False,
            enable_embedded_sim=False,
        )
        try:
            self.assertIn("关节1", window.joint_map)
            self.assertEqual(window.joint_map["关节1"].topic_ns, "joint1")
            self.assertEqual(window.windowTitle(), "ESP32 ROS2 机器人控制系统")
            self.assertEqual(window.backend_manager.dispatch_mode(), "none")
        finally:
            window.shutdown()
            window.close()

    def test_ros2_control_system_can_start_on_loopback_runtime(self) -> None:
        broker = LoopbackRos2Broker()
        runtime = Ros2GuiRuntimeDependencies(
            rclpy_module=LoopbackRclpyContext(),
            node_factory=broker.create_node,
        )
        control = ROS2ControlSystem("joint1", ros_runtime=runtime)
        try:
            node = control.start_ros()
            self.assertIsNotNone(node)
            self.assertTrue(control.connected)
            self.assertTrue(control.send_compact_command(1, 0x02, 10, 0, 0))
        finally:
            control.stop_ros()


if __name__ == "__main__":
    unittest.main()

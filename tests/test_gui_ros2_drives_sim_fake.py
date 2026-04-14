"""Fake end-to-end test for GUI -> standalone sim backend -> visualizer flow."""

from __future__ import annotations

import os
import sys
import unittest
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("MPLBACKEND", "Agg")

from PyQt5.QtWidgets import QApplication

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mpc_control_new.runtime_integration.gui.gui_ros2 import MultiJointControlGUI, Ros2GuiRuntimeDependencies
from mpc_control_new.runtime_integration.ros2_runtime_node import LoopbackRos2Broker, LoopbackRclpyContext
from mpc_control_new.runtime_integration.sim_backend.ros2_backend import (
    SimRos2BackendBridge,
    SimRos2RuntimeDependencies,
)
from mpc_control_new.runtime_integration.sim_backend.visualizer import SimStateListener, SimVisualizer


class GuiRos2DrivesSimFakeTest(unittest.TestCase):
    """Verify the canonical GUI can drive the standalone sim stack over ROS2-style topics."""

    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QApplication.instance() or QApplication([])

    def test_gui_command_reaches_sim_and_visualizer_receives_state(self) -> None:
        broker = LoopbackRos2Broker()
        gui_runtime = Ros2GuiRuntimeDependencies(
            rclpy_module=LoopbackRclpyContext(),
            node_factory=broker.create_node,
        )
        sim_runtime = SimRos2RuntimeDependencies(
            rclpy_module=LoopbackRclpyContext(),
            node_factory=broker.create_node,
        )
        backend = SimRos2BackendBridge(
            node_handle=broker.create_node("standalone_sim_backend"),
            runtime=sim_runtime,
        )
        listener = SimStateListener(
            node_handle=broker.create_node("standalone_sim_visualizer_listener"),
            runtime=sim_runtime,
        )
        command_frames: list[list[int]] = []
        capture_node = broker.create_node("standalone_sim_capture")
        capture_node.create_subscription(
            object,
            "joint1/motor_command",
            lambda msg: command_frames.append(list(msg.data)),
            10,
        )
        backend.start()
        listener.start()
        window = MultiJointControlGUI(
            ros_runtime=gui_runtime,
            enable_gamepad_thread=False,
            enable_live_backend=True,
            enable_embedded_sim=False,
        )
        try:
            joint1_widget = window.joint_map["关节1"]
            joint1_widget.set_velocity_physical(1, 5.0)
            backend.step_once()
            backend.step_once()
            self.app.processEvents()

            latest_state = listener.get_latest_state()
            visualizer = SimVisualizer(state_listener=listener)
            render_payload = visualizer.consume_state_snapshot(listener.get_latest_payload())

            self.assertEqual(window.backend_manager.dispatch_mode(), "live")
            self.assertTrue(any(command[1] == 0x02 for command in command_frames))
            self.assertIn(1, joint1_widget.ros_system.latest_feedback_by_source["live"])
            self.assertIsNotNone(latest_state)
            self.assertGreater(latest_state.c1, 0.0)
            self.assertTrue(render_payload["module_paths"]["joint1"][0])
            self.assertIn("seq=", render_payload["status_text"])
        finally:
            window.shutdown()
            window.close()
            listener.stop()
            backend.stop()


if __name__ == "__main__":
    unittest.main()

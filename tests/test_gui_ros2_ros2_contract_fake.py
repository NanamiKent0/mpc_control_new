"""Fake tests for the canonical gui_ros2 ROS2 topic contract."""

from __future__ import annotations

import os
import sys
import unittest
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mpc_control_new.control_core.models.runtime_bridge_types import (
    SchedulerDispatchEnvelope,
    ScheduledSkillCommand,
)
from mpc_control_new.control_core.models.skill_types import PrimitiveReference
from mpc_control_new.control_core.models.task_types import SchedulerState
from mpc_control_new.runtime_integration.gui.gui_ros2 import (
    ROS2ControlSystem,
    Ros2GuiRuntimeDependencies,
)
from mpc_control_new.runtime_integration.live_command_dispatcher import LiveRuntimeCommandDispatcher
from mpc_control_new.runtime_integration.live_runtime_provider import LiveRuntimeObservationProvider
from mpc_control_new.runtime_integration.ros2_runtime_node import (
    LoopbackRos2Broker,
    LoopbackRclpyContext,
    Ros2RuntimeNode,
)


class _ArrayMessage:
    def __init__(self, data: list[int]) -> None:
        self.data = list(data)


class GuiRos2Ros2ContractFakeTest(unittest.TestCase):
    """Verify the live ROS2 adapter now speaks the same compact topic contract as gui_ros2."""

    def test_unavailable_default_ros2_contract_is_structured(self) -> None:
        provider = LiveRuntimeObservationProvider()
        dispatcher = LiveRuntimeCommandDispatcher()
        provider.start()
        dispatcher.start()

        self.assertIsNone(provider.get_latest_frame())
        self.assertFalse(provider.runtime_diagnostics()["ros2_available"])
        self.assertFalse(dispatcher.runtime_diagnostics()["ros2_available"])

    def test_gui_and_live_runtime_share_motor_topics_under_loopback(self) -> None:
        broker = LoopbackRos2Broker()
        gui_runtime = Ros2GuiRuntimeDependencies(
            rclpy_module=LoopbackRclpyContext(),
            node_factory=broker.create_node,
        )
        runtime_node = Ros2RuntimeNode(node_handle=broker.create_node("runtime_node"))
        provider = LiveRuntimeObservationProvider(runtime_node=runtime_node)
        dispatcher = LiveRuntimeCommandDispatcher(runtime_node=runtime_node)
        control = ROS2ControlSystem("joint1", ros_runtime=gui_runtime)
        captured_commands: list[list[int]] = []
        capture_node = broker.create_node("capture_node")
        capture_node.create_subscription(
            object,
            "joint1/motor_command",
            lambda msg: captured_commands.append(list(msg.data)),
            10,
        )

        try:
            control.start_ros()
            provider.start()
            dispatcher.start()
            result = dispatcher.dispatch(
                SchedulerDispatchEnvelope(
                    scheduler_state=SchedulerState(graph_id="gui_ros2_contract"),
                    scheduled_command=ScheduledSkillCommand(
                        graph_id="gui_ros2_contract",
                        node_id="coarse_approach",
                        skill_key="coarse_approach",
                        active_module="joint1",
                        passive_module="tip",
                        relation_type="tip_joint",
                        selected_primitives=["joint_crawl"],
                        primitive_references=[
                            PrimitiveReference(
                                module_id="joint1",
                                primitive_name="joint_crawl",
                                axis="crawl",
                                reference_kind="velocity",
                                reference_value=2.0,
                                units="mm/s",
                                primary=True,
                                semantic="reduce_distance",
                            )
                        ],
                    ),
                    input_source="runtime_frame:ros2",
                    dispatch_target="ros2",
                    provider_kind="ros2",
                    dispatcher_kind="ros2",
                    diagnostics={"scheduler_input_source": "runtime_frame"},
                )
            )
            broker.publish("tip/motor_feedback", _ArrayMessage([4, 0, 1, 1, 1, 0]))
            broker.publish("joint1/motor_feedback", _ArrayMessage([1, 4550, 1, 1, 1, 0]))
            frame = provider.get_latest_frame()

            self.assertTrue(result.accepted)
            self.assertEqual(captured_commands[0][0], 1)
            self.assertEqual(captured_commands[0][1], 0x02)
            self.assertIn(1, control.latest_feedback_by_motor)
            self.assertIsNotNone(frame)
            self.assertTrue(frame.metadata["gui_ros2_compatible"])
            self.assertIn("joint1->tip", frame.pair_observations)
        finally:
            control.stop_ros()
            provider.stop()
            dispatcher.stop()


if __name__ == "__main__":
    unittest.main()

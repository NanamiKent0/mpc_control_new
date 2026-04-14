"""Fake tests for the minimal ROS2 real-path integration."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mpc_control_new.control_core.models.runtime_bridge_types import SchedulerDispatchEnvelope, ScheduledSkillCommand
from mpc_control_new.control_core.models.skill_types import PrimitiveReference
from mpc_control_new.control_core.models.task_types import SchedulerState
from mpc_control_new.runtime_integration.live_command_dispatcher import LiveRuntimeCommandDispatcher
from mpc_control_new.runtime_integration.live_runtime_provider import LiveRuntimeObservationProvider
from mpc_control_new.runtime_integration.ros2_runtime_node import Ros2RuntimeNode


class _FakePublisher:
    """Simple fake ROS2 publisher."""

    def __init__(self) -> None:
        self.payloads: list[object] = []

    def publish(self, payload: object) -> None:
        """Record one payload."""
        self.payloads.append(payload)


class _FakeRos2NodeHandle:
    """Simple fake ROS2 node handle for provider/dispatcher tests."""

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


class Ros2MinimalRealPathFakeTest(unittest.TestCase):
    """Verify the ROS2 main path is now a concrete integration surface."""

    def test_ros2_components_report_structured_unavailable_state(self) -> None:
        """The default ROS2 provider/dispatcher should degrade cleanly without throwing."""
        provider = LiveRuntimeObservationProvider()
        dispatcher = LiveRuntimeCommandDispatcher()
        provider.start()
        dispatcher.start()

        self.assertIsNone(provider.get_latest_frame())
        self.assertEqual(provider.runtime_diagnostics()["provider_kind"], "ros2")
        self.assertEqual(provider.runtime_diagnostics()["ros2_available"], False)
        self.assertEqual(dispatcher.runtime_diagnostics()["dispatcher_kind"], "ros2")
        self.assertEqual(dispatcher.runtime_diagnostics()["ros2_available"], False)

    def test_fake_ros2_node_runs_minimal_provider_dispatcher_path(self) -> None:
        """A fake node shim should let the provider and dispatcher run end to end."""
        runtime_node = Ros2RuntimeNode(node_handle=_FakeRos2NodeHandle())
        provider = LiveRuntimeObservationProvider(runtime_node=runtime_node)
        dispatcher = LiveRuntimeCommandDispatcher(runtime_node=runtime_node)
        provider.start()
        dispatcher.start()
        runtime_node.push_observation(
            {
                "timestamp_ns": 222,
                "modules": [
                    {"module_id": "joint1", "module_type": "joint", "dofs": {"crawl_mm": 1.0}},
                    {"module_id": "tip", "module_type": "tip", "dofs": {"growth_mm": 0.0}},
                    {"module_id": "joint2", "module_type": "joint", "dofs": {"crawl_mm": 0.0}},
                ],
                "pairs": [
                    {
                        "active_module": "joint1",
                        "passive_module": "tip",
                        "relation_type": "tip_joint",
                        "distance_mm": 15.0,
                        "orientation_error_deg": -3.0,
                        "coupled": False,
                        "observation_valid": True,
                    }
                ],
            }
        )
        frame = provider.get_latest_frame()
        result = dispatcher.dispatch(
            SchedulerDispatchEnvelope(
                scheduler_state=SchedulerState(graph_id="ros2_real_path"),
                scheduled_command=ScheduledSkillCommand(
                    graph_id="ros2_real_path",
                    node_id="fine_dock",
                    skill_key="fine_dock",
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
                            reference_value=1.0,
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

        self.assertIsNotNone(frame)
        self.assertEqual(frame.metadata["frame_origin"], "ros2")
        self.assertTrue(result.accepted)
        self.assertEqual(result.diagnostics["dispatcher_kind"], "ros2")
        self.assertEqual(result.diagnostics["published_primitive_count"], 1)


if __name__ == "__main__":
    unittest.main()

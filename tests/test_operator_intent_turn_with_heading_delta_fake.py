"""Fake tests for routing `TIP_TURN(delta)` into the turn workflow."""

from __future__ import annotations

import unittest

from tests.turn_workflow_fake_support import RecordingDispatcher, StaticProvider, build_turn_runtime_frame

from mpc_control_new.control_core.models.task_types import TIP_TURN_AUTONOMOUS
from mpc_control_new.control_core.supervisor.intent_router import route_operator_intent
from mpc_control_new.control_core.supervisor.operator_intent import tip_turn_intent
from mpc_control_new.runtime_integration.runtime_session import RuntimeSession


class OperatorIntentTurnWithHeadingDeltaFakeTest(unittest.TestCase):
    """Verify turn intent keeps the heading delta on the runtime/scheduler path."""

    def test_turn_intent_routes_into_turn_autonomous_with_heading_delta(self) -> None:
        task_request = route_operator_intent(tip_turn_intent(20.0))
        self.assertEqual(task_request.task_kind, TIP_TURN_AUTONOMOUS)
        self.assertEqual(task_request.metadata["operator_intent_kind"], "TIP_TURN")
        self.assertEqual(task_request.metadata["target_heading_delta_deg"], 20.0)

    def test_runtime_session_propagates_turn_heading_delta_into_graph_and_step_diagnostics(self) -> None:
        frame = build_turn_runtime_frame(idle_joint_ids={"joint1"})
        session = RuntimeSession(
            observation_provider=StaticProvider(frame),
            command_dispatcher=RecordingDispatcher(),
            graph_spec=None,
        )

        graph = session.submit_intent(tip_turn_intent(20.0), frame=frame)
        result = session.step()

        self.assertEqual(graph.metadata["high_level_task_kind"], TIP_TURN_AUTONOMOUS)
        self.assertEqual(graph.metadata["operator_intent_kind"], "TIP_TURN")
        self.assertEqual(graph.metadata["target_heading_delta_deg"], 20.0)
        self.assertEqual(result.diagnostics["operator_intent_kind"], "TIP_TURN")
        self.assertEqual(result.diagnostics["target_heading_delta_deg"], 20.0)
        self.assertEqual(result.diagnostics["selected_joint_id"], "joint1")
        self.assertEqual(result.diagnostics["planner_mode"], "direct")
        self.assertEqual(result.diagnostics["current_plan_node_kind"], "front_cooperate")


if __name__ == "__main__":
    unittest.main()

"""Fake tests for the free-growth operator intent route."""

from __future__ import annotations

import unittest

from tests.turn_workflow_fake_support import RecordingDispatcher, StaticProvider, build_turn_runtime_frame

from mpc_control_new.control_core.models.task_types import TIP_FREE_GROWTH
from mpc_control_new.control_core.supervisor.operator_intent import tip_free_growth_intent
from mpc_control_new.runtime_integration.runtime_session import RuntimeSession


class OperatorIntentFreeGrowthFakeTest(unittest.TestCase):
    """Verify free-growth intent stays on the steady-state graph."""

    def test_free_growth_intent_loads_only_tip_free_growth_task(self) -> None:
        frame = build_turn_runtime_frame(idle_joint_ids={"joint1"})
        session = RuntimeSession(
            observation_provider=StaticProvider(frame),
            command_dispatcher=RecordingDispatcher(),
            graph_spec=None,
        )

        graph = session.submit_intent(tip_free_growth_intent())
        result = session.step()

        self.assertEqual(graph.metadata["high_level_task_kind"], TIP_FREE_GROWTH)
        self.assertEqual(graph.metadata["operator_intent_kind"], "TIP_FREE_GROWTH")
        self.assertEqual(graph.metadata["ordered_node_kinds"], ["tip_free_growth"])
        self.assertEqual(result.diagnostics["operator_intent_kind"], "TIP_FREE_GROWTH")
        self.assertEqual(result.diagnostics["current_plan_node_kind"], "tip_free_growth")
        self.assertNotIn("front_cooperate", graph.metadata["ordered_node_kinds"])
        self.assertNotIn("local_transfer", graph.metadata["ordered_node_kinds"])


if __name__ == "__main__":
    unittest.main()

"""Smoke tests for the high-level intent entrypoints on RuntimeSession."""

from __future__ import annotations

import unittest

from tests.turn_workflow_fake_support import RecordingDispatcher, StaticProvider, build_turn_runtime_frame

from mpc_control_new.control_core.supervisor.operator_intent import tip_free_growth_intent, tip_turn_intent
from mpc_control_new.runtime_integration.runtime_session import RuntimeSession


class RuntimeSessionHighLevelIntentSmokeTest(unittest.TestCase):
    """Verify high-level intent helpers drive the runtime session surface."""

    def test_submit_intent_surfaces_turn_diagnostics(self) -> None:
        frame = build_turn_runtime_frame(idle_joint_ids={"joint1"})
        session = RuntimeSession(
            observation_provider=StaticProvider(frame),
            command_dispatcher=RecordingDispatcher(),
            graph_spec=None,
        )

        session.submit_intent(tip_turn_intent(10.0), frame=frame)
        result = session.step()

        self.assertEqual(result.diagnostics["operator_intent_kind"], "TIP_TURN")
        self.assertEqual(result.diagnostics["target_heading_delta_deg"], 10.0)
        self.assertEqual(result.diagnostics["selected_joint_id"], "joint1")
        self.assertEqual(result.diagnostics["planner_mode"], "direct")
        self.assertFalse(result.diagnostics["legacy_path_used"])

    def test_set_operator_intent_alias_loads_free_growth(self) -> None:
        frame = build_turn_runtime_frame(idle_joint_ids={"joint1"})
        session = RuntimeSession(
            observation_provider=StaticProvider(frame),
            command_dispatcher=RecordingDispatcher(),
            graph_spec=None,
        )

        graph = session.set_operator_intent(tip_free_growth_intent())

        self.assertEqual(graph.metadata["operator_intent_kind"], "TIP_FREE_GROWTH")
        self.assertEqual(graph.metadata["target_heading_delta_deg"], None)


if __name__ == "__main__":
    unittest.main()

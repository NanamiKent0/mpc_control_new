"""Fake end-to-end test for direct front cooperation driven by turn intent."""

from __future__ import annotations

import unittest

from tests.turn_workflow_fake_support import (
    RecordingDispatcher,
    SequenceProvider,
    build_turn_runtime_frame,
)

from mpc_control_new.control_core.controllers.turn_reference_mapper import module_heading_deg
from mpc_control_new.control_core.kinematics import compute_chain_snapshot
from mpc_control_new.control_core.supervisor.operator_intent import tip_turn_intent
from mpc_control_new.runtime_integration.runtime_session import RuntimeSession
from mpc_control_new.runtime_integration.runtime_state_builder import build_module_state_map


def _frame_for_tip_heading(target_heading_deg: float) -> object:
    best_frame = None
    best_error = None
    for bend_deg in range(-90, 91):
        for rotate_deg in range(-180, 181, 10):
            candidate = build_turn_runtime_frame(
                idle_joint_ids={"joint1"},
                module_overrides={
                    "joint1": {
                        "dofs": {
                            "crawl_mm": 0.0,
                            "bend_deg": float(bend_deg),
                            "rotate_deg": float(rotate_deg),
                        },
                    }
                },
            )
            module_states = build_module_state_map(candidate)
            snapshot = compute_chain_snapshot(
                module_states,
                ordered_modules=("tip", "joint1", "joint2", "joint3", "joint4", "joint5"),
            )
            tip_heading_deg = module_heading_deg(snapshot, "tip")
            if tip_heading_deg is None:
                continue
            error = abs(((float(tip_heading_deg) - float(target_heading_deg) + 180.0) % 360.0) - 180.0)
            if best_error is None or error < best_error:
                best_error = error
                best_frame = candidate
    assert best_frame is not None
    return best_frame


class TurnIntentDrivesDirectFrontCooperateFakeTest(unittest.TestCase):
    """Verify `TIP_TURN(+20)` drives direct joint1-tip cooperation."""

    def test_tip_turn_direct_path_converges_then_returns_to_free_growth(self) -> None:
        initial_frame = build_turn_runtime_frame(
            idle_joint_ids={"joint1"},
            module_overrides={
                "joint1": {
                    "dofs": {"crawl_mm": 0.0, "bend_deg": 0.0, "rotate_deg": 0.0},
                }
            },
        )
        settled_frame = build_turn_runtime_frame(
            idle_joint_ids={"joint1"},
            module_overrides={
                "joint1": {
                    "dofs": {"crawl_mm": 0.0, "bend_deg": 0.0, "rotate_deg": 20.0},
                }
            },
        )
        session = RuntimeSession(
            observation_provider=SequenceProvider([initial_frame, settled_frame]),
            command_dispatcher=RecordingDispatcher(),
            graph_spec=None,
        )

        graph = session.submit_intent(tip_turn_intent(20.0), frame=initial_frame)
        session.observation_provider.frames = [
            initial_frame,
            _frame_for_tip_heading(float(graph.metadata["tip_heading_target_deg"])),
        ]
        results = [session.step() for _ in range(4)]

        self.assertEqual(graph.metadata["selected_joint_id"], "joint1")
        self.assertEqual(graph.metadata["planner_mode"], "direct")
        self.assertEqual(results[0].diagnostics["current_plan_node_kind"], "front_cooperate")
        self.assertAlmostEqual(
            abs(float(results[0].scheduler_step_result.skill_result.diagnostics["heading_error_deg"])),
            20.0,
            places=3,
        )
        self.assertEqual(results[-1].diagnostics["current_plan_node_kind"], "tip_free_growth")
        self.assertTrue(results[-1].scheduler_step_result.scheduler_state.is_finished)


if __name__ == "__main__":
    unittest.main()

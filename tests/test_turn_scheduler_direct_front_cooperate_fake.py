"""Fake scheduler test for the direct `joint1->tip` turn workflow."""

from __future__ import annotations

import unittest

from tests.turn_workflow_fake_support import build_turn_runtime_frame, run_scheduler_until_finished

from mpc_control_new.control_core.orchestration.skill_scheduler import SkillScheduler
from mpc_control_new.control_core.supervisor.turn_task_supervisor import (
    compile_turn_autonomous_request,
)


class TurnSchedulerDirectFrontCooperateFakeTest(unittest.TestCase):
    """Verify the scheduler executes the direct turn path end to end."""

    def test_joint1_idle_runs_front_cooperate_then_returns_to_tip_free_growth(self) -> None:
        frame = build_turn_runtime_frame(idle_joint_ids={"joint1"})
        _, graph = compile_turn_autonomous_request(
            frame,
            target_heading_delta_deg=0.0,
            graph_id="turn_scheduler_direct",
        )
        scheduler = SkillScheduler()
        scheduler.load_graph(graph)

        results = run_scheduler_until_finished(scheduler, frame)

        self.assertEqual([result.node.node_id for result in results], [
            "front_cooperate_joint1_to_tip",
            "return_to_tip_free_growth",
            "tip_free_growth",
        ])
        self.assertEqual(results[0].diagnostics["skill_key"], "front_cooperate")
        self.assertEqual(results[0].diagnostics["planner_mode"], "direct")
        self.assertEqual(results[0].diagnostics["current_plan_node_kind"], "front_cooperate")
        self.assertEqual(results[0].diagnostics["current_active_pair"], "joint1->tip")
        self.assertTrue(results[-1].scheduler_state.is_finished)
        self.assertEqual(results[-1].diagnostics["current_plan_node_kind"], "tip_free_growth")
        self.assertTrue(results[-1].diagnostics["returning_to_tip_free_growth"])


if __name__ == "__main__":
    unittest.main()

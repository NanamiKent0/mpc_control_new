"""Fake scheduler tests for explicit return-to-free-growth completion."""

from __future__ import annotations

import unittest

from tests.turn_workflow_fake_support import build_turn_runtime_frame, run_scheduler_until_finished

from mpc_control_new.control_core.orchestration.skill_scheduler import SkillScheduler
from mpc_control_new.control_core.supervisor.turn_task_supervisor import (
    compile_turn_autonomous_request,
)


class TurnSchedulerReturnsToFreeGrowthFakeTest(unittest.TestCase):
    """Verify every successful turn workflow returns to tip free growth."""

    def test_direct_and_recursive_workflows_both_finish_at_tip_free_growth(self) -> None:
        direct_frame = build_turn_runtime_frame(idle_joint_ids={"joint1"})
        recursive_frame = build_turn_runtime_frame(idle_joint_ids={"joint3"})

        direct_scheduler = SkillScheduler()
        _, direct_graph = compile_turn_autonomous_request(
            direct_frame,
            target_heading_delta_deg=0.0,
            graph_id="turn_returns_direct",
        )
        direct_scheduler.load_graph(direct_graph)
        direct_results = run_scheduler_until_finished(direct_scheduler, direct_frame)

        recursive_scheduler = SkillScheduler()
        _, recursive_graph = compile_turn_autonomous_request(
            recursive_frame,
            target_heading_delta_deg=0.0,
            graph_id="turn_returns_recursive",
        )
        recursive_scheduler.load_graph(recursive_graph)
        recursive_results = run_scheduler_until_finished(recursive_scheduler, recursive_frame)

        self.assertIn("return_to_tip_free_growth", [result.node.node_id for result in direct_results])
        self.assertIn("return_to_tip_free_growth", [result.node.node_id for result in recursive_results])
        self.assertEqual(direct_results[-1].node.node_id, "tip_free_growth")
        self.assertEqual(recursive_results[-1].node.node_id, "tip_free_growth")
        self.assertTrue(direct_results[-1].scheduler_state.is_finished)
        self.assertTrue(recursive_results[-1].scheduler_state.is_finished)


if __name__ == "__main__":
    unittest.main()

"""Fake scheduler test for a recursive turn-transfer workflow."""

from __future__ import annotations

import unittest

from tests.turn_workflow_fake_support import build_turn_runtime_frame, run_scheduler_until_finished

from mpc_control_new.control_core.orchestration.skill_scheduler import SkillScheduler
from mpc_control_new.control_core.supervisor.turn_task_supervisor import (
    compile_turn_autonomous_request,
)


class TurnSchedulerRecursiveTransferFakeTest(unittest.TestCase):
    """Verify recursive transfer steps execute in planner order."""

    def test_joint4_idle_runs_recursive_transfer_chain_then_front_cooperate(self) -> None:
        frame = build_turn_runtime_frame(idle_joint_ids={"joint4"})
        _, graph = compile_turn_autonomous_request(
            frame,
            target_heading_delta_deg=0.0,
            graph_id="turn_scheduler_recursive",
        )
        scheduler = SkillScheduler()
        scheduler.load_graph(graph)

        results = run_scheduler_until_finished(scheduler, frame)

        self.assertEqual([result.node.node_id for result in results], [
            "local_transfer_joint4_to_joint3",
            "local_transfer_joint3_to_joint2",
            "local_transfer_joint2_to_joint1",
            "front_cooperate_joint1_to_tip",
            "return_to_tip_free_growth",
            "tip_free_growth",
        ])
        self.assertEqual(results[0].diagnostics["planner_mode"], "recursive")
        self.assertEqual(
            [result.diagnostics["current_active_pair"] for result in results[:4]],
            ["joint4->joint3", "joint3->joint2", "joint2->joint1", "joint1->tip"],
        )
        self.assertEqual(
            [result.diagnostics["current_plan_node_kind"] for result in results],
            [
                "local_transfer",
                "local_transfer",
                "local_transfer",
                "front_cooperate",
                "return_to_free_growth",
                "tip_free_growth",
            ],
        )


if __name__ == "__main__":
    unittest.main()

"""Fake scheduler test for the longest supported recursive turn workflow."""

from __future__ import annotations

import unittest

from tests.turn_workflow_fake_support import build_turn_runtime_frame, run_scheduler_until_finished

from mpc_control_new.control_core.orchestration.skill_scheduler import SkillScheduler
from mpc_control_new.control_core.supervisor.turn_task_supervisor import (
    compile_turn_autonomous_request,
)


class TurnSchedulerJoint5LastIdleFakeTest(unittest.TestCase):
    """Verify the workflow still converges when only `joint5` is idle."""

    def test_only_joint5_idle_runs_full_recursive_chain(self) -> None:
        frame = build_turn_runtime_frame(idle_joint_ids={"joint5"})
        _, graph = compile_turn_autonomous_request(
            frame,
            target_heading_delta_deg=0.0,
            graph_id="turn_scheduler_joint5",
        )
        scheduler = SkillScheduler()
        scheduler.load_graph(graph)

        results = run_scheduler_until_finished(scheduler, frame)

        self.assertEqual([result.diagnostics["current_active_pair"] for result in results[:5]], [
            "joint5->joint4",
            "joint4->joint3",
            "joint3->joint2",
            "joint2->joint1",
            "joint1->tip",
        ])
        self.assertEqual(results[0].diagnostics["selected_joint_id"], "joint5")
        self.assertEqual(results[0].diagnostics["selected_joint_index"], 5)
        self.assertTrue(results[0].diagnostics["requires_recursive_transfer"])
        self.assertEqual(results[-1].node.node_id, "tip_free_growth")


if __name__ == "__main__":
    unittest.main()

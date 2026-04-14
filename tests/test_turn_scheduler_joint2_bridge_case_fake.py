"""Fake scheduler test for the `joint2 -> joint1 -> tip` bridge case."""

from __future__ import annotations

import unittest

from tests.turn_workflow_fake_support import build_turn_runtime_frame, run_scheduler_until_finished

from mpc_control_new.control_core.orchestration.skill_scheduler import SkillScheduler
from mpc_control_new.control_core.supervisor.turn_task_supervisor import (
    compile_turn_autonomous_request,
)


class TurnSchedulerJoint2BridgeCaseFakeTest(unittest.TestCase):
    """Verify a `joint2` selection bridges once then converges to the tip."""

    def test_joint2_idle_bridges_once_then_returns_to_free_growth(self) -> None:
        frame = build_turn_runtime_frame(idle_joint_ids={"joint2"})
        _, graph = compile_turn_autonomous_request(
            frame,
            target_heading_delta_deg=0.0,
            graph_id="turn_scheduler_joint2",
        )
        scheduler = SkillScheduler()
        scheduler.load_graph(graph)

        results = run_scheduler_until_finished(scheduler, frame)

        self.assertEqual([result.node.node_id for result in results], [
            "local_transfer_joint2_to_joint1",
            "front_cooperate_joint1_to_tip",
            "return_to_tip_free_growth",
            "tip_free_growth",
        ])
        self.assertEqual(results[0].diagnostics["selected_joint_id"], "joint2")
        self.assertEqual(results[0].diagnostics["selected_joint_index"], 2)
        self.assertEqual(results[0].diagnostics["current_active_pair"], "joint2->joint1")
        self.assertEqual(results[1].diagnostics["current_active_pair"], "joint1->tip")


if __name__ == "__main__":
    unittest.main()

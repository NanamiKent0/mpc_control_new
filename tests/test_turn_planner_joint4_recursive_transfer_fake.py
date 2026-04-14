"""Fake test for recursive transfer starting from idle `joint4`."""

from __future__ import annotations

import unittest

from tests.turn_planner_fake_support import build_estimate_bundle, node_signature

from mpc_control_new.control_core.supervisor import plan_turn_workflow


class TurnPlannerJoint4RecursiveTransferFakeTest(unittest.TestCase):
    """Verify recursive transfer expands fully from `joint4` to `joint1-tip`."""

    def test_joint4_idle_builds_full_recursive_transfer_chain(self) -> None:
        plan = plan_turn_workflow(build_estimate_bundle(idle_joint_ids={"joint4"}))

        self.assertEqual(plan.selected_joint_id, "joint4")
        self.assertEqual(plan.selected_joint_index, 4)
        self.assertFalse(plan.direct_front_cooperation)
        self.assertTrue(plan.requires_recursive_transfer)
        self.assertTrue(plan.invariant_ok)
        self.assertEqual(
            [node_signature(node) for node in plan.ordered_nodes],
            [
                (
                    "local_transfer",
                    "joint4",
                    "joint3",
                    "local_transfer_joint4_to_joint3",
                ),
                (
                    "local_transfer",
                    "joint3",
                    "joint2",
                    "local_transfer_joint3_to_joint2",
                ),
                (
                    "local_transfer",
                    "joint2",
                    "joint1",
                    "local_transfer_joint2_to_joint1",
                ),
                (
                    "front_cooperate",
                    "joint1",
                    "tip",
                    "front_cooperate_joint1_to_tip",
                ),
                (
                    "return_to_free_growth",
                    "tip",
                    None,
                    "return_to_tip_free_growth",
                ),
            ],
        )
        self.assertEqual(
            plan.diagnostics["transfer_chain"],
            ["joint4->joint3", "joint3->joint2", "joint2->joint1"],
        )


if __name__ == "__main__":
    unittest.main()

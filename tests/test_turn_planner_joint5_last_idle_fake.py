"""Fake test for recursive transfer when only `joint5` is idle."""

from __future__ import annotations

import unittest

from tests.turn_planner_fake_support import build_estimate_bundle, node_signature

from mpc_control_new.control_core.supervisor import plan_turn_workflow


class TurnPlannerJoint5LastIdleFakeTest(unittest.TestCase):
    """Verify the last supported idle joint still converges to the front."""

    def test_joint5_idle_builds_longest_supported_transfer_chain(self) -> None:
        plan = plan_turn_workflow(build_estimate_bundle(idle_joint_ids={"joint5"}))

        self.assertEqual(plan.selected_joint_id, "joint5")
        self.assertEqual(plan.selected_joint_index, 5)
        self.assertTrue(plan.requires_recursive_transfer)
        self.assertTrue(plan.invariant_ok)
        self.assertEqual(
            [node_signature(node) for node in plan.ordered_nodes],
            [
                (
                    "local_transfer",
                    "joint5",
                    "joint4",
                    "local_transfer_joint5_to_joint4",
                ),
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


if __name__ == "__main__":
    unittest.main()

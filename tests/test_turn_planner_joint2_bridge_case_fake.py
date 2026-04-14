"""Fake test for the `joint2 -> joint1 -> tip` bridge case."""

from __future__ import annotations

import unittest

from tests.turn_planner_fake_support import build_estimate_bundle, node_signature

from mpc_control_new.control_core.supervisor import (
    LocalTransferNode,
    plan_turn_workflow,
)


class TurnPlannerJoint2BridgeCaseFakeTest(unittest.TestCase):
    """Verify `joint2` first bridges to `joint1`, then converges to the tip."""

    def test_joint2_idle_builds_single_bridge_then_front_cooperate(self) -> None:
        plan = plan_turn_workflow(
            build_estimate_bundle(idle_joint_ids={"joint2", "joint5"})
        )

        self.assertEqual(plan.selected_joint_id, "joint2")
        self.assertEqual(plan.selected_joint_index, 2)
        self.assertFalse(plan.direct_front_cooperation)
        self.assertTrue(plan.requires_recursive_transfer)
        self.assertTrue(plan.invariant_ok)
        self.assertEqual(
            [node_signature(node) for node in plan.ordered_nodes],
            [
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
        self.assertIsInstance(plan.ordered_nodes[0], LocalTransferNode)


if __name__ == "__main__":
    unittest.main()

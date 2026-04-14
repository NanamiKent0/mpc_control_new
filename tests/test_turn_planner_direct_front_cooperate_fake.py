"""Fake test for direct front-cooperation turn planning via idle `joint1`."""

from __future__ import annotations

import unittest

from tests.turn_planner_fake_support import build_estimate_bundle, node_signature

from mpc_control_new.control_core.supervisor import (
    FrontCooperateNode,
    ReturnToFreeGrowthNode,
    plan_turn_workflow,
)


class TurnPlannerDirectFrontCooperateFakeTest(unittest.TestCase):
    """Verify direct front cooperation is planned when `joint1` is idle."""

    def test_joint1_idle_builds_direct_front_cooperation_plan(self) -> None:
        plan = plan_turn_workflow(
            build_estimate_bundle(idle_joint_ids={"joint1", "joint4"})
        )

        self.assertEqual(plan.selected_joint_id, "joint1")
        self.assertEqual(plan.selected_joint_index, 1)
        self.assertTrue(plan.direct_front_cooperation)
        self.assertFalse(plan.requires_recursive_transfer)
        self.assertTrue(plan.invariant_ok)
        self.assertEqual(
            [node_signature(node) for node in plan.ordered_nodes],
            [
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
        self.assertIsInstance(plan.ordered_nodes[0], FrontCooperateNode)
        self.assertIsInstance(plan.ordered_nodes[-1], ReturnToFreeGrowthNode)


if __name__ == "__main__":
    unittest.main()

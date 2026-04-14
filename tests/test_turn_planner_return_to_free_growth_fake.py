"""Fake test for the explicit return-to-free-growth tail node."""

from __future__ import annotations

import unittest

from tests.turn_planner_fake_support import build_estimate_bundle

from mpc_control_new.control_core.supervisor import ReturnToFreeGrowthNode, plan_turn_workflow


class TurnPlannerReturnToFreeGrowthFakeTest(unittest.TestCase):
    """Verify successful plans always terminate in tip free growth handoff."""

    def test_successful_plan_always_returns_to_tip_free_growth(self) -> None:
        direct_plan = plan_turn_workflow(build_estimate_bundle(idle_joint_ids={"joint1"}))
        recursive_plan = plan_turn_workflow(build_estimate_bundle(idle_joint_ids={"joint3"}))

        self.assertIsInstance(direct_plan.ordered_nodes[-1], ReturnToFreeGrowthNode)
        self.assertIsInstance(recursive_plan.ordered_nodes[-1], ReturnToFreeGrowthNode)
        self.assertTrue(direct_plan.diagnostics["returns_to_tip_free_growth"])
        self.assertTrue(recursive_plan.diagnostics["returns_to_tip_free_growth"])


if __name__ == "__main__":
    unittest.main()

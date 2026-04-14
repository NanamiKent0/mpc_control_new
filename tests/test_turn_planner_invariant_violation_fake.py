"""Fake test for the no-idle-joint turn-planner invariant violation path."""

from __future__ import annotations

import unittest

from tests.turn_planner_fake_support import build_estimate_bundle

from mpc_control_new.control_core.supervisor import (
    assert_turn_planner_invariant,
    plan_turn_workflow,
)


class TurnPlannerInvariantViolationFakeTest(unittest.TestCase):
    """Verify the planner returns a structured error result on invariant failure."""

    def test_no_idle_joint_returns_structured_error_plan(self) -> None:
        plan = plan_turn_workflow(build_estimate_bundle(idle_joint_ids=set()))

        self.assertIsNone(plan.selected_joint_id)
        self.assertIsNone(plan.selected_joint_index)
        self.assertFalse(plan.direct_front_cooperation)
        self.assertFalse(plan.requires_recursive_transfer)
        self.assertFalse(plan.invariant_ok)
        self.assertEqual(plan.ordered_nodes, [])
        self.assertEqual(plan.selection_reason, "no_idle_joint_found")
        self.assertEqual(plan.diagnostics["error_code"], "invariant_violated")
        self.assertEqual(plan.diagnostics["invariant_errors"], ["no_idle_joint_found"])
        with self.assertRaises(AssertionError):
            assert_turn_planner_invariant(plan)


if __name__ == "__main__":
    unittest.main()

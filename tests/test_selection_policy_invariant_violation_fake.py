"""Fake test for the no-idle-joint invariant violation path."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mpc_control_new.control_core.estimation import EstimateBundle, JointAvailability, TopologyEstimate
from mpc_control_new.control_core.supervisor import (
    assert_idle_joint_invariant,
    select_front_joint_candidate,
)

JOINT_IDS = ("joint1", "joint2", "joint3", "joint4", "joint5")


def _build_estimate_bundle() -> EstimateBundle:
    joint_availability = {
        joint_id: JointAvailability(
            joint_id=joint_id,
            is_idle=False,
            bend_near_zero=False,
            motion_quiet=True,
            estimate_valid=True,
            reason="bend_not_near_zero",
        )
        for joint_id in JOINT_IDS
    }
    return EstimateBundle(
        timestamp_ns=1,
        module_states={},
        relation_states={},
        joint_availability=joint_availability,
        topology_estimate=TopologyEstimate(
            ordered_modules=("tip", *JOINT_IDS),
            topology_valid=True,
        ),
    )


class SelectionPolicyInvariantViolationFakeTest(unittest.TestCase):
    """Verify the policy returns structured diagnostics on invariant failure."""

    def test_no_idle_joint_returns_structured_error_result(self) -> None:
        estimate_bundle = _build_estimate_bundle()

        with self.assertRaises(AssertionError):
            assert_idle_joint_invariant(estimate_bundle)

        result = select_front_joint_candidate(estimate_bundle)

        self.assertIsNone(result.selected_joint_id)
        self.assertIsNone(result.selected_joint_index)
        self.assertFalse(result.direct_front_cooperation)
        self.assertFalse(result.requires_recursive_transfer)
        self.assertEqual(result.selection_reason, "no_idle_joint_found")
        self.assertFalse(result.diagnostics["invariant_satisfied"])
        self.assertEqual(result.diagnostics["error_code"], "invariant_violated")


if __name__ == "__main__":
    unittest.main()

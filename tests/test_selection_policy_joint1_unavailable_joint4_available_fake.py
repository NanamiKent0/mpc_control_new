"""Fake test for fixed-order idle selection reaching `joint4`."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mpc_control_new.control_core.estimation import EstimateBundle, JointAvailability, TopologyEstimate
from mpc_control_new.control_core.supervisor import find_first_idle_joint_from_joint1_to_joint5

JOINT_IDS = ("joint1", "joint2", "joint3", "joint4", "joint5")


def _build_estimate_bundle(*, idle_joint_ids: set[str]) -> EstimateBundle:
    joint_availability = {
        joint_id: JointAvailability(
            joint_id=joint_id,
            is_idle=joint_id in idle_joint_ids,
            bend_near_zero=joint_id in idle_joint_ids,
            motion_quiet=True,
            estimate_valid=True,
            reason="idle" if joint_id in idle_joint_ids else "bend_not_near_zero",
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


class SelectionPolicyJoint4AvailableFakeTest(unittest.TestCase):
    """Verify the fixed traversal picks `joint4` before a later idle joint."""

    def test_joint4_is_selected_as_first_idle_joint(self) -> None:
        estimate_bundle = _build_estimate_bundle(idle_joint_ids={"joint4", "joint5"})

        result = find_first_idle_joint_from_joint1_to_joint5(estimate_bundle)

        self.assertEqual(result.selected_joint_id, "joint4")
        self.assertEqual(result.selected_joint_index, 4)
        self.assertFalse(result.direct_front_cooperation)
        self.assertTrue(result.requires_recursive_transfer)
        self.assertEqual(
            result.selection_reason,
            "first_idle_joint_selected_requires_recursive_transfer",
        )
        self.assertEqual(
            result.diagnostics["searched_joint_ids"],
            ["joint1", "joint2", "joint3", "joint4"],
        )


if __name__ == "__main__":
    unittest.main()

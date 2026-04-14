"""Fake test for direct front cooperation via idle `joint1`."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mpc_control_new.control_core.estimation import EstimateBundle, JointAvailability, TopologyEstimate
from mpc_control_new.control_core.supervisor import can_front_cooperate_now, select_front_joint_candidate

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


class SelectionPolicyJoint1AvailableFakeTest(unittest.TestCase):
    """Verify `joint1` is selected for direct front cooperation when idle."""

    def test_joint1_idle_selects_direct_front_cooperation(self) -> None:
        estimate_bundle = _build_estimate_bundle(idle_joint_ids={"joint1", "joint4"})

        self.assertTrue(can_front_cooperate_now(estimate_bundle))

        result = select_front_joint_candidate(estimate_bundle)

        self.assertEqual(result.selected_joint_id, "joint1")
        self.assertEqual(result.selected_joint_index, 1)
        self.assertTrue(result.direct_front_cooperation)
        self.assertFalse(result.requires_recursive_transfer)
        self.assertEqual(result.selection_reason, "joint1_idle_direct_front_cooperation")
        self.assertEqual(result.diagnostics["searched_joint_ids"], ["joint1"])


if __name__ == "__main__":
    unittest.main()

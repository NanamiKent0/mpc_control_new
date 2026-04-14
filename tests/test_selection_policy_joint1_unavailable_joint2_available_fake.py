"""Fake test for recursive transfer selection via idle `joint2`."""

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


class SelectionPolicyJoint2AvailableFakeTest(unittest.TestCase):
    """Verify the first idle joint after `joint1` is chosen for transfer."""

    def test_joint2_selected_when_joint1_is_not_idle(self) -> None:
        estimate_bundle = _build_estimate_bundle(idle_joint_ids={"joint2", "joint5"})

        self.assertFalse(can_front_cooperate_now(estimate_bundle))

        result = select_front_joint_candidate(estimate_bundle)

        self.assertEqual(result.selected_joint_id, "joint2")
        self.assertEqual(result.selected_joint_index, 2)
        self.assertFalse(result.direct_front_cooperation)
        self.assertTrue(result.requires_recursive_transfer)
        self.assertEqual(
            result.selection_reason,
            "first_idle_joint_selected_requires_recursive_transfer",
        )
        self.assertEqual(result.diagnostics["searched_joint_ids"], ["joint1", "joint2"])


if __name__ == "__main__":
    unittest.main()

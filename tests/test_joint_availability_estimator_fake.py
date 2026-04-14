"""Fake tests for joint availability estimation."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mpc_control_new.control_core.estimation import AvailabilityEstimator
from mpc_control_new.control_core.models.module_state import make_joint_module_state


class JointAvailabilityEstimatorFakeTest(unittest.TestCase):
    """Verify the simplified idle-joint decision rules."""

    def test_bend_near_zero_and_valid_estimate_is_idle(self) -> None:
        estimator = AvailabilityEstimator()
        module_states = {
            "joint1": make_joint_module_state(
                "joint1",
                bend_deg=2.0,
                metadata={"estimate_valid": True},
            )
        }

        availability = estimator.estimate(module_states)["joint1"]

        self.assertTrue(availability.is_idle)
        self.assertTrue(availability.bend_near_zero)
        self.assertTrue(availability.motion_quiet)
        self.assertEqual(availability.reason, "idle")

    def test_large_bend_is_not_idle(self) -> None:
        estimator = AvailabilityEstimator()
        module_states = {
            "joint1": make_joint_module_state(
                "joint1",
                bend_deg=12.0,
                metadata={"estimate_valid": True},
            )
        }

        availability = estimator.estimate(module_states)["joint1"]

        self.assertFalse(availability.is_idle)
        self.assertFalse(availability.bend_near_zero)
        self.assertEqual(availability.reason, "bend_not_near_zero")

    def test_invalid_estimate_is_not_idle(self) -> None:
        estimator = AvailabilityEstimator()
        module_states = {
            "joint1": make_joint_module_state(
                "joint1",
                bend_deg=0.0,
                metadata={"estimate_valid": False},
            )
        }

        availability = estimator.estimate(module_states)["joint1"]

        self.assertFalse(availability.is_idle)
        self.assertFalse(availability.estimate_valid)
        self.assertEqual(availability.reason, "estimate_invalid")

    def test_motion_signal_blocks_idle_even_when_bend_is_near_zero(self) -> None:
        estimator = AvailabilityEstimator()
        module_states = {
            "joint1": make_joint_module_state(
                "joint1",
                bend_deg=1.0,
                crawl_mm_s=3.0,
                metadata={"estimate_valid": True},
            )
        }

        availability = estimator.estimate(module_states)["joint1"]

        self.assertFalse(availability.is_idle)
        self.assertFalse(availability.motion_quiet)
        self.assertEqual(availability.reason, "motion_not_quiet")


if __name__ == "__main__":
    unittest.main()

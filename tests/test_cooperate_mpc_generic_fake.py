"""Fake tests for the generic cooperate MPC template."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mpc_control_new.control_core.mpc.cooperate_mpc import CooperateMPC


class CooperateMPCGenericFakeTest(unittest.TestCase):
    """Verify the cooperate MPC stays generic across pair families."""

    def test_same_solver_handles_local_transfer_cooperate(self) -> None:
        """Local transfer cooperation should use the same generic solver structure."""
        solver = CooperateMPC()
        result = solver.compute(
            feed_forward_mm_s=2.0,
            current_distance_mm=3.0,
            distance_ref_mm=0.0,
            current_orientation_error_deg=-8.0,
            orientation_ref_deg=0.0,
            current_bend_error_deg=4.0,
            bend_ref_deg=0.0,
            dt=0.1,
            limits={"feed_mm_s": 4.0, "rotate_deg_s": 6.0, "bend_deg_s": 5.0},
            config={"distance_gain": 0.1, "orientation_gain": 0.3, "bend_gain": 0.25},
        )
        self.assertGreater(result.feed_velocity_mm_s, 0.0)
        self.assertGreater(result.rotate_velocity_deg_s, 0.0)
        self.assertLess(result.bend_velocity_deg_s, 0.0)

    def test_same_solver_handles_front_cooperate_stabilization(self) -> None:
        """Front cooperation should reuse the same coupled feed/stabilize solver."""
        solver = CooperateMPC()
        result = solver.compute(
            feed_forward_mm_s=3.0,
            current_distance_mm=0.2,
            distance_ref_mm=0.0,
            current_orientation_error_deg=0.1,
            orientation_ref_deg=0.0,
            current_bend_error_deg=0.2,
            bend_ref_deg=0.0,
            dt=0.1,
            limits={"crawl_mm_s": 4.0, "rotate_deg_s": 6.0, "bend_deg_s": 5.0},
            config={
                "distance_gain": 0.1,
                "orientation_gain": 0.3,
                "bend_gain": 0.25,
                "orientation_deadband_deg": 0.5,
                "bend_deadband_deg": 0.5,
            },
        )
        self.assertGreater(result.feed_velocity_mm_s, 0.0)
        self.assertEqual(result.rotate_velocity_deg_s, 0.0)
        self.assertEqual(result.bend_velocity_deg_s, 0.0)
        self.assertIn("orientation stabilized", result.notes)


if __name__ == "__main__":
    unittest.main()

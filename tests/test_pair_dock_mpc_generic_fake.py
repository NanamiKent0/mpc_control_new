"""Fake tests for the generic pair dock MPC template."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mpc_control_new.control_core.mpc.pair_dock_mpc import PairDockMPC


class PairDockMPCGenericFakeTest(unittest.TestCase):
    """Verify the pair dock MPC stays generic across pair families."""

    def test_same_solver_handles_local_transfer_joint_pair(self) -> None:
        """One pair dock solver should handle a joint_i-joint_{i-1} docking problem."""
        solver = PairDockMPC()
        result = solver.compute(
            current_distance_mm=8.0,
            current_orientation_error_deg=-12.0,
            distance_ref_mm=0.0,
            orientation_ref_deg=0.0,
            dt=0.1,
            limits={"feed_mm_s": 4.0, "rotate_deg_s": 6.0},
            config={"distance_gain": 0.5, "orientation_gain": 0.5},
        )
        self.assertGreater(result.feed_velocity_mm_s, 0.0)
        self.assertGreater(result.rotate_velocity_deg_s, 0.0)
        self.assertEqual(result.crawl_velocity_mm_s, result.feed_velocity_mm_s)

    def test_same_solver_handles_front_joint_tip_pair_without_new_mode(self) -> None:
        """The same pair dock solver should also handle joint1-tip docking."""
        solver = PairDockMPC()
        result = solver.compute(
            current_distance_mm=5.0,
            current_orientation_error_deg=10.0,
            distance_ref_mm=1.0,
            orientation_ref_deg=2.0,
            dt=0.1,
            limits={"crawl_mm_s": 3.0, "rotate_deg_s": 5.0},
            config={"distance_gain": 0.4, "orientation_gain": 0.5},
        )
        self.assertGreater(result.feed_velocity_mm_s, 0.0)
        self.assertLess(result.rotate_velocity_deg_s, 0.0)
        self.assertTrue(result.active)


if __name__ == "__main__":
    unittest.main()

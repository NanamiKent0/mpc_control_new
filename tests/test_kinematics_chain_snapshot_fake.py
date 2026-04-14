"""Fake tests for chain-level geometry snapshots."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mpc_control_new.control_core.kinematics import compute_chain_snapshot
from mpc_control_new.control_core.models.module_state import make_joint_module_state, make_tip_module_state


class KinematicsChainSnapshotFakeTest(unittest.TestCase):
    """Verify chain snapshots expose stable points, frames, and pair geometry."""

    def test_chain_snapshot_is_visualizer_friendly_and_sorts_modules_canonically(self) -> None:
        snapshot = compute_chain_snapshot(
            {
                "joint2": make_joint_module_state("joint2", crawl_mm=340.0, bend_deg=0.0, rotate_deg=0.0),
                "tip": make_tip_module_state(growth_mm=400.0),
                "joint1": make_joint_module_state("joint1", crawl_mm=380.0, bend_deg=0.0, rotate_deg=0.0),
            }
        )

        self.assertEqual(snapshot.ordered_modules, ("tip", "joint1", "joint2"))
        self.assertEqual(snapshot.geometry_order, ("joint2", "joint1", "tip"))
        self.assertEqual(snapshot.frame_conventions["world_frame"], "base_frame")
        self.assertIn("base_origin", snapshot.named_points)
        self.assertIn("joint2_end_frame", snapshot.named_frames)
        self.assertIn("tip_end", snapshot.named_points)
        self.assertEqual(len(snapshot.module_paths["joint1"][0]), 4)
        self.assertEqual(len(snapshot.module_paths["tip"][0]), 2)
        self.assertAlmostEqual(snapshot.named_points["tip_end"][0], 400.0, places=6)
        self.assertAlmostEqual(snapshot.named_points["tip_start"][0], 250.0, places=6)

        joint1_tip = snapshot.get_pair_geometry("joint1", "tip")
        joint2_joint1 = snapshot.get_pair_geometry("joint2", "joint1")

        self.assertIsNotNone(joint1_tip)
        self.assertIsNotNone(joint2_joint1)
        if joint1_tip is None or joint2_joint1 is None:
            self.fail("expected adjacent pair geometry to be present in chain snapshot")
        self.assertAlmostEqual(joint1_tip.distance_mm, 20.0, places=6)
        self.assertAlmostEqual(joint2_joint1.distance_mm, 40.0, places=6)
        self.assertAlmostEqual(joint1_tip.orientation_error_deg, 0.0, places=6)
        self.assertAlmostEqual(joint2_joint1.orientation_error_deg, 0.0, places=6)


if __name__ == "__main__":
    unittest.main()

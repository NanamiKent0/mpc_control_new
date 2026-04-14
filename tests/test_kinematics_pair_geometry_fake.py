"""Fake tests for pair-wise geometry in the unified kinematics layer."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mpc_control_new.control_core.kinematics import (
    compute_module_pose,
    compute_pair_distance,
    compute_pair_geometry,
    compute_pair_orientation_error,
)
from mpc_control_new.control_core.models.module_state import make_joint_module_state, make_tip_module_state


def _translation_x(distance_mm: float) -> np.ndarray:
    transform = np.eye(4, dtype=float)
    transform[0, 3] = float(distance_mm)
    return transform


class KinematicsPairGeometryFakeTest(unittest.TestCase):
    """Verify pair distance and orientation are stable for detached module poses."""

    def test_pair_distance_and_twist_error_are_computed_from_interface_frames(self) -> None:
        active_pose = compute_module_pose(
            make_joint_module_state(
                "joint1",
                crawl_mm=0.0,
                bend_deg=0.0,
                rotate_deg=15.0,
            ),
            entry_offset_mm=0.0,
        )
        passive_pose = compute_module_pose(
            make_tip_module_state(growth_mm=150.0),
            anchor_transform=_translation_x(250.0),
            entry_offset_mm=0.0,
        )

        distance_mm = compute_pair_distance(active_pose, passive_pose)
        orientation_error_deg = compute_pair_orientation_error(active_pose, passive_pose)
        pair_geometry = compute_pair_geometry(active_pose, passive_pose)

        self.assertAlmostEqual(distance_mm, 40.0, places=6)
        self.assertAlmostEqual(orientation_error_deg, 15.0, places=6)
        self.assertEqual(pair_geometry.key(), "joint1->tip")
        self.assertAlmostEqual(pair_geometry.distance_mm, 40.0, places=6)
        self.assertAlmostEqual(pair_geometry.orientation_error_deg, 15.0, places=6)
        self.assertEqual(pair_geometry.diagnostics["orientation_rule"], "signed twist passive.start.y -> active.end.y about passive.start.x")


if __name__ == "__main__":
    unittest.main()

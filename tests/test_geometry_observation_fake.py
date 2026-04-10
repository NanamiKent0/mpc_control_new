"""Fake tests for the intermediate geometry observation layer."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mpc_control_new.controllers.adapters.legacy_extractors import (
    extract_tip_joint_geometry_observation,
    geometry_observation_to_relation_state,
)


def _estimate(
    *,
    distance_mm: float | None,
    orientation_deg: float | None,
    signed_orientation_deg: float | None,
    coupled: bool | None,
) -> SimpleNamespace:
    """Build a minimal fake estimate for geometry-observation tests."""
    return SimpleNamespace(
        geometry=SimpleNamespace(
            tip_joint1_distance_mm=distance_mm,
            tip_joint1_orientation_error_deg=orientation_deg,
            tip_joint1_orientation_error_signed_deg=signed_orientation_deg,
            tip_joint1_coupled=coupled,
        ),
        control=SimpleNamespace(
            c1=0.0,
            psi1=0.0,
            theta1=0.0,
            d_t1=False if coupled is None else coupled,
        ),
    )


class GeometryObservationFakeTest(unittest.TestCase):
    """Verify the geometry observation middle layer introduced in Phase 5."""

    def test_legacy_estimate_can_become_observation_then_relation_state(self) -> None:
        """Tip-joint geometry should flow through the observation layer before relation state."""
        observation = extract_tip_joint_geometry_observation(
            _estimate(distance_mm=14.0, orientation_deg=9.0, signed_orientation_deg=-9.0, coupled=False)
        )
        relation_state = geometry_observation_to_relation_state(observation)

        self.assertEqual(observation.pair, ("joint1", "tip"))
        self.assertEqual(observation.orientation_error_deg, -9.0)
        self.assertEqual(observation.source_fields["distance_mm"], "geometry.tip_joint1_distance_mm")
        self.assertEqual(relation_state.diagnostics["orientation_signal_source"], "tip_joint1_orientation_error_signed_deg")

    def test_missing_fields_gracefully_degrade_and_preserve_notes(self) -> None:
        """Missing geometry should degrade without exploding and should retain notes."""
        observation = extract_tip_joint_geometry_observation(
            _estimate(distance_mm=None, orientation_deg=None, signed_orientation_deg=None, coupled=None)
        )
        relation_state = geometry_observation_to_relation_state(observation)

        self.assertFalse(observation.observation_valid)
        self.assertIsNone(observation.distance_mm)
        self.assertIn("missing_geometry_fields:tip_joint1_distance_mm", observation.notes)
        self.assertIn("orientation_signal_unavailable", observation.notes)
        self.assertEqual(relation_state.diagnostics["missing_fields"], "tip_joint1_distance_mm")
        self.assertIn("missing_geometry_fields:tip_joint1_distance_mm", relation_state.diagnostics["observation_notes"])


if __name__ == "__main__":
    unittest.main()

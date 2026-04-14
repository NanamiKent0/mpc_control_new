"""Fake tests for the explicit geometry observation abstraction boundary."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mpc_control_new.control_core.controllers.adapters.legacy_extractors import (
    build_geometry_observation_from_legacy_pair,
    relation_state_from_observation,
)


def _estimate(
    *,
    distance_mm: float | None,
    orientation_deg: float | None,
    signed_orientation_deg: float | None,
    coupled: bool | None,
) -> SimpleNamespace:
    """Build a minimal fake estimate for observation-abstraction tests."""
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


class ObservationAbstractionFakeTest(unittest.TestCase):
    """Verify legacy geometry now lands on an explicit observation boundary."""

    def test_legacy_pair_builds_observation_with_source_schema(self) -> None:
        """A legacy pair estimate should produce a fully described geometry observation."""
        observation = build_geometry_observation_from_legacy_pair(
            _estimate(distance_mm=14.0, orientation_deg=9.0, signed_orientation_deg=-9.0, coupled=False),
            active_module="joint1",
            passive_module="tip",
            relation_type="tip_joint",
        )
        relation_state = relation_state_from_observation(observation)

        self.assertEqual(observation.observation_kind, "tip_joint")
        self.assertEqual(observation.source_schema, "legacy.geometry.v1")
        self.assertEqual(observation.source_fields["distance_mm"], "geometry.tip_joint1_distance_mm")
        self.assertEqual(observation.metrics["distance_mm"], 14.0)
        self.assertEqual(relation_state.diagnostics["geometry_source_schema"], "legacy.geometry.v1")
        self.assertEqual(relation_state.diagnostics["geometry_observation_kind"], "tip_joint")
        self.assertEqual(relation_state.diagnostics["orientation_signal_source"], "tip_joint1_orientation_error_signed_deg")

    def test_missing_fields_gracefully_degrade_and_preserve_notes(self) -> None:
        """Missing legacy fields should degrade without losing observation provenance."""
        observation = build_geometry_observation_from_legacy_pair(
            _estimate(distance_mm=None, orientation_deg=None, signed_orientation_deg=None, coupled=None),
            active_module="joint1",
            passive_module="tip",
            relation_type="tip_joint",
        )
        relation_state = relation_state_from_observation(observation)

        self.assertFalse(observation.observation_valid)
        self.assertIn("missing_geometry_fields:tip_joint1_distance_mm", observation.notes)
        self.assertIn("orientation_signal_unavailable", observation.notes)
        self.assertEqual(observation.metrics["missing_field_count"], 1.0)
        self.assertEqual(relation_state.diagnostics["missing_fields"], "tip_joint1_distance_mm")
        self.assertEqual(relation_state.diagnostics["source_field_distance_mm"], "geometry.tip_joint1_distance_mm")
        self.assertIn("missing_geometry_fields:tip_joint1_distance_mm", relation_state.diagnostics["observation_notes"])


if __name__ == "__main__":
    unittest.main()

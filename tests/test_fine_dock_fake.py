"""Fake tests for the Phase-1 fine docking skill."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mpc_control_new.control_core.models.skill_types import SkillSpec
from mpc_control_new.control_core.skills.fine_dock import FineDockSkill
from mpc_control_new.control_core.controllers.adapters.legacy_extractors import (
    extract_tip_joint1_relation_state,
)


def _build_estimate(
    *,
    distance_mm: float | None,
    orientation_error_deg: float | None,
    orientation_error_signed_deg: float | None,
    coupled: bool | None,
) -> SimpleNamespace:
    """Build a minimal fake legacy-like estimate for tip-joint1 fine docking."""
    geometry = SimpleNamespace(
        tip_joint1_distance_mm=distance_mm,
        tip_joint1_orientation_error_deg=orientation_error_deg,
        tip_joint1_orientation_error_signed_deg=orientation_error_signed_deg,
        tip_joint1_coupled=coupled,
    )
    control = SimpleNamespace(
        c1=0.0,
        psi1=0.0,
        theta1=0.0,
        d_t1=False if coupled is None else coupled,
    )
    return SimpleNamespace(geometry=geometry, control=control)


def _build_spec() -> SkillSpec:
    """Build the default fine docking spec used by the fake tests."""
    return SkillSpec(
        skill_key="fine_dock",
        active_module="joint1",
        passive_module="tip",
        relation_type="tip_joint",
        distance_done_mm=3.0,
        orientation_done_deg=5.0,
        solver_dt=0.1,
        limits={"crawl_mm_s": 4.0, "rotate_deg_s": 6.0},
        config={
            "distance_gain": 0.2,
            "orientation_gain": 0.5,
            "distance_deadband_mm": 0.5,
            "orientation_deadband_deg": 0.5,
            "rotate_control_sign": 1.0,
        },
    )


class FineDockFakeTest(unittest.TestCase):
    """Verify the Phase-1 fine docking semantics."""

    def test_positive_crawl_and_rotate_references_when_far_and_misaligned(self) -> None:
        """Far misalignment should request active crawl and rotate closure."""
        skill = FineDockSkill()
        relation_state = extract_tip_joint1_relation_state(
            _build_estimate(
                distance_mm=18.0,
                orientation_error_deg=12.0,
                orientation_error_signed_deg=-12.0,
                coupled=False,
            ),
        )
        result = skill.execute(relation_state, _build_spec())
        crawl_reference = next(ref for ref in result.primitive_references if ref.axis == "crawl")
        rotate_reference = next(ref for ref in result.primitive_references if ref.axis == "rotate")
        self.assertEqual(result.status, "active")
        self.assertGreater(crawl_reference.reference_value, 0.0)
        self.assertGreater(rotate_reference.reference_value, 0.0)

    def test_zero_hold_when_already_docked(self) -> None:
        """Already-coupled relations should emit explicit zero holds."""
        skill = FineDockSkill()
        relation_state = extract_tip_joint1_relation_state(
            _build_estimate(
                distance_mm=1.0,
                orientation_error_deg=1.0,
                orientation_error_signed_deg=1.0,
                coupled=True,
            ),
        )
        result = skill.execute(relation_state, _build_spec())
        primary_references = [ref for ref in result.primitive_references if ref.primary]
        self.assertEqual(result.status, "done")
        self.assertTrue(result.completion.done)
        self.assertTrue(all(ref.reference_value == 0.0 for ref in primary_references))
        self.assertTrue(all(ref.semantic == "zero_hold" for ref in primary_references))

    def test_completion_inside_distance_and_orientation_tolerances(self) -> None:
        """Inside the docking tolerances should count as complete."""
        skill = FineDockSkill()
        relation_state = extract_tip_joint1_relation_state(
            _build_estimate(
                distance_mm=2.0,
                orientation_error_deg=3.0,
                orientation_error_signed_deg=3.0,
                coupled=False,
            ),
        )
        result = skill.execute(relation_state, _build_spec())
        self.assertEqual(result.status, "done")
        self.assertTrue(result.completion.done)
        self.assertEqual(result.completion.completion_reason, "within_docking_tolerances")


if __name__ == "__main__":
    unittest.main()

"""Fake tests for the Phase-1 coarse approach skill."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mpc_control_new.control_core.models.skill_types import SkillSpec
from mpc_control_new.control_core.skills.coarse_approach import CoarseApproachSkill
from mpc_control_new.control_core.controllers.adapters.legacy_extractors import (
    extract_tip_joint1_relation_state,
)


def _build_estimate(
    *,
    distance_mm: float | None,
    coupled: bool | None,
) -> SimpleNamespace:
    """Build a minimal fake legacy-like estimate for tip-joint1."""
    geometry = SimpleNamespace(
        tip_joint1_distance_mm=distance_mm,
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
    """Build the default coarse approach spec used by the fake tests."""
    return SkillSpec(
        skill_key="coarse_approach",
        active_module="joint1",
        passive_module="tip",
        relation_type="tip_joint",
        distance_done_mm=180.0,
        solver_dt=0.1,
        limits={"crawl_mm_s": 5.0},
        config={"gain": 0.2, "deadband_mm": 0.5},
    )


class CoarseApproachFakeTest(unittest.TestCase):
    """Verify the Phase-1 coarse approach semantics."""

    def test_invalid_entry_block_when_coupled(self) -> None:
        """Coupled entry should block the coarse approach skill."""
        skill = CoarseApproachSkill()
        relation_state = extract_tip_joint1_relation_state(
            _build_estimate(distance_mm=40.0, coupled=True),
        )
        result = skill.execute(relation_state, _build_spec())
        self.assertEqual(result.status, "blocked")
        self.assertFalse(result.preconditions.passed)
        self.assertEqual(result.preconditions.blocking_reason, "tip_joint1_still_coupled")
        self.assertEqual(result.primitive_references, [])

    def test_already_near_completion(self) -> None:
        """Already-near decoupled entry should complete with zero crawl."""
        skill = CoarseApproachSkill()
        relation_state = extract_tip_joint1_relation_state(
            _build_estimate(distance_mm=150.0, coupled=False),
        )
        result = skill.execute(relation_state, _build_spec())
        crawl_reference = next(ref for ref in result.primitive_references if ref.axis == "crawl")
        self.assertEqual(result.status, "done")
        self.assertTrue(result.completion.done)
        self.assertEqual(crawl_reference.semantic, "zero_hold")
        self.assertEqual(crawl_reference.reference_value, 0.0)

    def test_positive_crawl_reference_when_far_and_decoupled(self) -> None:
        """Far decoupled entry should request positive crawl closure."""
        skill = CoarseApproachSkill()
        relation_state = extract_tip_joint1_relation_state(
            _build_estimate(distance_mm=260.0, coupled=False),
        )
        result = skill.execute(relation_state, _build_spec())
        crawl_reference = next(ref for ref in result.primitive_references if ref.axis == "crawl")
        self.assertEqual(result.status, "active")
        self.assertTrue(result.preconditions.passed)
        self.assertFalse(result.completion.done)
        self.assertGreater(crawl_reference.reference_value, 0.0)


if __name__ == "__main__":
    unittest.main()

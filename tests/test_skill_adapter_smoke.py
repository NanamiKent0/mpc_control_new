"""Smoke tests for the Phase-1 legacy-semantic skill adapter."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mpc_control_new.control_core.models.skill_types import SkillSpec
from mpc_control_new.controllers.adapters.skill_controller_adapter import SkillControllerAdapter


def _build_estimate(
    *,
    tip_joint1_distance_mm: float | None = None,
    tip_joint1_orientation_error_deg: float | None = None,
    tip_joint1_orientation_error_signed_deg: float | None = None,
    tip_joint1_coupled: bool | None = None,
    joint1_joint2_distance_mm: float | None = None,
    joint1_joint2_coupled: bool | None = None,
) -> SimpleNamespace:
    """Build a minimal fake legacy-like estimate for adapter smoke tests."""
    geometry = SimpleNamespace(
        tip_joint1_distance_mm=tip_joint1_distance_mm,
        tip_joint1_orientation_error_deg=tip_joint1_orientation_error_deg,
        tip_joint1_orientation_error_signed_deg=tip_joint1_orientation_error_signed_deg,
        tip_joint1_coupled=tip_joint1_coupled,
        joint1_joint2_distance_mm=joint1_joint2_distance_mm,
        joint1_joint2_coupled=joint1_joint2_coupled,
    )
    control = SimpleNamespace(
        c1=0.0,
        psi1=0.0,
        theta1=0.0,
        c2=0.0,
        psi2=0.0,
        theta2=0.0,
        d_t1=False if tip_joint1_coupled is None else tip_joint1_coupled,
        d_12=False if joint1_joint2_coupled is None else joint1_joint2_coupled,
    )
    return SimpleNamespace(geometry=geometry, control=control)


class SkillAdapterSmokeTest(unittest.TestCase):
    """Verify adapter dispatch and result structure for Phase-1 skills."""

    def test_adapter_dispatches_coarse_approach(self) -> None:
        """Adapter should dispatch coarse approach through the joint2-joint1 extractor."""
        adapter = SkillControllerAdapter()
        spec = SkillSpec(
            skill_key="coarse_approach",
            active_module="joint2",
            passive_module="joint1",
            relation_type="joint_joint",
            distance_done_mm=220.0,
            limits={"crawl_mm_s": 4.0},
            config={"gain": 0.2, "deadband_mm": 0.5},
        )
        result = adapter.execute(
            spec,
            _build_estimate(joint1_joint2_distance_mm=260.0, joint1_joint2_coupled=False),
        )
        self.assertEqual(result.status, "active")
        self.assertEqual(result.relation_state.active_module, "joint2")
        self.assertIn("current_distance_mm", result.diagnostics)
        self.assertFalse(result.completion.done)

    def test_adapter_dispatches_fine_dock(self) -> None:
        """Adapter should dispatch fine docking through the tip-joint1 extractor."""
        adapter = SkillControllerAdapter()
        spec = SkillSpec(
            skill_key="fine_dock",
            active_module="joint1",
            passive_module="tip",
            relation_type="tip_joint",
            distance_done_mm=3.0,
            orientation_done_deg=5.0,
            limits={"crawl_mm_s": 4.0, "rotate_deg_s": 6.0},
            config={
                "distance_gain": 0.2,
                "orientation_gain": 0.5,
                "distance_deadband_mm": 0.5,
                "orientation_deadband_deg": 0.5,
            },
        )
        result = adapter.execute(
            spec,
            _build_estimate(
                tip_joint1_distance_mm=14.0,
                tip_joint1_orientation_error_deg=9.0,
                tip_joint1_orientation_error_signed_deg=-9.0,
                tip_joint1_coupled=False,
            ),
        )
        self.assertEqual(result.status, "active")
        self.assertEqual(result.diagnostics["orientation_signal_source"], "tip_joint1_orientation_error_signed_deg")
        self.assertGreaterEqual(len(result.primitive_references), 2)

    def test_adapter_result_contains_completion_status_and_diagnostics(self) -> None:
        """Adapter results should expose completion state and merged diagnostics."""
        adapter = SkillControllerAdapter()
        spec = SkillSpec(
            skill_key="fine_dock",
            active_module="joint1",
            passive_module="tip",
            relation_type="tip_joint",
            distance_done_mm=3.0,
            orientation_done_deg=5.0,
        )
        result = adapter.execute(
            spec,
            _build_estimate(
                tip_joint1_distance_mm=2.0,
                tip_joint1_orientation_error_deg=4.0,
                tip_joint1_orientation_error_signed_deg=4.0,
                tip_joint1_coupled=False,
            ),
        )
        self.assertEqual(result.status, "done")
        self.assertTrue(result.completion.done)
        self.assertIn("current_distance_mm", result.diagnostics)
        self.assertIn("orientation_signal_source", result.diagnostics)


if __name__ == "__main__":
    unittest.main()

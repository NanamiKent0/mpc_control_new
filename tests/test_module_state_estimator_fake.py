"""Fake tests for module-state estimation."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mpc_control_new.control_core.estimation import ModuleStateEstimator, build_estimate_bundle
from mpc_control_new.runtime_integration.observation_types import (
    ModuleObservation,
    PairObservation,
    RuntimeObservationFrame,
)


def _frame() -> RuntimeObservationFrame:
    return RuntimeObservationFrame(
        timestamp_ns=100,
        module_observations={
            "tip": ModuleObservation(
                module_id="tip",
                module_type="tip",
                dofs={"growth_mm": 155.0},
                attach_state={"joint1": False},
                source_name="fake_runtime",
            ),
            "joint1": ModuleObservation(
                module_id="joint1",
                module_type="joint",
                dofs={"crawl_mm": 220.0, "bend_deg": 1.5, "rotate_deg": -2.0},
                velocities={"crawl_mm_s": 0.0, "rotate_deg_s": 0.0},
                attach_state={"tip": False},
                source_name="fake_runtime",
            ),
        },
        pair_observations={
            "joint1->tip": PairObservation(
                active_module="joint1",
                passive_module="tip",
                relation_type="tip_joint",
                distance_mm=12.0,
                orientation_error_deg=-3.0,
                coupled=False,
                observation_valid=True,
                source_name="fake_runtime",
            )
        },
        topology_hint={"ordered_modules": ["tip", "joint1", "joint2"]},
        metadata={"source_name": "fake_runtime"},
    )


class ModuleStateEstimatorFakeTest(unittest.TestCase):
    """Verify module-state estimation annotates validity and coupling semantics."""

    def test_estimator_preserves_state_and_marks_placeholder_invalid(self) -> None:
        estimator = ModuleStateEstimator()

        module_states = estimator.estimate(_frame(), ordered_modules=("tip", "joint1", "joint2"))

        self.assertEqual(module_states["joint1"].dofs["bend_deg"], 1.5)
        self.assertEqual(module_states["joint1"].velocities["crawl_mm_s"], 0.0)
        self.assertFalse(module_states["joint1"].metadata["coupled_to_tip"])
        self.assertTrue(module_states["joint1"].metadata["estimate_valid"])
        self.assertFalse(module_states["joint2"].metadata["estimate_valid"])
        self.assertIn("runtime_module_observation_missing", module_states["joint2"].notes)

    def test_bundle_builds_module_state_with_geometry_context(self) -> None:
        bundle = build_estimate_bundle(_frame(), ordered_modules=("tip", "joint1", "joint2"))

        self.assertEqual(bundle.timestamp_ns, 100)
        self.assertIn("joint1", bundle.module_states)
        self.assertTrue(bundle.module_states["joint1"].metadata["estimate_pose_available"])
        self.assertIsNotNone(bundle.chain_snapshot)


if __name__ == "__main__":
    unittest.main()

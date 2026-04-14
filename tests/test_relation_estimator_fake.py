"""Fake tests for relation-state estimation."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mpc_control_new.control_core.estimation import ModuleStateEstimator, RelationEstimator
from mpc_control_new.control_core.kinematics import compute_chain_snapshot
from mpc_control_new.runtime_integration.observation_types import (
    ModuleObservation,
    PairObservation,
    RuntimeObservationFrame,
)


def _frame() -> RuntimeObservationFrame:
    return RuntimeObservationFrame(
        timestamp_ns=77,
        module_observations={
            "tip": ModuleObservation(
                module_id="tip",
                module_type="tip",
                dofs={"growth_mm": 160.0},
            ),
            "joint1": ModuleObservation(
                module_id="joint1",
                module_type="joint",
                dofs={"crawl_mm": 240.0, "bend_deg": 0.0, "rotate_deg": 1.0},
            ),
            "joint2": ModuleObservation(
                module_id="joint2",
                module_type="joint",
                dofs={"crawl_mm": 310.0, "bend_deg": 2.0, "rotate_deg": -1.0},
            ),
        },
        pair_observations={
            "joint1->tip": PairObservation(
                active_module="joint1",
                passive_module="tip",
                relation_type="tip_joint",
                distance_mm=8.0,
                orientation_error_deg=-4.0,
                coupled=False,
                observation_valid=True,
            )
        },
        topology_hint={"ordered_modules": ["tip", "joint1", "joint2"]},
    )


class RelationEstimatorFakeTest(unittest.TestCase):
    """Verify relation estimation uses runtime data first and geometry as fallback."""

    def test_relation_estimation_fuses_runtime_and_kinematics(self) -> None:
        frame = _frame()
        module_states = ModuleStateEstimator().estimate(frame, ordered_modules=("tip", "joint1", "joint2"))
        snapshot = compute_chain_snapshot(module_states, ordered_modules=("tip", "joint1", "joint2"))

        relation_states = RelationEstimator().estimate(
            frame,
            module_states=module_states,
            chain_snapshot=snapshot,
            ordered_modules=("tip", "joint1", "joint2"),
        )

        tip_pair = relation_states["joint1->tip"]
        joint_pair = relation_states["joint2->joint1"]

        self.assertTrue(tip_pair.observation_valid)
        self.assertEqual(tip_pair.distance_mm, 8.0)
        self.assertEqual(tip_pair.diagnostics["distance_source"], "runtime_pair_observation")
        self.assertTrue(joint_pair.observation_valid)
        self.assertEqual(joint_pair.diagnostics["distance_source"], "kinematics_pair_geometry")
        self.assertFalse(joint_pair.diagnostics["runtime_observation_valid"])


if __name__ == "__main__":
    unittest.main()

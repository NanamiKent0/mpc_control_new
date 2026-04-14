"""End-to-end smoke tests for the self-contained sim runtime session path."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mpc_control_new.runtime_integration.runtime_session import build_sim_runtime_session
from mpc_control_new.runtime_integration.sim_backend import SimState


class RuntimeSessionSimEndToEndSmokeTest(unittest.TestCase):
    """Verify the sim runtime path stays end-to-end inside the new package."""

    def test_sim_runtime_session_runs_provider_scheduler_dispatcher_chain(self) -> None:
        """The self-contained sim path should drive provider -> scheduler -> dispatcher end to end."""
        session = build_sim_runtime_session(
            initial_state=SimState(
                tip_joint1_distance_mm=20.0,
                tip_joint1_orientation_error_deg=-10.0,
                tip_joint1_coupled=False,
                joint1_joint2_distance_mm=0.0,
                joint1_joint2_orientation_error_deg=0.0,
                joint1_joint2_coupled=True,
            ),
            dt=10.0,
        )

        first = session.step()
        second = session.step()

        self.assertTrue(first.accepted)
        self.assertTrue(second.accepted)
        self.assertEqual(first.diagnostics["input_source"], "runtime_frame:sim")
        self.assertEqual(first.diagnostics["dispatch_target"], "sim")
        self.assertEqual(first.diagnostics["provider_kind"], "sim")
        self.assertEqual(first.dispatch_result.diagnostics["dispatch_target"], "sim")
        self.assertIsNotNone(first.dispatch_envelope)
        self.assertEqual(first.dispatch_envelope.input_source, "runtime_frame:sim")
        self.assertFalse(first.diagnostics["legacy_path_used"])
        self.assertLess(second.frame.pair_observations["joint1->tip"].distance_mm, first.frame.pair_observations["joint1->tip"].distance_mm)
        self.assertEqual(second.scheduler_step_result.scheduler_state.current_node_id, "fine_dock")


if __name__ == "__main__":
    unittest.main()

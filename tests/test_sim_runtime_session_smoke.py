"""Smoke tests for the sim-backed runtime session path."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mpc_control_new.control_core.orchestration.graph_factories import build_runtime_demo_pair_graph
from mpc_control_new.runtime_integration.runtime_session import RuntimeSession
from mpc_control_new.runtime_integration.sim_backend import SimState
from mpc_control_new.runtime_integration.sim_command_dispatcher import SimCommandDispatcher, SimRuntimeBackend
from mpc_control_new.runtime_integration.sim_observation_provider import SimObservationProvider


class SimRuntimeSessionSmokeTest(unittest.TestCase):
    """Verify the runtime-facing session can drive the lightweight sim adapters."""

    def test_sim_runtime_session_connects_provider_scheduler_and_dispatcher(self) -> None:
        """A shared sim backend should let the runtime session progress into fine docking."""
        backend = SimRuntimeBackend(
            SimState(
                g=0.0,
                c1=0.0,
                psi1=-10.0,
                tip_joint1_coupled=False,
                joint1_joint2_coupled=True,
                tip_joint1_distance_mm=20.0,
                tip_joint1_orientation_error_deg=-10.0,
                joint1_joint2_distance_mm=0.0,
                joint1_joint2_orientation_error_deg=0.0,
            ),
            dt=10.0,
        )
        session = RuntimeSession(
            observation_provider=SimObservationProvider(backend=backend),
            command_dispatcher=SimCommandDispatcher(backend=backend),
            graph_spec=build_runtime_demo_pair_graph(
                graph_id="sim_runtime_demo",
                active_module="joint1",
                passive_module="tip",
                relation_type="tip_joint",
                coarse_distance_threshold_mm=12.0,
                dock_distance_done_mm=2.0,
                dock_orientation_done_deg=5.0,
                include_finalize_node=False,
            ),
        )

        first = session.step()
        second = session.step()
        third = session.step()
        fourth = session.step()

        self.assertTrue(first.accepted)
        self.assertEqual(first.scheduler_step_result.node.node_id, "coarse_approach")
        self.assertEqual(first.diagnostics["input_source"], "runtime_frame:sim")
        self.assertEqual(first.diagnostics["dispatch_target"], "sim")
        self.assertTrue(second.scheduler_step_result.transitioned)
        self.assertEqual(second.scheduler_step_result.scheduler_state.current_node_id, "fine_dock")
        self.assertEqual(third.scheduler_step_result.node.node_id, "fine_dock")
        self.assertTrue(third.accepted)
        self.assertFalse(third.scheduler_step_result.transitioned)
        self.assertIn("topology_blocked", third.scheduler_step_result.transition_reason)
        self.assertFalse(fourth.scheduler_step_result.transitioned)
        self.assertEqual(fourth.scheduler_step_result.scheduler_state.current_node_id, "fine_dock")


if __name__ == "__main__":
    unittest.main()

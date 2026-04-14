"""Smoke tests for scheduler dispatch-envelope generation."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mpc_control_new.control_core.models.execution_context import ExecutionContext
from mpc_control_new.control_core.orchestration.graph_factories import (
    build_retryable_pair_approach_then_dock_graph,
)
from mpc_control_new.control_core.orchestration.skill_scheduler import SkillScheduler
from mpc_control_new.control_core.topology.chain_topology import ChainTopology
from mpc_control_new.control_core.controllers.adapters.skill_controller_adapter import (
    SkillControllerAdapter,
)


def _estimate() -> SimpleNamespace:
    """Build a fake estimate for scheduler bridge smoke tests."""
    geometry = SimpleNamespace(
        tip_joint1_distance_mm=30.0,
        tip_joint1_orientation_error_deg=10.0,
        tip_joint1_orientation_error_signed_deg=-10.0,
        tip_joint1_coupled=False,
    )
    control = SimpleNamespace(
        c1=0.0,
        psi1=0.0,
        theta1=0.0,
        d_t1=False,
    )
    return SimpleNamespace(geometry=geometry, control=control)


class SchedulerBridgeSmokeTest(unittest.TestCase):
    """Verify scheduler output can be bridged into a runtime dispatch envelope."""

    def test_scheduler_step_can_emit_dispatch_envelope(self) -> None:
        """One scheduler step should preserve topology, context, command, and decision data."""
        adapter = SkillControllerAdapter()
        scheduler = SkillScheduler(adapter=adapter)
        scheduler.load_graph(
            build_retryable_pair_approach_then_dock_graph(
                graph_id="scheduler_bridge_graph",
                active_module="joint1",
                passive_module="tip",
                relation_type="tip_joint",
                coarse_distance_threshold_mm=12.0,
                dock_distance_done_mm=2.0,
                dock_orientation_done_deg=5.0,
                coarse_max_attempts=2,
            )
        )
        topology = ChainTopology(ordered_modules=["tip", "joint1", "joint2"])
        result = scheduler.step(
            _estimate(),
            context=ExecutionContext(
                topology=topology,
                metadata={"caller": "scheduler_bridge_smoke"},
            ),
        )
        envelope = scheduler.to_dispatch_envelope(result)

        self.assertFalse(result.transitioned)
        self.assertTrue(result.retry_scheduled)
        self.assertEqual(result.diagnostics["resolved_transition_policy_key"], "default")
        self.assertEqual(result.diagnostics["geometry_observation_kind"], "tip_joint")

        self.assertIsNotNone(envelope.scheduled_command)
        self.assertEqual(envelope.scheduled_command.graph_id, "scheduler_bridge_graph")
        self.assertEqual(envelope.scheduled_command.node_id, "coarse_approach")
        self.assertEqual(envelope.scheduled_command.skill_key, "coarse_approach")
        self.assertIn("joint_crawl", envelope.scheduled_command.selected_primitives)
        self.assertTrue(envelope.transition_decision.retry_scheduled)
        self.assertEqual(envelope.context_metadata["caller"], "scheduler_bridge_smoke")
        self.assertIn("topology_snapshot", envelope.context_metadata)
        self.assertEqual(envelope.scheduled_command.diagnostics["geometry_source_schema"], "legacy.geometry.v1")


if __name__ == "__main__":
    unittest.main()

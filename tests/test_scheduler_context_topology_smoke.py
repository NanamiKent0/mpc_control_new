"""Smoke tests for scheduler-level context and topology propagation."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mpc_control_new.control_core.models.execution_context import ExecutionContext
from mpc_control_new.control_core.models.skill_types import SkillSpec
from mpc_control_new.control_core.orchestration.graph_factories import build_pair_approach_then_dock_graph
from mpc_control_new.control_core.orchestration.skill_scheduler import SkillScheduler
from mpc_control_new.control_core.skills.coarse_approach import CoarseApproachSkill
from mpc_control_new.control_core.topology.chain_topology import ChainTopology
from mpc_control_new.controllers.adapters.skill_controller_adapter import SkillControllerAdapter


def _estimate() -> SimpleNamespace:
    """Build a fake estimate for scheduler context/topology tests."""
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


class _RecordingCoarseApproachSkill(CoarseApproachSkill):
    """Coarse approach test double that records the final execution context."""

    def __init__(self, *args: object, **kwargs: object) -> None:
        super().__init__(*args, **kwargs)
        self.last_context: ExecutionContext | None = None

    def execute(
        self,
        source: object,
        spec: SkillSpec,
        *,
        module_states: dict[str, object] | None = None,
        context: ExecutionContext | None = None,
    ):
        self.last_context = context
        return super().execute(source, spec, module_states=module_states, context=context)


class SchedulerContextTopologySmokeTest(unittest.TestCase):
    """Verify scheduler-owned context metadata and topology override behavior."""

    def test_scheduler_passes_external_topology_and_context_metadata(self) -> None:
        """External topology should be used, preserved, and visible in diagnostics."""
        recording_skill = _RecordingCoarseApproachSkill(
            topology=ChainTopology(ordered_modules=["tip", "joint1"])
        )
        adapter = SkillControllerAdapter(coarse_skill=recording_skill)
        scheduler = SkillScheduler(adapter=adapter)
        scheduler.load_graph(
            build_pair_approach_then_dock_graph(
                graph_id="scheduler_context_topology",
                active_module="joint1",
                passive_module="tip",
                relation_type="tip_joint",
                coarse_distance_threshold_mm=12.0,
                dock_distance_done_mm=2.0,
                dock_orientation_done_deg=5.0,
            )
        )
        external_topology = ChainTopology(ordered_modules=["tip", "joint1", "joint2"])
        external_topology.set_blocked_edge("joint1", "tip", True)
        result = scheduler.step(
            _estimate(),
            context=ExecutionContext(
                topology=external_topology,
                metadata={"caller": "scheduler_context_smoke"},
            ),
        )

        self.assertIs(recording_skill.last_context.topology, external_topology)
        self.assertEqual(recording_skill.last_context.metadata["graph_id"], "scheduler_context_topology")
        self.assertEqual(recording_skill.last_context.metadata["node_id"], "coarse_approach")
        self.assertEqual(recording_skill.last_context.metadata["active_skill_key"], "coarse_approach")
        self.assertFalse(result.transitioned)
        self.assertEqual(result.scheduler_state.current_node_id, "coarse_approach")
        self.assertEqual(result.transition_reason, "topology_blocked:blocked_edge:joint1->tip")
        self.assertTrue(result.diagnostics["topology_used"])
        self.assertEqual(result.skill_result.diagnostics["topology_source"], "external_context")
        self.assertEqual(result.skill_result.diagnostics["context_graph_id"], "scheduler_context_topology")
        self.assertEqual(result.skill_result.diagnostics["context_node_id"], "coarse_approach")
        self.assertEqual(result.skill_result.diagnostics["context_active_skill_key"], "coarse_approach")


if __name__ == "__main__":
    unittest.main()

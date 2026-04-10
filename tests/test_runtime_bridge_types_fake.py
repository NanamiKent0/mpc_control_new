"""Fake tests for scheduler-to-runtime bridge dataclasses."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mpc_control_new.control_core.models.runtime_bridge_types import SchedulerDispatchEnvelope
from mpc_control_new.control_core.models.skill_types import (
    PrimitiveReference,
    SkillCheckResult,
    SkillCompletionResult,
    SkillExecutionResult,
    SkillSpec,
)
from mpc_control_new.control_core.models.task_types import SchedulerState, SchedulerStepResult, TaskNode
from mpc_control_new.control_core.orchestration.transition_policy import TransitionDecision
from mpc_control_new.control_core.topology.relation_state import RelationState


def _spec(skill_key: str = "coarse_approach") -> SkillSpec:
    """Build a minimal skill spec for runtime-bridge tests."""
    return SkillSpec(
        skill_key=skill_key,
        active_module="joint1",
        passive_module="tip",
        relation_type="tip_joint",
        distance_done_mm=10.0,
    )


class RuntimeBridgeTypesFakeTest(unittest.TestCase):
    """Verify bridge objects preserve scheduler and command context."""

    def test_step_result_can_become_dispatch_envelope(self) -> None:
        """A step result should convert into a dispatch envelope with command details."""
        spec = _spec()
        node = TaskNode(node_id="node1", skill_spec=spec, next_node_id="node2")
        skill_result = SkillExecutionResult(
            skill_spec=spec,
            relation_state=RelationState(
                active_module=spec.active_module,
                passive_module=spec.passive_module,
                relation_type=spec.relation_type,
                distance_mm=20.0,
                orientation_error_deg=None,
                coupled=False,
                observation_valid=True,
            ),
            preconditions=SkillCheckResult(passed=True),
            completion=SkillCompletionResult(done=False),
            primitive_references=[
                PrimitiveReference(
                    module_id="joint1",
                    primitive_name="joint_crawl",
                    axis="crawl",
                    reference_kind="velocity",
                    reference_value=2.0,
                    units="mm/s",
                    primary=True,
                    semantic="reduce_distance",
                )
            ],
            selected_primitives=["joint_crawl"],
            status="active",
            topology_snapshot={"topology_active": True},
            frontier_snapshot={"frontier_joint1_tip": True},
            support_snapshot={"support_tip": False},
            diagnostics={"geometry_observation_kind": "tip_joint"},
        )
        step_result = SchedulerStepResult(
            scheduler_state=SchedulerState(
                graph_id="bridge_graph",
                current_node_id="node1",
                node_attempt_counts={"node1": 1},
                last_transition_reason="retry_scheduled:1/2",
            ),
            node=node,
            skill_result=skill_result,
            transitioned=False,
            transition_reason="retry_scheduled:1/2",
            selected_next_node_id=None,
            retry_scheduled=True,
            fallback_used=False,
            transition_decision=TransitionDecision(
                retry_scheduled=True,
                reason="retry_scheduled:1/2",
            ),
            resolved_transition_policy_key="default",
            transition_policy_source="node",
            execution_context_metadata={
                "graph_id": "bridge_graph",
                "node_id": "node1",
                "resolved_transition_policy_key": "default",
                "caller": "runtime_bridge_test",
            },
            diagnostics={
                "graph_id": "bridge_graph",
                "geometry_observation_kind": "tip_joint",
                "selected_primitives": ["joint_crawl"],
            },
        )

        envelope = SchedulerDispatchEnvelope.from_step_result(step_result)
        envelope_via_method = step_result.to_dispatch_envelope()

        self.assertEqual(envelope.scheduled_command.graph_id, "bridge_graph")
        self.assertEqual(envelope.scheduled_command.node_id, "node1")
        self.assertEqual(envelope.scheduled_command.skill_key, "coarse_approach")
        self.assertEqual(envelope.scheduled_command.selected_primitives, ["joint_crawl"])
        self.assertEqual(envelope.scheduled_command.topology_snapshot["topology_active"], True)
        self.assertEqual(envelope.context_metadata["caller"], "runtime_bridge_test")
        self.assertTrue(envelope.transition_decision.retry_scheduled)
        self.assertEqual(envelope.diagnostics["geometry_observation_kind"], "tip_joint")
        self.assertEqual(envelope_via_method.scheduled_command.skill_key, "coarse_approach")


if __name__ == "__main__":
    unittest.main()

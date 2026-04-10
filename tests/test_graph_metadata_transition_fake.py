"""Fake tests for metadata-aware scheduler transitions."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mpc_control_new.control_core.models.execution_context import ExecutionContext
from mpc_control_new.control_core.models.skill_types import (
    SkillCheckResult,
    SkillCompletionResult,
    SkillExecutionResult,
    SkillSpec,
)
from mpc_control_new.control_core.models.task_types import TaskGraphSpec, TaskNode
from mpc_control_new.control_core.orchestration.skill_scheduler import SkillScheduler
from mpc_control_new.control_core.topology.relation_state import RelationState


def _spec(skill_key: str) -> SkillSpec:
    """Build a minimal scheduler spec for metadata tests."""
    return SkillSpec(
        skill_key=skill_key,
        active_module="joint1",
        passive_module="tip",
        relation_type="tip_joint",
        distance_done_mm=10.0,
    )


def _result(spec: SkillSpec, *, done: bool) -> SkillExecutionResult:
    """Return a minimal result for scheduler metadata tests."""
    return SkillExecutionResult(
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
        completion=SkillCompletionResult(done=done, completion_reason="done" if done else None),
        status="done" if done else "active",
    )


class _RecordingAdapter:
    """Adapter double that records the contexts supplied by the scheduler."""

    def __init__(self, queued_results: list[SkillExecutionResult]) -> None:
        self.queued_results = list(queued_results)
        self.contexts: list[ExecutionContext | None] = []

    def execute_skill(
        self,
        spec: SkillSpec,
        estimate: object,
        *,
        context: ExecutionContext | None = None,
    ) -> SkillExecutionResult:
        del spec, estimate
        self.contexts.append(context)
        return self.queued_results.pop(0)


class GraphMetadataTransitionFakeTest(unittest.TestCase):
    """Verify metadata influences transition decisions and is visible in context."""

    def test_force_failure_after_n_attempts_uses_metadata(self) -> None:
        """Node metadata should be able to force a failure path after N attempts."""
        coarse_spec = _spec("coarse_approach")
        fail_spec = _spec("terminal_noop")
        adapter = _RecordingAdapter(
            [
                _result(coarse_spec, done=False),
                _result(coarse_spec, done=False),
            ]
        )
        scheduler = SkillScheduler(adapter=adapter)
        scheduler.load_graph(
            TaskGraphSpec(
                graph_id="metadata_transition_graph",
                start_node_id="coarse",
                metadata={"transition_policy_key": "default"},
                nodes={
                    "coarse": TaskNode(
                        node_id="coarse",
                        skill_spec=coarse_spec,
                        retry_limit=5,
                        on_failure_node_id="failure_sink",
                        transition_policy_key="default",
                        metadata={"force_failure_after_n_attempts": 2},
                    ),
                    "failure_sink": TaskNode(node_id="failure_sink", skill_spec=fail_spec),
                },
            )
        )

        first = scheduler.step(object())
        self.assertTrue(first.retry_scheduled)
        self.assertEqual(adapter.contexts[0].metadata["graph_id"], "metadata_transition_graph")
        self.assertEqual(adapter.contexts[0].metadata["node_id"], "coarse")
        self.assertEqual(adapter.contexts[0].metadata["node_retry_limit"], 5)
        self.assertEqual(adapter.contexts[0].metadata["node_transition_policy_key"], "default")
        self.assertEqual(adapter.contexts[0].metadata["resolved_transition_policy_key"], "default")

        second = scheduler.step(object())
        self.assertTrue(second.transitioned)
        self.assertTrue(second.fallback_used)
        self.assertEqual(second.selected_next_node_id, "failure_sink")
        self.assertIn("forced_failure_after_n_attempts", second.transition_reason)
        self.assertEqual(second.diagnostics["node_force_failure_after_n_attempts"], 2)


if __name__ == "__main__":
    unittest.main()

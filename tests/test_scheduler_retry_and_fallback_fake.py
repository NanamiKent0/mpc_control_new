"""Fake tests for scheduler retry and fallback behaviour."""

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
    """Build a small scheduler test spec."""
    return SkillSpec(
        skill_key=skill_key,
        active_module="joint1",
        passive_module="tip",
        relation_type="tip_joint",
        distance_done_mm=10.0,
    )


def _result(spec: SkillSpec, *, passed: bool, done: bool) -> SkillExecutionResult:
    """Build a minimal execution result returned by the fake adapter."""
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
        preconditions=SkillCheckResult(passed=passed),
        completion=SkillCompletionResult(done=done, completion_reason="done" if done else None),
        status="done" if done else "active" if passed else "blocked",
    )


class _FakeAdapter:
    """Small adapter double that records calls and returns queued results."""

    def __init__(self, queued_results: list[SkillExecutionResult]) -> None:
        self.queued_results = list(queued_results)
        self.calls: list[tuple[SkillSpec, object, ExecutionContext | None]] = []

    def execute_skill(
        self,
        spec: SkillSpec,
        estimate: object,
        *,
        context: ExecutionContext | None = None,
    ) -> SkillExecutionResult:
        self.calls.append((spec, estimate, context))
        return self.queued_results.pop(0)


class SchedulerRetryAndFallbackFakeTest(unittest.TestCase):
    """Verify bounded retry and fallback behaviour in the scheduler."""

    def test_scheduler_retries_then_uses_failure_path(self) -> None:
        """An incomplete node should retry until the limit, then branch to fallback."""
        coarse_spec = _spec("coarse_approach")
        fail_spec = _spec("terminal_noop")
        graph_spec = TaskGraphSpec(
            graph_id="retry_then_fail_graph",
            start_node_id="coarse",
            nodes={
                "coarse": TaskNode(
                    node_id="coarse",
                    skill_spec=coarse_spec,
                    retry_limit=2,
                    on_failure_node_id="failure_sink",
                ),
                "failure_sink": TaskNode(node_id="failure_sink", skill_spec=fail_spec),
            },
        )
        adapter = _FakeAdapter(
            [
                _result(coarse_spec, passed=True, done=False),
                _result(coarse_spec, passed=True, done=False),
                _result(fail_spec, passed=True, done=True),
            ]
        )
        scheduler = SkillScheduler(adapter=adapter)
        scheduler.load_graph(graph_spec)

        first = scheduler.step(object())
        self.assertFalse(first.transitioned)
        self.assertTrue(first.retry_scheduled)
        self.assertFalse(first.fallback_used)
        self.assertEqual(first.scheduler_state.current_node_id, "coarse")
        self.assertEqual(first.scheduler_state.node_attempt_counts["coarse"], 1)

        second = scheduler.step(object())
        self.assertTrue(second.transitioned)
        self.assertFalse(second.retry_scheduled)
        self.assertTrue(second.fallback_used)
        self.assertEqual(second.selected_next_node_id, "failure_sink")
        self.assertEqual(second.scheduler_state.current_node_id, "failure_sink")
        self.assertEqual(second.scheduler_state.node_attempt_counts["coarse"], 2)

        third = scheduler.step(object())
        self.assertTrue(third.transitioned)
        self.assertTrue(third.scheduler_state.is_finished)
        self.assertEqual(third.scheduler_state.node_attempt_counts["failure_sink"], 1)


if __name__ == "__main__":
    unittest.main()

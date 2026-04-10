"""Fake tests for normalized max-attempt retry semantics."""

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
    """Build a minimal scheduler test spec."""
    return SkillSpec(
        skill_key=skill_key,
        active_module="joint1",
        passive_module="tip",
        relation_type="tip_joint",
        distance_done_mm=10.0,
    )


def _result(spec: SkillSpec, *, done: bool) -> SkillExecutionResult:
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
        preconditions=SkillCheckResult(passed=True),
        completion=SkillCompletionResult(done=done, completion_reason="done" if done else None),
        status="done" if done else "active",
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


class RetrySemanticsFakeTest(unittest.TestCase):
    """Verify that retry semantics now consistently mean max attempts."""

    def test_task_node_retry_limit_is_normalized_as_max_attempts(self) -> None:
        """The legacy field name should now expose max-attempt semantics explicitly."""
        node = TaskNode(node_id="coarse", skill_spec=_spec("coarse_approach"), retry_limit=3)

        self.assertEqual(node.max_attempts, 3)
        self.assertEqual(node.retry_budget, 2)

    def test_scheduler_retries_until_max_attempts_then_falls_back(self) -> None:
        """Three max attempts means two retries, then fallback on the third failure."""
        coarse_spec = _spec("coarse_approach")
        failure_spec = _spec("terminal_noop")
        graph_spec = TaskGraphSpec(
            graph_id="max_attempts_graph",
            start_node_id="coarse",
            nodes={
                "coarse": TaskNode(
                    node_id="coarse",
                    skill_spec=coarse_spec,
                    retry_limit=3,
                    on_failure_node_id="failure_sink",
                ),
                "failure_sink": TaskNode(node_id="failure_sink", skill_spec=failure_spec),
            },
        )
        adapter = _FakeAdapter(
            [
                _result(coarse_spec, done=False),
                _result(coarse_spec, done=False),
                _result(coarse_spec, done=False),
                _result(failure_spec, done=True),
            ]
        )
        scheduler = SkillScheduler(adapter=adapter)
        scheduler.load_graph(graph_spec)

        first = scheduler.step(object())
        second = scheduler.step(object())
        third = scheduler.step(object())
        fourth = scheduler.step(object())

        self.assertTrue(first.retry_scheduled)
        self.assertEqual(first.scheduler_state.node_attempt_counts["coarse"], 1)
        self.assertTrue(first.scheduler_state.last_retry_scheduled)
        self.assertFalse(first.scheduler_state.last_fallback_used)

        self.assertTrue(second.retry_scheduled)
        self.assertEqual(second.scheduler_state.node_attempt_counts["coarse"], 2)
        self.assertEqual(second.diagnostics["node_retry_budget"], 2)

        self.assertTrue(third.transitioned)
        self.assertTrue(third.fallback_used)
        self.assertEqual(third.selected_next_node_id, "failure_sink")
        self.assertEqual(third.scheduler_state.node_attempt_counts["coarse"], 3)
        self.assertTrue(third.scheduler_state.last_fallback_used)
        self.assertEqual(
            third.scheduler_state.last_failure_reason,
            "max_attempts_exhausted:3/3->failure_sink",
        )

        self.assertTrue(fourth.transitioned)
        self.assertTrue(fourth.scheduler_state.is_finished)
        self.assertEqual(fourth.scheduler_state.node_attempt_counts["failure_sink"], 1)


if __name__ == "__main__":
    unittest.main()

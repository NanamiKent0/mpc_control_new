"""Fake scheduler tests focused on state transitions and adapter calls."""

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
        orientation_done_deg=5.0 if skill_key == "fine_dock" else None,
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
            orientation_error_deg=10.0,
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


class SkillSchedulerFakeTest(unittest.TestCase):
    """Verify scheduler state updates independently of adapter internals."""

    def test_scheduler_load_reset_and_step_flow(self) -> None:
        """The scheduler should stay, advance, and finish as node results change."""
        coarse_spec = _spec("coarse_approach")
        fine_spec = _spec("fine_dock")
        graph_spec = TaskGraphSpec(
            graph_id="scheduler_fake_graph",
            start_node_id="node1",
            nodes={
                "node1": TaskNode(node_id="node1", skill_spec=coarse_spec, next_node_id="node2"),
                "node2": TaskNode(node_id="node2", skill_spec=fine_spec),
            },
        )
        adapter = _FakeAdapter(
            [
                _result(coarse_spec, passed=True, done=False),
                _result(coarse_spec, passed=True, done=True),
                _result(fine_spec, passed=True, done=True),
            ]
        )
        scheduler = SkillScheduler(adapter=adapter)

        load_state = scheduler.load_graph(graph_spec)
        self.assertEqual(load_state.current_node_id, "node1")
        reset_state = scheduler.reset()
        self.assertEqual(reset_state.current_node_id, "node1")

        first = scheduler.step(object())
        self.assertEqual(len(adapter.calls), 1)
        self.assertEqual(first.skill_result.skill_key, "coarse_approach")
        self.assertFalse(first.transitioned)
        self.assertEqual(first.scheduler_state.current_node_id, "node1")

        second = scheduler.step(object())
        self.assertTrue(second.transitioned)
        self.assertEqual(second.scheduler_state.current_node_id, "node2")
        self.assertEqual(second.scheduler_state.completed_node_ids, ["node1"])
        self.assertEqual(second.diagnostics["skill_key"], "coarse_approach")

        third = scheduler.step(object())
        self.assertTrue(third.transitioned)
        self.assertTrue(third.scheduler_state.is_finished)
        self.assertIsNone(third.scheduler_state.current_node_id)
        self.assertEqual(third.scheduler_state.completed_node_ids, ["node1", "node2"])


if __name__ == "__main__":
    unittest.main()

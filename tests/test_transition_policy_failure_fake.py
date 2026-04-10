"""Fake tests for Phase-5 transition policy failure and success routing."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mpc_control_new.control_core.models.skill_types import (
    SkillCheckResult,
    SkillCompletionResult,
    SkillExecutionResult,
    SkillSpec,
)
from mpc_control_new.control_core.models.task_types import TaskGraphSpec, TaskNode
from mpc_control_new.control_core.orchestration.task_graph import TaskGraph
from mpc_control_new.control_core.orchestration.transition_policy import DefaultTransitionPolicy
from mpc_control_new.control_core.topology.relation_state import RelationState


def _spec(skill_key: str = "coarse_approach") -> SkillSpec:
    """Build a minimal skill spec for policy tests."""
    return SkillSpec(
        skill_key=skill_key,
        active_module="joint1",
        passive_module="tip",
        relation_type="tip_joint",
        distance_done_mm=10.0,
    )


def _result(*, passed: bool, done: bool, block_reason: str | None = None) -> SkillExecutionResult:
    """Build a minimal skill execution result for policy tests."""
    spec = _spec()
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
        preconditions=SkillCheckResult(
            passed=passed,
            blocking_reason=block_reason,
            block_reason=block_reason,
        ),
        completion=SkillCompletionResult(done=done, completion_reason="done" if done else None),
        status="done" if done else "active" if passed else "blocked",
        block_reason=block_reason,
    )


class TransitionPolicyFailureFakeTest(unittest.TestCase):
    """Verify failure and success branch handling in the default policy."""

    def test_preconditions_failure_uses_failure_path(self) -> None:
        """A failed precondition should route into the configured failure node."""
        graph = TaskGraph(
            TaskGraphSpec(
                graph_id="failure_path_graph",
                start_node_id="node1",
                nodes={
                    "node1": TaskNode(
                        node_id="node1",
                        skill_spec=_spec(),
                        on_failure_node_id="node_fail",
                    ),
                    "node_fail": TaskNode(node_id="node_fail", skill_spec=_spec("terminal_noop")),
                },
            )
        )
        node = graph.get_node("node1")

        decision = DefaultTransitionPolicy().decide(
            graph,
            node,
            _result(passed=False, done=False, block_reason="sensor_missing"),
            attempt_count=1,
        )

        self.assertTrue(decision.transitioned)
        self.assertEqual(decision.next_node_id, "node_fail")
        self.assertTrue(decision.fallback_used)

    def test_completion_done_uses_success_path(self) -> None:
        """Completion should prefer explicit success edges over linear next edges."""
        graph = TaskGraph(
            TaskGraphSpec(
                graph_id="success_path_graph",
                start_node_id="node1",
                nodes={
                    "node1": TaskNode(
                        node_id="node1",
                        skill_spec=_spec(),
                        next_node_id="node2",
                        on_success_node_id="node_success",
                    ),
                    "node2": TaskNode(node_id="node2", skill_spec=_spec("fine_dock")),
                    "node_success": TaskNode(node_id="node_success", skill_spec=_spec("fine_dock")),
                },
            )
        )
        node = graph.get_node("node1")

        decision = DefaultTransitionPolicy().decide(
            graph,
            node,
            _result(passed=True, done=True),
            attempt_count=1,
        )

        self.assertTrue(decision.transitioned)
        self.assertEqual(decision.next_node_id, "node_success")

    def test_no_successor_marks_graph_finished(self) -> None:
        """When a completed node has no successor, the graph should finish."""
        graph = TaskGraph(
            TaskGraphSpec(
                graph_id="terminal_graph",
                start_node_id="node1",
                nodes={"node1": TaskNode(node_id="node1", skill_spec=_spec("fine_dock"))},
            )
        )

        decision = DefaultTransitionPolicy().decide(
            graph,
            graph.get_node("node1"),
            _result(passed=True, done=True),
            attempt_count=1,
        )

        self.assertTrue(decision.transitioned)
        self.assertTrue(decision.finished)
        self.assertIsNone(decision.next_node_id)

    def test_preconditions_failure_without_failure_path_stays_put(self) -> None:
        """Without a failure edge, a failed precondition should not force a transition."""
        graph = TaskGraph(
            TaskGraphSpec(
                graph_id="stay_put_graph",
                start_node_id="node1",
                nodes={"node1": TaskNode(node_id="node1", skill_spec=_spec())},
            )
        )

        decision = DefaultTransitionPolicy().decide(
            graph,
            graph.get_node("node1"),
            _result(passed=False, done=False, block_reason="missing_geometry"),
            attempt_count=1,
        )

        self.assertFalse(decision.transitioned)
        self.assertEqual(decision.reason, "preconditions_failed:missing_geometry")


if __name__ == "__main__":
    unittest.main()

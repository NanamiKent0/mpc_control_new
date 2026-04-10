"""Fake tests for the standalone transition policy registry."""

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
from mpc_control_new.control_core.orchestration.transition_policy import DefaultTransitionPolicy
from mpc_control_new.control_core.orchestration.transition_policy_registry import (
    TransitionPolicyRegistry,
    build_default_transition_policy_registry,
)
from mpc_control_new.control_core.topology.relation_state import RelationState


def _spec(skill_key: str) -> SkillSpec:
    """Build a minimal skill spec for policy-registry tests."""
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
    """Small adapter double that records contexts and returns queued results."""

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


class TransitionPolicyRegistryFakeTest(unittest.TestCase):
    """Verify transition policies can be registered, resolved, and consumed."""

    def test_registry_can_register_and_resolve_policy(self) -> None:
        """A registered policy should be retrievable and listable by key."""
        registry = TransitionPolicyRegistry()
        policy = DefaultTransitionPolicy()
        registry.register("custom_default", policy, metadata={"family": "fake"})

        self.assertIs(registry.get("custom_default"), policy)
        resolution = registry.resolve("custom_default")
        self.assertTrue(resolution.found)
        self.assertIs(resolution.policy_instance, policy)
        self.assertEqual(resolution.metadata["family"], "fake")
        self.assertEqual(registry.list_registered()[0].policy_key, "custom_default")

    def test_default_registry_contains_default_and_fail_fast(self) -> None:
        """The default registry should expose the baseline sample policies."""
        registry = build_default_transition_policy_registry()

        registered = {descriptor.policy_key for descriptor in registry.list_registered()}
        self.assertIn("default", registered)
        self.assertIn("fail_fast", registered)

    def test_scheduler_switches_policy_by_node_key(self) -> None:
        """The scheduler should execute different transition semantics per policy key."""
        coarse_spec = _spec("coarse_approach")
        failure_spec = _spec("terminal_noop")
        graph_spec = TaskGraphSpec(
            graph_id="policy_switch_graph",
            start_node_id="coarse",
            nodes={
                "coarse": TaskNode(
                    node_id="coarse",
                    skill_spec=coarse_spec,
                    transition_policy_key="fail_fast",
                    retry_limit=3,
                    on_failure_node_id="failure_sink",
                ),
                "failure_sink": TaskNode(node_id="failure_sink", skill_spec=failure_spec),
            },
        )
        adapter = _FakeAdapter([_result(coarse_spec, done=False)])
        scheduler = SkillScheduler(
            adapter=adapter,
            transition_policy_registry=build_default_transition_policy_registry(),
        )
        scheduler.load_graph(graph_spec)

        result = scheduler.step(object())

        self.assertTrue(result.transitioned)
        self.assertTrue(result.fallback_used)
        self.assertEqual(result.selected_next_node_id, "failure_sink")
        self.assertEqual(result.resolved_transition_policy_key, "fail_fast")
        self.assertEqual(result.transition_policy_source, "node")

    def test_scheduler_reports_structured_error_for_missing_policy(self) -> None:
        """Missing policy keys should return a structured scheduler error."""
        coarse_spec = _spec("coarse_approach")
        graph_spec = TaskGraphSpec(
            graph_id="missing_policy_graph",
            start_node_id="coarse",
            nodes={
                "coarse": TaskNode(
                    node_id="coarse",
                    skill_spec=coarse_spec,
                    transition_policy_key="missing_policy",
                )
            },
        )
        scheduler = SkillScheduler(
            adapter=_FakeAdapter([_result(coarse_spec, done=False)]),
            transition_policy_registry=build_default_transition_policy_registry(),
        )
        scheduler.load_graph(graph_spec)

        result = scheduler.step(object())

        self.assertFalse(result.transitioned)
        self.assertEqual(result.transition_policy_error, "transition_policy_not_registered:missing_policy")
        self.assertEqual(result.diagnostics["transition_policy_error"], "transition_policy_not_registered:missing_policy")
        self.assertEqual(result.scheduler_state.current_node_id, "coarse")


if __name__ == "__main__":
    unittest.main()

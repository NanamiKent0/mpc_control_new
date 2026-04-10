"""Transition policies used by the Phase-6 skill scheduler."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Protocol

from ..models.execution_context import ExecutionContext
from ..models.skill_types import SkillExecutionResult
from ..models.task_types import SchedulerState, TaskNode
from .task_graph import TaskGraph


@dataclass(slots=True)
class TransitionDecision:
    """Structured transition decision emitted by the scheduler policy."""

    next_node_id: str | None = None
    transitioned: bool = False
    retry_scheduled: bool = False
    fallback_used: bool = False
    finished: bool = False
    reason: str | None = None
    diagnostics: dict[str, object] = field(default_factory=dict)

    @property
    def is_finished(self) -> bool:
        """Backward-compatible alias used by older callers/tests."""
        return self.finished


class TransitionPolicy(Protocol):
    """Protocol implemented by scheduler transition policies."""

    def decide(
        self,
        graph: TaskGraph,
        node: TaskNode,
        skill_result: SkillExecutionResult | None,
        *,
        attempt_count: int,
        context: ExecutionContext | None = None,
        scheduler_state: SchedulerState | None = None,
    ) -> TransitionDecision:
        """Decide whether the scheduler should stay, retry, branch, or finish."""


class DefaultTransitionPolicy:
    """Transparent policy supporting success, bounded retries, and fallback edges."""

    def decide(
        self,
        graph: TaskGraph,
        node: TaskNode,
        skill_result: SkillExecutionResult | None,
        *,
        attempt_count: int,
        context: ExecutionContext | None = None,
        scheduler_state: SchedulerState | None = None,
    ) -> TransitionDecision:
        """Decide whether the scheduler should stay, retry, branch, or finish."""
        del scheduler_state
        if skill_result is None:
            return TransitionDecision(reason="missing_skill_result")

        if not skill_result.preconditions.ok:
            reason = self._precondition_reason(skill_result)
            if node.on_failure_node_id is not None:
                return TransitionDecision(
                    next_node_id=node.on_failure_node_id,
                    transitioned=True,
                    fallback_used=True,
                    reason=f"{reason}->{node.on_failure_node_id}",
                    diagnostics={"decision_kind": "precondition_failure"},
                )
            return TransitionDecision(
                reason=reason,
                diagnostics={"decision_kind": "precondition_failure"},
            )

        if skill_result.completion.done:
            next_node_id = node.on_success_node_id or node.next_node_id
            if next_node_id is None:
                return TransitionDecision(
                    transitioned=True,
                    finished=True,
                    reason=f"graph_finished:{node.node_id}",
                    diagnostics={"decision_kind": "success_finish"},
                )
            return TransitionDecision(
                next_node_id=next_node_id,
                transitioned=True,
                reason=f"completion_done:{node.node_id}->{next_node_id}",
                diagnostics={"decision_kind": "success_transition"},
            )

        forced_failure_threshold = self._force_failure_after_n_attempts(graph, node, context)
        if forced_failure_threshold is not None and attempt_count >= forced_failure_threshold:
            if node.on_failure_node_id is not None:
                return TransitionDecision(
                    next_node_id=node.on_failure_node_id,
                    transitioned=True,
                    fallback_used=True,
                    reason=(
                        "forced_failure_after_n_attempts:"
                        f"{attempt_count}/{forced_failure_threshold}->{node.on_failure_node_id}"
                    ),
                    diagnostics={"decision_kind": "forced_failure"},
                )
            return TransitionDecision(
                reason=f"forced_failure_after_n_attempts:{attempt_count}/{forced_failure_threshold}",
                diagnostics={"decision_kind": "forced_failure"},
            )

        if attempt_count < node.max_attempts:
            return TransitionDecision(
                retry_scheduled=True,
                reason=f"retry_scheduled:{attempt_count}/{node.max_attempts}",
                diagnostics={"decision_kind": "retry"},
            )

        if node.on_failure_node_id is not None:
            return TransitionDecision(
                next_node_id=node.on_failure_node_id,
                transitioned=True,
                fallback_used=True,
                reason=f"max_attempts_exhausted:{attempt_count}/{node.max_attempts}->{node.on_failure_node_id}",
                diagnostics={"decision_kind": "fallback"},
            )

        return TransitionDecision(
            reason=f"skill_in_progress_no_transition:{attempt_count}/{node.max_attempts}",
            diagnostics={"decision_kind": "stay"},
        )

    @staticmethod
    def _precondition_reason(skill_result: SkillExecutionResult) -> str:
        """Return the stable failure reason for a failed precondition check."""
        reason = (
            skill_result.block_reason
            or skill_result.preconditions.blocking_reason
            or "preconditions_not_met"
        )
        if skill_result.blocked_by_topology:
            return f"topology_blocked:{reason}"
        return f"preconditions_failed:{reason}"

    @staticmethod
    def _force_failure_after_n_attempts(
        graph: TaskGraph,
        node: TaskNode,
        context: ExecutionContext | None,
    ) -> int | None:
        """Resolve the optional metadata-driven forced failure threshold."""
        candidates = (
            node.metadata.get("force_failure_after_n_attempts"),
            graph.spec.metadata.get("force_failure_after_n_attempts"),
            None if context is None else context.metadata.get("force_failure_after_n_attempts"),
        )
        for candidate in candidates:
            if isinstance(candidate, int) and candidate > 0:
                return candidate
            if isinstance(candidate, float) and candidate.is_integer() and candidate > 0:
                return int(candidate)
            if isinstance(candidate, str) and candidate.strip().isdigit():
                return int(candidate.strip())
        return None


class FailFastTransitionPolicy:
    """Simple policy that skips retries and routes failures immediately."""

    def decide(
        self,
        graph: TaskGraph,
        node: TaskNode,
        skill_result: SkillExecutionResult | None,
        *,
        attempt_count: int,
        context: ExecutionContext | None = None,
        scheduler_state: SchedulerState | None = None,
    ) -> TransitionDecision:
        """Route success normally and treat incomplete/failure outcomes as immediate failure."""
        del graph, context, scheduler_state
        if skill_result is None:
            return TransitionDecision(reason="missing_skill_result")

        if not skill_result.preconditions.ok:
            reason = DefaultTransitionPolicy._precondition_reason(skill_result)
            if node.on_failure_node_id is not None:
                return TransitionDecision(
                    next_node_id=node.on_failure_node_id,
                    transitioned=True,
                    fallback_used=True,
                    reason=f"fail_fast:{reason}->{node.on_failure_node_id}",
                    diagnostics={"decision_kind": "fail_fast_precondition_failure"},
                )
            return TransitionDecision(
                transitioned=True,
                finished=True,
                reason=f"fail_fast:{reason}",
                diagnostics={"decision_kind": "fail_fast_finish"},
            )

        if skill_result.completion.done:
            next_node_id = node.on_success_node_id or node.next_node_id
            if next_node_id is None:
                return TransitionDecision(
                    transitioned=True,
                    finished=True,
                    reason=f"graph_finished:{node.node_id}",
                    diagnostics={"decision_kind": "success_finish"},
                )
            return TransitionDecision(
                next_node_id=next_node_id,
                transitioned=True,
                reason=f"completion_done:{node.node_id}->{next_node_id}",
                diagnostics={"decision_kind": "success_transition"},
            )

        if node.on_failure_node_id is not None:
            return TransitionDecision(
                next_node_id=node.on_failure_node_id,
                transitioned=True,
                fallback_used=True,
                reason=f"fail_fast_incomplete:{attempt_count}/{node.max_attempts}->{node.on_failure_node_id}",
                diagnostics={"decision_kind": "fail_fast_fallback"},
            )

        return TransitionDecision(
            reason=f"fail_fast_incomplete:{attempt_count}/{node.max_attempts}",
            diagnostics={"decision_kind": "fail_fast_stay"},
        )

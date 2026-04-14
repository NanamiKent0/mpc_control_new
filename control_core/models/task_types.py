"""Task-graph dataclasses used by the Phase-6 orchestration scaffold."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Literal

from .skill_types import SkillExecutionResult, SkillSpec

if TYPE_CHECKING:
    from ..orchestration.transition_policy import TransitionDecision
    from .runtime_bridge_types import SchedulerDispatchEnvelope


TIP_FREE_GROWTH = "TIP_FREE_GROWTH"
TIP_TURN_AUTONOMOUS = "TIP_TURN_AUTONOMOUS"
HighLevelTaskKind = Literal["TIP_FREE_GROWTH", "TIP_TURN_AUTONOMOUS"]


@dataclass(slots=True)
class HighLevelTaskRequest:
    """One high-level orchestration request compiled into a task graph."""

    task_kind: HighLevelTaskKind
    metadata: dict[str, object] = field(default_factory=dict)


@dataclass(slots=True)
class TaskNode:
    """One orchestration node bound to a concrete skill specification.

    `retry_limit` keeps its legacy field name for compatibility, but its
    normalized semantics are now "maximum attempts", including the first
    execution. A value of `1` means "execute once and do not retry".
    """

    node_id: str
    skill_spec: SkillSpec
    next_node_id: str | None = None
    on_success_node_id: str | None = None
    on_failure_node_id: str | None = None
    transition_policy_key: str | None = None
    retry_limit: int = 1
    metadata: dict[str, object] = field(default_factory=dict)

    @property
    def max_attempts(self) -> int:
        """Return the normalized maximum-attempt count for this node."""
        return int(self.retry_limit)

    @property
    def retry_budget(self) -> int:
        """Return how many additional retries remain beyond the first attempt."""
        return max(self.max_attempts - 1, 0)


@dataclass(slots=True)
class TaskGraphSpec:
    """Declarative task graph specification for the Phase-6 scheduler."""

    graph_id: str
    start_node_id: str
    nodes: dict[str, TaskNode] = field(default_factory=dict)
    metadata: dict[str, object] = field(default_factory=dict)


@dataclass(slots=True)
class SchedulerState:
    """Mutable scheduler state kept alongside one loaded task graph."""

    graph_id: str | None = None
    current_node_id: str | None = None
    previous_node_id: str | None = None
    completed_node_ids: list[str] = field(default_factory=list)
    node_attempt_counts: dict[str, int] = field(default_factory=dict)
    is_finished: bool = False
    last_transition_reason: str | None = None
    last_skill_key: str | None = None
    last_failure_reason: str | None = None
    last_completion_done: bool | None = None
    last_preconditions_ok: bool | None = None
    last_retry_scheduled: bool = False
    last_fallback_used: bool = False
    last_transitioned: bool = False
    last_resolved_transition_policy_key: str | None = None
    last_transition_policy_source: str | None = None
    last_transition_policy_error: str | None = None

    def attempt_count_for(self, node_id: str) -> int:
        """Return the current attempt count for one node."""
        return self.node_attempt_counts.get(node_id, 0)


@dataclass(slots=True)
class SchedulerStepResult:
    """Returned state for one scheduler step invocation."""

    scheduler_state: SchedulerState
    node: TaskNode | None = None
    skill_result: SkillExecutionResult | None = None
    transitioned: bool = False
    transition_reason: str | None = None
    selected_next_node_id: str | None = None
    retry_scheduled: bool = False
    fallback_used: bool = False
    transition_decision: "TransitionDecision | None" = None
    resolved_transition_policy_key: str | None = None
    transition_policy_source: str | None = None
    transition_policy_error: str | None = None
    execution_context_metadata: dict[str, object] = field(default_factory=dict)
    diagnostics: dict[str, object] = field(default_factory=dict)

    def to_dispatch_envelope(self) -> "SchedulerDispatchEnvelope":
        """Convert this step result into the scheduler-runtime bridge envelope."""
        from .runtime_bridge_types import SchedulerDispatchEnvelope

        return SchedulerDispatchEnvelope.from_step_result(self)

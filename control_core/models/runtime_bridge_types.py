"""Bridge dataclasses between the scheduler and runtime dispatch layers."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

from .skill_types import PrimitiveReference
from ..topology.relation_state import DiagnosticValue

if TYPE_CHECKING:
    from ..orchestration.transition_policy import TransitionDecision
    from .task_types import SchedulerState, SchedulerStepResult


@dataclass(slots=True)
class DispatchResult:
    """Structured command-dispatch outcome returned by runtime dispatchers."""

    accepted: bool
    dispatched_commands: list[str] = field(default_factory=list)
    reason: str | None = None
    diagnostics: dict[str, object] = field(default_factory=dict)


@dataclass(slots=True)
class ScheduledSkillCommand:
    """Skill command payload emitted by the scheduler for future runtime dispatch."""

    graph_id: str | None
    node_id: str
    skill_key: str
    active_module: str
    passive_module: str
    relation_type: str
    selected_primitives: list[str] = field(default_factory=list)
    primitive_references: list[PrimitiveReference] = field(default_factory=list)
    topology_snapshot: dict[str, DiagnosticValue] = field(default_factory=dict)
    frontier_snapshot: dict[str, DiagnosticValue] = field(default_factory=dict)
    support_snapshot: dict[str, DiagnosticValue] = field(default_factory=dict)
    command_summary: dict[str, object] = field(default_factory=dict)
    provider_hint: str | None = None
    dispatcher_hint: str | None = None
    bridge_source: str = "skill_scheduler.runtime_bridge"
    diagnostics: dict[str, object] = field(default_factory=dict)


@dataclass(slots=True)
class SchedulerDispatchEnvelope:
    """Stable scheduler snapshot that can later be consumed by a runtime layer."""

    scheduler_state: "SchedulerState"
    scheduled_command: ScheduledSkillCommand | None = None
    transition_decision: "TransitionDecision | None" = None
    input_source: str | None = None
    dispatch_target: str | None = None
    provider_kind: str | None = None
    dispatcher_kind: str | None = None
    bridge_source: str = "skill_scheduler.runtime_bridge"
    provider_hint: str | None = None
    dispatcher_hint: str | None = None
    command_summary: dict[str, object] = field(default_factory=dict)
    context_metadata: dict[str, object] = field(default_factory=dict)
    diagnostics: dict[str, object] = field(default_factory=dict)

    @property
    def dispatch_ready(self) -> bool:
        """Return whether the envelope carries a dispatchable scheduled command."""
        return self.scheduled_command is not None

    @classmethod
    def from_step_result(cls, step_result: "SchedulerStepResult") -> "SchedulerDispatchEnvelope":
        """Build a bridge envelope from one scheduler step result."""
        provider_hint = _string_or_none(
            step_result.execution_context_metadata.get("runtime_provider_kind")
        )
        dispatcher_hint = _string_or_none(
            step_result.execution_context_metadata.get("runtime_dispatcher_kind")
        )
        scheduled_command = None
        command_summary = _empty_command_summary(step_result.scheduler_state.graph_id)
        if step_result.node is not None and step_result.skill_result is not None:
            skill_result = step_result.skill_result
            command_summary = _build_command_summary(
                graph_id=step_result.scheduler_state.graph_id,
                node_id=step_result.node.node_id,
                skill_key=skill_result.skill_key or step_result.node.skill_spec.skill_key,
                active_module=skill_result.active_module or step_result.node.skill_spec.active_module,
                passive_module=skill_result.passive_module or step_result.node.skill_spec.passive_module,
                selected_primitives=list(skill_result.selected_primitives),
                primitive_references=skill_result.primitive_references,
            )
            scheduled_command = ScheduledSkillCommand(
                graph_id=step_result.scheduler_state.graph_id,
                node_id=step_result.node.node_id,
                skill_key=skill_result.skill_key or step_result.node.skill_spec.skill_key,
                active_module=skill_result.active_module or step_result.node.skill_spec.active_module,
                passive_module=skill_result.passive_module or step_result.node.skill_spec.passive_module,
                relation_type=skill_result.skill_spec.relation_type,
                selected_primitives=list(skill_result.selected_primitives),
                primitive_references=deepcopy(skill_result.primitive_references),
                topology_snapshot=dict(skill_result.topology_snapshot),
                frontier_snapshot=dict(skill_result.frontier_snapshot),
                support_snapshot=dict(skill_result.support_snapshot),
                command_summary=dict(command_summary),
                provider_hint=provider_hint,
                dispatcher_hint=dispatcher_hint,
                diagnostics={
                    "pair": f"{skill_result.active_module}->{skill_result.passive_module}",
                    "status": skill_result.status,
                    "module_ids": sorted(
                        {reference.module_id for reference in skill_result.primitive_references}
                    ),
                    "selected_primitives": list(skill_result.selected_primitives),
                    "bridge_source": "skill_scheduler.runtime_bridge",
                    **dict(skill_result.diagnostics),
                },
            )
        return cls(
            scheduler_state=deepcopy(step_result.scheduler_state),
            scheduled_command=scheduled_command,
            transition_decision=deepcopy(step_result.transition_decision),
            input_source=_string_or_none(step_result.diagnostics.get("scheduler_input_source")),
            bridge_source="skill_scheduler.runtime_bridge",
            provider_hint=provider_hint,
            dispatcher_hint=dispatcher_hint,
            command_summary=dict(command_summary),
            context_metadata=dict(step_result.execution_context_metadata),
            diagnostics={
                **dict(step_result.diagnostics),
                "dispatch_ready": scheduled_command is not None,
                "scheduler_input_source": step_result.diagnostics.get("scheduler_input_source"),
                "state_builder_source": step_result.diagnostics.get("state_builder_source"),
                "legacy_path_used": bool(step_result.diagnostics.get("legacy_path_used", False)),
                "bridge_source": "skill_scheduler.runtime_bridge",
                "provider_hint": provider_hint,
                "dispatcher_hint": dispatcher_hint,
                "command_summary": dict(command_summary),
            },
        )


def _string_or_none(value: object) -> str | None:
    """Normalize an optional non-empty string."""
    if isinstance(value, str) and value:
        return value
    return None


def _empty_command_summary(graph_id: str | None) -> dict[str, object]:
    """Return a stable empty command summary for non-dispatchable steps."""
    return {
        "graph_id": graph_id,
        "node_id": None,
        "skill_key": None,
        "pair": None,
        "primitive_count": 0,
        "selected_primitives": [],
        "module_ids": [],
        "dispatch_ready": False,
    }


def _build_command_summary(
    *,
    graph_id: str | None,
    node_id: str,
    skill_key: str,
    active_module: str,
    passive_module: str,
    selected_primitives: list[str],
    primitive_references: list[PrimitiveReference],
) -> dict[str, object]:
    """Build a compact runtime-facing summary for one scheduled command."""
    module_ids = sorted({reference.module_id for reference in primitive_references})
    return {
        "graph_id": graph_id,
        "node_id": node_id,
        "skill_key": skill_key,
        "pair": f"{active_module}->{passive_module}",
        "primitive_count": len(primitive_references),
        "selected_primitives": list(selected_primitives),
        "module_ids": module_ids,
        "dispatch_ready": True,
    }

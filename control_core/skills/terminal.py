"""Terminal relation skills used by orchestration sink nodes."""

from __future__ import annotations

from typing import Any

from ..models.execution_context import ExecutionContext
from ..models.skill_types import (
    PrimitiveReference,
    SkillCheckResult,
    SkillCompletionResult,
    SkillSpec,
)
from ..topology.chain_topology import ChainTopology
from ..topology.relation_state import RelationState
from .base import RelationSkill


class TerminalNoopSkill(RelationSkill):
    """Minimal terminal skill used by failure or finalize sink nodes."""

    skill_key = "terminal_noop"

    def __init__(self, *, topology: ChainTopology | None = None) -> None:
        self.topology = topology

    def build_relation_state(
        self,
        source: Any,
        spec: SkillSpec,
        context: ExecutionContext | None = None,
    ) -> RelationState:
        """Return the provided relation state or a placeholder terminal snapshot."""
        del context
        return self._coerce_relation_state(
            source
            if isinstance(source, RelationState)
            else RelationState(
                active_module=spec.active_module,
                passive_module=spec.passive_module,
                relation_type=spec.relation_type,
                distance_mm=None,
                orientation_error_deg=None,
                coupled=None,
                observation_valid=False,
                diagnostics={"terminal_noop_placeholder": True},
            ),
            spec,
        )

    def check_preconditions(
        self,
        relation_state: RelationState,
        spec: SkillSpec,
        context: ExecutionContext | None = None,
    ) -> SkillCheckResult:
        """Always allow the terminal skill to execute."""
        del relation_state, spec, context
        return SkillCheckResult(passed=True)

    def generate_primitive_references(
        self,
        relation_state: RelationState,
        spec: SkillSpec,
        context: ExecutionContext | None = None,
    ) -> list[PrimitiveReference]:
        """Terminal nodes do not emit actuator references."""
        del relation_state, spec, context
        return []

    def check_completion(
        self,
        relation_state: RelationState,
        spec: SkillSpec,
        context: ExecutionContext | None = None,
    ) -> SkillCompletionResult:
        """Immediately complete the terminal node."""
        del relation_state, spec, context
        return SkillCompletionResult(done=True, completion_reason="terminal_noop")

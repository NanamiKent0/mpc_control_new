"""Base skill interfaces and helpers for relation-centric control logic."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any, Mapping

from ..models.execution_context import ExecutionContext
from ..models.module_state import ModuleState
from ..models.skill_types import (
    PrimitiveReference,
    SkillCheckResult,
    SkillCompletionResult,
    SkillExecutionResult,
    SkillSpec,
)
from ..topology.chain_topology import ChainTopology
from ..topology.relation_state import DiagnosticValue, RelationState


class SkillSupportMixin:
    """Shared helper methods for small Phase-1 relation skills."""

    @staticmethod
    def safe_float(value: Any, default: float | None = None) -> float | None:
        """Convert a scalar-like value into float with a nullable fallback."""
        if value is None:
            return default
        try:
            return float(value)
        except (TypeError, ValueError):
            return default

    @staticmethod
    def normalize_bool(value: Any) -> bool | None:
        """Normalize a bool-like value while preserving unknown as `None`."""
        if isinstance(value, bool):
            return value
        if value is None:
            return None
        if isinstance(value, str):
            normalized = value.strip().lower()
            if normalized in {"1", "true", "yes", "on"}:
                return True
            if normalized in {"0", "false", "no", "off"}:
                return False
            return None
        if isinstance(value, (int, float)):
            return bool(value)
        return None

    @staticmethod
    def push_note(notes: list[str], message: str | None) -> None:
        """Append a debug note if it is non-empty and not duplicated."""
        if message and message not in notes:
            notes.append(message)

    @classmethod
    def merge_diagnostics(cls, *parts: Mapping[str, Any]) -> dict[str, DiagnosticValue]:
        """Merge diagnostic mappings into a serializable flat dictionary."""
        merged: dict[str, DiagnosticValue] = {}
        for part in parts:
            for key, value in part.items():
                merged[str(key)] = cls._normalize_debug_value(value)
        return merged

    @classmethod
    def _normalize_debug_value(cls, value: Any) -> DiagnosticValue:
        """Convert arbitrary values into the supported debug scalar set."""
        if value is None or isinstance(value, (bool, float, str)):
            return value
        if isinstance(value, int):
            return float(value)
        return str(value)

    def _coerce_relation_state(self, source: Any, spec: SkillSpec) -> RelationState:
        """Return a `RelationState` from a relation object or mapping."""
        if isinstance(source, RelationState):
            return source
        if isinstance(source, Mapping):
            getter = source.get
        else:
            getter = lambda key, default=None: getattr(source, key, default)
        diagnostics = getter("diagnostics", {}) or {}
        return RelationState(
            active_module=str(getter("active_module", spec.active_module)),
            passive_module=str(getter("passive_module", spec.passive_module)),
            relation_type=str(getter("relation_type", spec.relation_type)),
            distance_mm=self.safe_float(getter("distance_mm", None), None),
            orientation_error_deg=self.safe_float(getter("orientation_error_deg", None), None),
            coupled=self.normalize_bool(getter("coupled", None)),
            observation_valid=bool(getter("observation_valid", False)),
            diagnostics=self.merge_diagnostics(diagnostics),
        )


class RelationSkill(SkillSupportMixin, ABC):
    """Abstract interface for reusable relation-centric skills."""

    skill_key: str = ""

    @abstractmethod
    def build_relation_state(
        self,
        source: Any,
        spec: SkillSpec,
        context: ExecutionContext | None = None,
    ) -> RelationState:
        """Build the relation view consumed by this skill."""

    @abstractmethod
    def check_preconditions(
        self,
        relation_state: RelationState,
        spec: SkillSpec,
        context: ExecutionContext | None = None,
    ) -> SkillCheckResult:
        """Validate whether the skill may execute on the current relation."""

    @abstractmethod
    def generate_primitive_references(
        self,
        relation_state: RelationState,
        spec: SkillSpec,
        context: ExecutionContext | None = None,
    ) -> list[PrimitiveReference]:
        """Produce primitive references for the active module."""

    @abstractmethod
    def check_completion(
        self,
        relation_state: RelationState,
        spec: SkillSpec,
        context: ExecutionContext | None = None,
    ) -> SkillCompletionResult:
        """Evaluate whether the skill should be treated as complete."""

    def _resolve_context(self, spec: SkillSpec, context: ExecutionContext | None) -> ExecutionContext:
        """Return a context object with safe defaults applied."""
        metadata = dict(context.metadata) if context is not None else {}
        dt = context.dt if context is not None else None
        if dt is None:
            dt = spec.float_param("solver_dt", spec.solver_dt)
        topology = context.topology if context is not None and context.topology is not None else self._fallback_topology()
        target_ns = context.target_ns if context is not None else None
        return ExecutionContext(
            topology=topology,
            dt=dt,
            target_ns=target_ns,
            metadata=metadata,
        )

    def _fallback_topology(self) -> ChainTopology | None:
        """Return the implicit skill-owned topology when available."""
        topology = getattr(self, "topology", None)
        if isinstance(topology, ChainTopology):
            return topology
        return None

    def _resolve_topology(
        self,
        context: ExecutionContext | None,
    ) -> tuple[ChainTopology | None, bool]:
        """Return the topology source for this execution."""
        if context is not None and context.topology is not None:
            return context.topology, True
        return self._fallback_topology(), False

    def _topology_snapshots(
        self,
        topology: ChainTopology | None,
    ) -> tuple[dict[str, DiagnosticValue], dict[str, DiagnosticValue], dict[str, DiagnosticValue]]:
        """Return topology, frontier, and support snapshots for diagnostics."""
        if topology is None:
            return ({}, {}, {})
        return (
            topology.snapshot(),
            topology.frontier_snapshot(),
            topology.support_snapshot(),
        )

    def _context_bool(
        self,
        spec: SkillSpec,
        context: ExecutionContext | None,
        name: str,
        default: bool = False,
    ) -> bool:
        """Resolve a boolean policy flag from context metadata or spec params."""
        if context is not None and name in context.metadata:
            value = context.metadata[name]
            if isinstance(value, bool):
                return value
            if isinstance(value, str):
                normalized = value.strip().lower()
                if normalized in {"1", "true", "yes", "on"}:
                    return True
                if normalized in {"0", "false", "no", "off"}:
                    return False
            if isinstance(value, (int, float)):
                return bool(value)
        return spec.bool_param(name, default)

    def _evaluate_topology_gate(
        self,
        relation_state: RelationState,
        spec: SkillSpec,
        context: ExecutionContext | None,
    ) -> dict[str, Any]:
        """Evaluate topology, frontier, and support policy for one relation pair."""
        topology, used_context_topology = self._resolve_topology(context)
        topology_snapshot, frontier_snapshot, support_snapshot = self._topology_snapshots(topology)
        allow_off_frontier = self._context_bool(spec, context, "allow_off_frontier", False)
        requires_support_stability = self._context_bool(spec, context, "requires_support_stability", False)
        allow_support_breaking = (
            self._context_bool(spec, context, "allow_support_breaking", False)
            or self._context_bool(spec, context, "allow_support_breaking_actions", False)
        )
        pair_allowed = True
        block_reason = None
        if topology is not None:
            pair_allowed, block_reason = topology.pair_allowed(
                relation_state.active_module,
                relation_state.passive_module,
                allow_off_frontier=allow_off_frontier,
            )
            if pair_allowed and requires_support_stability and not allow_support_breaking:
                if topology.is_support_module(relation_state.active_module):
                    pair_allowed = False
                    block_reason = f"protected_support_module:{relation_state.active_module}"
                elif topology.is_support_module(relation_state.passive_module):
                    pair_allowed = False
                    block_reason = f"protected_support_module:{relation_state.passive_module}"
        return {
            "topology": topology,
            "used_context_topology": used_context_topology,
            "pair_allowed": pair_allowed,
            "block_reason": block_reason,
            "topology_snapshot": topology_snapshot,
            "frontier_snapshot": frontier_snapshot,
            "support_snapshot": support_snapshot,
            "allow_off_frontier": allow_off_frontier,
            "requires_support_stability": requires_support_stability,
            "allow_support_breaking": allow_support_breaking,
        }

    def execute(
        self,
        source: Any,
        spec: SkillSpec,
        *,
        module_states: dict[str, ModuleState] | None = None,
        context: ExecutionContext | None = None,
    ) -> SkillExecutionResult:
        """Execute the full Phase-3 relation-skill pipeline."""
        resolved_context = self._resolve_context(spec, context)
        relation_state = self.build_relation_state(source, spec, resolved_context)
        preconditions = self.check_preconditions(relation_state, spec, resolved_context)
        completion = self.check_completion(relation_state, spec, resolved_context)
        should_generate = preconditions.passed or completion.done
        primitive_references = (
            self.generate_primitive_references(relation_state, spec, resolved_context) if should_generate else []
        )
        status = "active"
        blocked_by_topology = bool(preconditions.blocked_by_topology or completion.blocked_by_topology)
        not_fully_supported = bool(preconditions.not_fully_supported)
        block_reason = (
            preconditions.block_reason
            or completion.block_reason
            or preconditions.blocking_reason
        )
        topology_snapshot = dict(preconditions.topology_snapshot or completion.topology_snapshot or {})
        frontier_snapshot = dict(preconditions.frontier_snapshot or completion.frontier_snapshot or {})
        support_snapshot = dict(preconditions.support_snapshot or completion.support_snapshot or {})
        if completion.done:
            status = "done"
        elif not preconditions.passed:
            status = "blocked"
        notes = list(preconditions.notes) + list(completion.notes)
        diagnostics = self.merge_diagnostics(
            relation_state.diagnostics,
            preconditions.diagnostics,
            completion.diagnostics,
            topology_snapshot,
            frontier_snapshot,
            support_snapshot,
            {
                "skill_key": spec.skill_key,
                "active_module": spec.active_module,
                "passive_module": spec.passive_module,
                "relation_type": spec.relation_type,
                "status": status,
                "blocked_by_topology": blocked_by_topology,
                "not_fully_supported": not_fully_supported,
                "block_reason": block_reason,
                "used_context_topology": bool(preconditions.used_context_topology or completion.used_context_topology),
                "context_dt": resolved_context.dt,
                "context_target_ns": resolved_context.target_ns,
            },
        )
        return SkillExecutionResult(
            skill_spec=spec,
            relation_state=relation_state,
            skill_key=spec.skill_key,
            active_module=spec.active_module,
            passive_module=spec.passive_module,
            module_states=dict(module_states or {}),
            preconditions=preconditions,
            completion=completion,
            primitive_references=primitive_references,
            status=status,
            blocked_by_topology=blocked_by_topology,
            not_fully_supported=not_fully_supported,
            block_reason=block_reason,
            used_context_topology=bool(preconditions.used_context_topology or completion.used_context_topology),
            topology_snapshot=topology_snapshot,
            frontier_snapshot=frontier_snapshot,
            support_snapshot=support_snapshot,
            diagnostics=diagnostics,
            notes=notes,
        )

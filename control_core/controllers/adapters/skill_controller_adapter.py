"""Thin adapter that binds legacy estimates or runtime frames to relation skills."""

from __future__ import annotations

from typing import Any

from ...models.execution_context import ExecutionContext
from ...models.skill_types import (
    SkillCheckResult,
    SkillCompletionResult,
    SkillExecutionResult,
    SkillResolutionResult,
    SkillSpec,
)
from ...orchestration.skill_registry import (
    SkillRegistry,
    build_default_skill_registry,
)
from ...skills.coarse_approach import CoarseApproachSkill
from ...skills.fine_dock import FineDockSkill
from ...topology.chain_topology import ChainTopology
from ...topology.pair_registry import PairExtractorRegistry
from ...topology.relation_state import RelationState
from ....runtime_integration.observation_types import RuntimeObservationFrame, pair_key
from ....runtime_integration.runtime_state_builder import (
    RUNTIME_STATE_BUILDER_SOURCE,
    build_module_state_for_module,
    build_module_state_map,
    build_relation_state_from_frame,
)
from .legacy_extractors import (
    build_default_pair_extractor_registry,
    extract_module_state,
    extract_relation_state_via_registry,
)


def _default_phase3_topology() -> ChainTopology:
    """Return the default chain ordering used by the scaffold."""
    return ChainTopology(
        ordered_modules=["tip", "joint1", "joint2", "joint3", "joint4", "joint5"]
    )


class SkillControllerAdapter:
    """Translate legacy estimates or runtime frames into structured skill execution."""

    def __init__(
        self,
        *,
        coarse_skill: CoarseApproachSkill | None = None,
        fine_skill: FineDockSkill | None = None,
        topology: ChainTopology | None = None,
        pair_registry: PairExtractorRegistry | None = None,
        skill_registry: SkillRegistry | None = None,
    ) -> None:
        self._topology_defaulted = topology is None
        self.topology = topology or _default_phase3_topology()
        self.pair_registry = pair_registry or build_default_pair_extractor_registry()
        self.skill_registry = skill_registry or build_default_skill_registry(
            topology=self.topology,
            coarse_skill=coarse_skill,
            fine_skill=fine_skill,
        )
        if coarse_skill is not None:
            self.skill_registry.register(
                "coarse_approach",
                coarse_skill,
                metadata={"adapter_override": True},
                override=True,
            )
        if fine_skill is not None:
            self.skill_registry.register(
                "fine_dock",
                fine_skill,
                metadata={"adapter_override": True},
                override=True,
            )
        self._bind_topology_to_registered_skills()
        self.coarse_skill = self.skill_registry.get("coarse_approach")
        self.fine_skill = self.skill_registry.get("fine_dock")

    def execute_skill(
        self,
        spec: SkillSpec,
        estimate: Any,
        *,
        topology: ChainTopology | None = None,
        context: ExecutionContext | None = None,
        skill_registry: SkillRegistry | None = None,
        pair_registry: PairExtractorRegistry | None = None,
    ) -> SkillExecutionResult:
        """Dispatch the requested skill against a legacy-like estimate."""
        resolved_skill_registry = skill_registry or self.skill_registry
        resolved_pair_registry = pair_registry or self.pair_registry
        skill_resolution = resolved_skill_registry.resolve(spec.skill_key)
        execution_context = self._build_execution_context(spec, topology=topology, context=context).merged_metadata(
            {
                "input_source": "legacy_estimate",
                "pair_key": pair_key(spec.active_module, spec.passive_module),
                "skill_registry_source": skill_resolution.registry_source,
                "pair_registry_source": getattr(resolved_pair_registry, "source_name", None),
                "resolved_skill_key": skill_resolution.resolved_skill_key or spec.skill_key,
                "state_builder_source": "legacy_extractors",
            }
        ).with_updates(
            input_source="legacy_estimate"
        )
        module_states = {
            spec.active_module: extract_module_state(estimate, spec.active_module),
            spec.passive_module: extract_module_state(estimate, spec.passive_module),
        }
        try:
            relation_state = self._extract_relation_state(spec, estimate, registry=resolved_pair_registry)
        except LookupError as exc:
            reason = str(exc)
            relation_state = self._placeholder_relation_state(spec, reason)
            return self._build_blocked_result(
                spec,
                relation_state,
                module_states,
                context=execution_context,
                reason=reason,
                note="no registered relation extractor",
                skill_resolution=skill_resolution,
            )
        return self._dispatch_resolved_skill(
            spec,
            relation_state,
            module_states,
            execution_context=execution_context,
            skill_resolution=skill_resolution,
        )

    def execute_skill_from_frame(
        self,
        spec: SkillSpec,
        frame: RuntimeObservationFrame,
        *,
        topology: ChainTopology | None = None,
        context: ExecutionContext | None = None,
        skill_registry: SkillRegistry | None = None,
        pair_registry: PairExtractorRegistry | None = None,
    ) -> SkillExecutionResult:
        """Dispatch the requested skill against a runtime-facing observation frame."""
        del pair_registry
        resolved_skill_registry = skill_registry or self.skill_registry
        skill_resolution = resolved_skill_registry.resolve(spec.skill_key)
        execution_context = self._build_execution_context(spec, topology=topology, context=context).merged_metadata(
            {
                "input_source": "runtime_frame",
                "pair_key": pair_key(spec.active_module, spec.passive_module),
                "skill_registry_source": skill_resolution.registry_source,
                "pair_registry_source": None,
                "resolved_skill_key": skill_resolution.resolved_skill_key or spec.skill_key,
                "state_builder_source": RUNTIME_STATE_BUILDER_SOURCE,
            }
        ).with_updates(
            input_source="runtime_frame",
            frame_timestamp_ns=frame.timestamp_ns,
        )
        module_states = build_module_state_map(frame)
        module_states.setdefault(
            spec.active_module,
            build_module_state_for_module(frame, spec.active_module),
        )
        module_states.setdefault(
            spec.passive_module,
            build_module_state_for_module(frame, spec.passive_module),
        )
        relation_state = build_relation_state_from_frame(
            frame,
            active_module=spec.active_module,
            passive_module=spec.passive_module,
            relation_type=spec.relation_type,
        )
        return self._dispatch_resolved_skill(
            spec,
            relation_state,
            module_states,
            execution_context=execution_context,
            skill_resolution=skill_resolution,
        )

    def execute(
        self,
        spec: SkillSpec,
        estimate: Any,
        *,
        topology: ChainTopology | None = None,
        context: ExecutionContext | None = None,
        skill_registry: SkillRegistry | None = None,
        pair_registry: PairExtractorRegistry | None = None,
    ) -> SkillExecutionResult:
        """Backward-compatible wrapper for direct adapter callers."""
        return self.execute_skill(
            spec,
            estimate,
            topology=topology,
            context=context,
            skill_registry=skill_registry,
            pair_registry=pair_registry,
        )

    def _bind_topology_to_registered_skills(self) -> None:
        """Backfill adapter topology into registered skills that do not own one."""
        for descriptor in self.skill_registry.list_registered():
            skill_instance = self.skill_registry.get(descriptor.skill_key)
            if skill_instance is None:
                continue
            if getattr(skill_instance, "topology", None) is None:
                try:
                    setattr(skill_instance, "topology", self.topology)
                except AttributeError:
                    continue

    def _build_execution_context(
        self,
        spec: SkillSpec,
        *,
        topology: ChainTopology | None,
        context: ExecutionContext | None,
    ) -> ExecutionContext:
        """Build the explicit execution context passed into skill execution."""
        metadata = dict(context.metadata) if context is not None else {}
        target_ns = context.target_ns if context is not None else None
        if target_ns is None:
            target_ns_value = spec.get_param("target_ns", spec.metadata.get("target_ns"))
            if isinstance(target_ns_value, (int, float)):
                target_ns = int(target_ns_value)
            elif isinstance(target_ns_value, str) and target_ns_value.strip().isdigit():
                target_ns = int(target_ns_value.strip())
        if topology is not None:
            resolved_topology = topology
            topology_source = "execute_override"
            topology_source_public = "external_context"
        elif context is not None and context.topology is not None:
            resolved_topology = context.topology
            topology_source = "external_context"
            topology_source_public = "external_context"
        else:
            resolved_topology = self.topology
            topology_source = "adapter_default" if self._topology_defaulted else "adapter_constructor"
            topology_source_public = "default" if self._topology_defaulted else "adapter_internal"
        dt = context.dt if context is not None else None
        if dt is None:
            dt = spec.float_param("solver_dt", spec.solver_dt)
        metadata.setdefault("adapter_topology_source", topology_source)
        metadata.setdefault("adapter_topology_source_public", topology_source_public)
        metadata.setdefault("adapter_topology_defaulted", topology_source == "adapter_default")
        return ExecutionContext(
            topology=resolved_topology,
            dt=dt,
            target_ns=target_ns,
            metadata=metadata,
            input_source=None if context is None else context.input_source,
            frame_timestamp_ns=None if context is None else context.frame_timestamp_ns,
            topology_source=topology_source,
        )

    def _dispatch_resolved_skill(
        self,
        spec: SkillSpec,
        relation_state: RelationState,
        module_states: dict[str, Any],
        *,
        execution_context: ExecutionContext,
        skill_resolution: SkillResolutionResult,
    ) -> SkillExecutionResult:
        """Execute one resolved skill or produce a structured blocked result."""
        if not skill_resolution.found or skill_resolution.skill_instance is None:
            reason = skill_resolution.error or f"skill_not_registered:{spec.skill_key}"
            return self._build_blocked_result(
                spec,
                relation_state,
                module_states,
                context=execution_context,
                reason=reason,
                note="unsupported skill requested",
                skill_resolution=skill_resolution,
            )
        skill = skill_resolution.skill_instance
        if not hasattr(skill, "execute"):
            reason = f"invalid_skill_registration:{spec.skill_key}"
            return self._build_blocked_result(
                spec,
                relation_state,
                module_states,
                context=execution_context,
                reason=reason,
                note="registered skill does not implement execute(...)",
                skill_resolution=skill_resolution,
            )
        result = skill.execute(
            relation_state,
            spec,
            module_states=module_states,
            context=execution_context,
        )
        self._augment_result_diagnostics(
            result,
            execution_context,
            skill_resolution=skill_resolution,
        )
        return result

    def _augment_result_diagnostics(
        self,
        result: SkillExecutionResult,
        context: ExecutionContext,
        *,
        skill_resolution: SkillResolutionResult,
    ) -> None:
        """Attach adapter-level diagnostics after skill execution."""
        context_step_index = context.metadata.get("scheduler_step_index")
        if isinstance(context_step_index, int):
            context_step_index = float(context_step_index)
        result.diagnostics.update(
            {
                "resolved_skill_key": skill_resolution.resolved_skill_key or result.skill_key,
                "resolved_pair_extractor_key": result.relation_state.diagnostics.get("pair_extractor_key"),
                "input_source": self._diagnostic_value(context.input_source or context.metadata.get("input_source")),
                "pair_key": self._diagnostic_value(
                    context.metadata.get("pair_key") or f"{result.active_module}->{result.passive_module}"
                ),
                "topology_source": context.metadata.get("adapter_topology_source_public"),
                "context_topology_source": context.metadata.get("adapter_topology_source"),
                "context_topology_defaulted": context.metadata.get("adapter_topology_defaulted"),
                "context_target_ns": context.target_ns,
                "context_dt": context.dt,
                "context_frame_timestamp_ns": self._diagnostic_value(context.frame_timestamp_ns),
                "context_graph_id": self._diagnostic_value(context.metadata.get("graph_id")),
                "context_node_id": self._diagnostic_value(context.metadata.get("node_id")),
                "context_scheduler_step_index": self._diagnostic_value(context_step_index),
                "context_active_skill_key": self._diagnostic_value(context.metadata.get("active_skill_key")),
                "context_node_retry_limit": self._diagnostic_value(context.metadata.get("node_retry_limit")),
                "context_node_max_attempts": self._diagnostic_value(context.metadata.get("node_max_attempts")),
                "skill_registry_source": self._diagnostic_value(skill_resolution.registry_source),
                "pair_registry_source": self._diagnostic_value(context.metadata.get("pair_registry_source")),
                "state_builder_source": self._diagnostic_value(
                    context.metadata.get("state_builder_source")
                ),
                "graph_transition_policy_key": self._diagnostic_value(
                    context.metadata.get("node_transition_policy_key")
                    or context.metadata.get("graph_transition_policy_key")
                ),
                "resolved_transition_policy_key": self._diagnostic_value(
                    context.metadata.get("resolved_transition_policy_key")
                ),
                "transition_policy_source": self._diagnostic_value(
                    context.metadata.get("transition_policy_source")
                ),
                "geometry_observation_kind": self._diagnostic_value(
                    result.relation_state.diagnostics.get("geometry_observation_kind")
                ),
                "geometry_source_schema": self._diagnostic_value(
                    result.relation_state.diagnostics.get("geometry_source_schema")
                ),
                "result_pair": f"{result.active_module}->{result.passive_module}",
                "selected_primitives": ",".join(result.selected_primitives) or None,
            }
        )
        if result.blocked_by_topology:
            result.diagnostics["adapter_topology_block_reason"] = result.block_reason

    def _extract_relation_state(
        self,
        spec: SkillSpec,
        estimate: Any,
        *,
        registry: PairExtractorRegistry,
    ) -> RelationState:
        """Resolve the requested relation extractor through the pair registry."""
        return extract_relation_state_via_registry(
            estimate,
            active_module=spec.active_module,
            passive_module=spec.passive_module,
            relation_type=spec.relation_type,
            registry=registry,
        )

    def _placeholder_relation_state(self, spec: SkillSpec, reason: str) -> RelationState:
        """Build a placeholder relation state when the adapter cannot proceed."""
        return RelationState(
            active_module=spec.active_module,
            passive_module=spec.passive_module,
            relation_type=spec.relation_type,
            distance_mm=None,
            orientation_error_deg=None,
            coupled=None,
            observation_valid=False,
            diagnostics={
                "adapter_error": reason,
                "requested_pair": f"{spec.active_module}->{spec.passive_module}",
                "requested_relation_type": spec.relation_type,
            },
        )

    def _build_blocked_result(
        self,
        spec: SkillSpec,
        relation_state: RelationState,
        module_states: dict[str, Any],
        *,
        context: ExecutionContext,
        reason: str,
        note: str,
        skill_resolution: SkillResolutionResult,
    ) -> SkillExecutionResult:
        """Build a structured blocked result for adapter-level failures."""
        topology_snapshot = context.topology.snapshot() if context.topology is not None else {}
        frontier_snapshot = context.topology.frontier_snapshot() if context.topology is not None else {}
        support_snapshot = context.topology.support_snapshot() if context.topology is not None else {}
        context_step_index = context.metadata.get("scheduler_step_index")
        if isinstance(context_step_index, int):
            context_step_index = float(context_step_index)
        diagnostics = {
            **relation_state.diagnostics,
            **topology_snapshot,
            **frontier_snapshot,
            **support_snapshot,
            "adapter_error": reason,
            "skill_registry_error": skill_resolution.error,
            "resolved_skill_key": skill_resolution.resolved_skill_key or spec.skill_key,
            "status": "blocked",
            "blocked_by_topology": False,
            "block_reason": reason,
            "resolved_pair_extractor_key": relation_state.diagnostics.get("pair_extractor_key"),
            "input_source": self._diagnostic_value(context.input_source or context.metadata.get("input_source")),
            "pair_key": self._diagnostic_value(
                context.metadata.get("pair_key") or f"{spec.active_module}->{spec.passive_module}"
            ),
            "topology_source": context.metadata.get("adapter_topology_source_public"),
            "context_topology_source": context.metadata.get("adapter_topology_source"),
            "context_topology_defaulted": context.metadata.get("adapter_topology_defaulted"),
            "context_target_ns": context.target_ns,
            "context_dt": context.dt,
            "context_frame_timestamp_ns": self._diagnostic_value(context.frame_timestamp_ns),
            "context_graph_id": self._diagnostic_value(context.metadata.get("graph_id")),
            "context_node_id": self._diagnostic_value(context.metadata.get("node_id")),
            "context_scheduler_step_index": self._diagnostic_value(context_step_index),
            "context_active_skill_key": self._diagnostic_value(context.metadata.get("active_skill_key")),
            "context_node_retry_limit": self._diagnostic_value(context.metadata.get("node_retry_limit")),
            "context_node_max_attempts": self._diagnostic_value(context.metadata.get("node_max_attempts")),
            "skill_registry_source": self._diagnostic_value(skill_resolution.registry_source),
            "pair_registry_source": self._diagnostic_value(context.metadata.get("pair_registry_source")),
            "state_builder_source": self._diagnostic_value(context.metadata.get("state_builder_source")),
            "graph_transition_policy_key": self._diagnostic_value(
                context.metadata.get("node_transition_policy_key")
                or context.metadata.get("graph_transition_policy_key")
            ),
            "resolved_transition_policy_key": self._diagnostic_value(
                context.metadata.get("resolved_transition_policy_key")
            ),
            "transition_policy_source": self._diagnostic_value(
                context.metadata.get("transition_policy_source")
            ),
            "geometry_observation_kind": self._diagnostic_value(
                relation_state.diagnostics.get("geometry_observation_kind")
            ),
            "geometry_source_schema": self._diagnostic_value(
                relation_state.diagnostics.get("geometry_source_schema")
            ),
            "result_pair": f"{spec.active_module}->{spec.passive_module}",
            "selected_primitives": None,
        }
        return SkillExecutionResult(
            skill_spec=spec,
            relation_state=relation_state,
            skill_key=spec.skill_key,
            active_module=spec.active_module,
            passive_module=spec.passive_module,
            module_states=module_states,
            preconditions=SkillCheckResult(
                passed=False,
                blocking_reason=reason,
                block_reason=reason,
                used_context_topology=context.topology is not None,
                topology_snapshot=topology_snapshot,
                frontier_snapshot=frontier_snapshot,
                support_snapshot=support_snapshot,
                diagnostics={"adapter_error": reason},
                notes=[note],
            ),
            completion=SkillCompletionResult(
                done=False,
                block_reason=reason,
                used_context_topology=context.topology is not None,
                topology_snapshot=topology_snapshot,
                frontier_snapshot=frontier_snapshot,
                support_snapshot=support_snapshot,
            ),
            primitive_references=[],
            status="blocked",
            blocked_by_topology=False,
            block_reason=reason,
            used_context_topology=context.topology is not None,
            topology_snapshot=topology_snapshot,
            frontier_snapshot=frontier_snapshot,
            support_snapshot=support_snapshot,
            diagnostics=diagnostics,
            notes=[note],
        )

    @staticmethod
    def _diagnostic_value(value: object) -> str | float | bool | None:
        """Normalize adapter diagnostics into the supported scalar set."""
        if value is None or isinstance(value, (str, float, bool)):
            return value
        if isinstance(value, int):
            return float(value)
        return str(value)

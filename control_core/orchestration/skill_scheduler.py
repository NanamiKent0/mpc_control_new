"""Skill scheduler for the Phase-6 task-graph orchestration scaffold."""

from __future__ import annotations

import inspect
from copy import deepcopy
from dataclasses import dataclass

from ..controllers.adapters.skill_controller_adapter import SkillControllerAdapter
from ...runtime_integration.observation_types import RuntimeObservationFrame
from ..models.execution_context import ExecutionContext
from ..models.runtime_bridge_types import SchedulerDispatchEnvelope
from ..models.geometry_observation import GeometryObservation
from ..models.skill_types import SkillExecutionResult, SkillResolutionResult
from ..models.task_types import SchedulerState, SchedulerStepResult, TaskGraphSpec, TaskNode
from ..topology.pair_registry import PairExtractorRegistry
from ..topology.relation_state import RelationState
from .skill_registry import SkillRegistry, build_default_skill_registry
from .task_graph import TaskGraph
from .transition_policy import TransitionDecision, TransitionPolicy
from .transition_policy_registry import (
    TransitionPolicyRegistry,
    TransitionPolicyResolutionResult,
    build_default_transition_policy_registry,
)


@dataclass(slots=True)
class _TransitionPolicySelection:
    """Internal policy-selection snapshot for one scheduler step."""

    requested_policy_key: str
    graph_policy_key: str | None
    source: str
    resolution: TransitionPolicyResolutionResult


class SkillScheduler:
    """Drive one loaded task graph through adapter-backed skill execution."""

    def __init__(
        self,
        *,
        adapter: SkillControllerAdapter | None = None,
        skill_registry: SkillRegistry | None = None,
        pair_registry: PairExtractorRegistry | None = None,
        transition_policy: TransitionPolicy | None = None,
        transition_policy_registry: TransitionPolicyRegistry | None = None,
    ) -> None:
        resolved_adapter = adapter
        if resolved_adapter is None:
            resolved_adapter = SkillControllerAdapter(
                skill_registry=skill_registry,
                pair_registry=pair_registry,
            )
        self.adapter = resolved_adapter
        self.skill_registry = (
            skill_registry
            or getattr(self.adapter, "skill_registry", None)
            or build_default_skill_registry()
        )
        self.pair_registry = pair_registry or getattr(self.adapter, "pair_registry", None)
        if hasattr(self.adapter, "skill_registry"):
            self.adapter.skill_registry = self.skill_registry
        if self.pair_registry is not None and hasattr(self.adapter, "pair_registry"):
            self.adapter.pair_registry = self.pair_registry
        self.transition_policy_registry = (
            transition_policy_registry or build_default_transition_policy_registry()
        )
        if transition_policy is not None:
            self.transition_policy_registry.register(
                self.transition_policy_registry.default_policy_key,
                transition_policy,
                metadata={"scheduler_override": True},
                override=True,
            )
        self.task_graph: TaskGraph | None = None
        self.state = SchedulerState()
        self._step_index = 0

    def load_graph(self, graph_spec: TaskGraphSpec) -> SchedulerState:
        """Load and validate one task graph, then reset scheduler state."""
        self.task_graph = TaskGraph(graph_spec)
        return self.reset()

    def reset(self) -> SchedulerState:
        """Reset scheduler state back to the loaded graph start node."""
        if self.task_graph is None:
            raise ValueError("skill_scheduler_graph_not_loaded")
        self._step_index = 0
        self.state = SchedulerState(
            graph_id=self.task_graph.spec.graph_id,
            current_node_id=self.task_graph.spec.start_node_id,
            previous_node_id=None,
            completed_node_ids=[],
            node_attempt_counts={},
            is_finished=False,
            last_transition_reason="scheduler_reset",
            last_skill_key=None,
            last_failure_reason=None,
            last_completion_done=None,
            last_preconditions_ok=None,
            last_retry_scheduled=False,
            last_fallback_used=False,
            last_transitioned=False,
            last_resolved_transition_policy_key=None,
            last_transition_policy_source=None,
            last_transition_policy_error=None,
        )
        return self.state

    def step(
        self,
        estimate: object,
        *,
        context: ExecutionContext | None = None,
    ) -> SchedulerStepResult:
        """Execute the active node skill once and update scheduler state."""
        if self.task_graph is None:
            raise ValueError("skill_scheduler_graph_not_loaded")
        scheduler_input_source = self._scheduler_input_source(estimate)
        graph_metadata = dict(self.task_graph.spec.metadata)
        if self.state.is_finished or self.state.current_node_id is None:
            return SchedulerStepResult(
                scheduler_state=deepcopy(self.state),
                node=None,
                skill_result=None,
                transitioned=False,
                transition_reason="scheduler_finished",
                selected_next_node_id=None,
                retry_scheduled=False,
                fallback_used=False,
                transition_decision=None,
                resolved_transition_policy_key=self.state.last_resolved_transition_policy_key,
                transition_policy_source=self.state.last_transition_policy_source,
                transition_policy_error=self.state.last_transition_policy_error,
                execution_context_metadata={},
                diagnostics={
                    "graph_id": self.state.graph_id,
                    "current_node_id": self.state.current_node_id,
                    "transitioned": False,
                    "transition_reason": "scheduler_finished",
                    "selected_next_node_id": None,
                    "retry_scheduled": False,
                    "fallback_used": False,
                    "skill_key": None,
                    "pair": None,
                    "topology_used": False,
                    "scheduler_step_index": self._step_index,
                    "scheduler_input_source": scheduler_input_source,
                    "state_builder_source": None,
                    "legacy_path_used": scheduler_input_source == "legacy_estimate",
                    "context_topology_source": None,
                    "dispatch_ready": False,
                    "resolved_transition_policy_key": self.state.last_resolved_transition_policy_key,
                    "transition_policy_source": self.state.last_transition_policy_source,
                    **self._workflow_metadata(graph_metadata, {}),
                },
            )

        node = self.task_graph.get_node(self.state.current_node_id)
        skill_resolution = self.skill_registry.resolve(node.skill_spec.skill_key)
        current_attempt = self.state.attempt_count_for(node.node_id) + 1
        policy_selection = self._select_transition_policy(node)
        execution_context = self._build_context(
            node,
            context=context,
            skill_resolution=skill_resolution,
            policy_selection=policy_selection,
            attempt_count=current_attempt,
            scheduler_input_source=scheduler_input_source,
            estimate=estimate,
        )
        if not policy_selection.resolution.found or policy_selection.resolution.policy_instance is None:
            result = self._build_policy_resolution_error_result(
                node,
                policy_selection=policy_selection,
                execution_context=execution_context,
                attempt_count=current_attempt,
                scheduler_input_source=scheduler_input_source,
            )
            self._step_index += 1
            return result

        skill_result = self._execute_skill(
            node,
            estimate,
            execution_context,
        )
        decision = policy_selection.resolution.policy_instance.decide(
            self.task_graph,
            node,
            skill_result,
            attempt_count=current_attempt,
            context=execution_context,
            scheduler_state=self.state,
        )
        self._apply_decision(
            node,
            skill_result,
            decision,
            attempt_count=current_attempt,
            policy_selection=policy_selection,
        )
        diagnostics = self._step_diagnostics(
            node,
            skill_result,
            decision,
            attempt_count=current_attempt,
            skill_resolution=skill_resolution,
            policy_selection=policy_selection,
            execution_context=execution_context,
        )
        step_result = SchedulerStepResult(
            scheduler_state=deepcopy(self.state),
            node=node,
            skill_result=skill_result,
            transitioned=decision.transitioned,
            transition_reason=decision.reason,
            selected_next_node_id=decision.next_node_id,
            retry_scheduled=decision.retry_scheduled,
            fallback_used=decision.fallback_used,
            transition_decision=decision,
            resolved_transition_policy_key=policy_selection.resolution.resolved_policy_key,
            transition_policy_source=policy_selection.source,
            transition_policy_error=policy_selection.resolution.error,
            execution_context_metadata=execution_context.bridge_metadata(),
            diagnostics=diagnostics,
        )
        self._step_index += 1
        return step_result

    def to_dispatch_envelope(self, step_result: SchedulerStepResult) -> SchedulerDispatchEnvelope:
        """Convert one scheduler step result into the runtime bridge envelope."""
        return SchedulerDispatchEnvelope.from_step_result(step_result)

    def _select_transition_policy(self, node: TaskNode) -> _TransitionPolicySelection:
        """Resolve the transition policy for the current node."""
        graph_metadata = {} if self.task_graph is None else dict(self.task_graph.spec.metadata)
        graph_policy_key_value = graph_metadata.get("transition_policy_key") or graph_metadata.get(
            "default_transition_policy_key"
        )
        graph_policy_key = str(graph_policy_key_value) if graph_policy_key_value is not None else None
        if node.transition_policy_key is not None:
            requested_policy_key = node.transition_policy_key
            source = "node"
        elif graph_policy_key is not None:
            requested_policy_key = graph_policy_key
            source = "graph"
        else:
            requested_policy_key = self.transition_policy_registry.default_policy_key
            source = "scheduler_default"
        return _TransitionPolicySelection(
            requested_policy_key=requested_policy_key,
            graph_policy_key=graph_policy_key,
            source=source,
            resolution=self.transition_policy_registry.resolve(requested_policy_key),
        )

    def _build_context(
        self,
        node: TaskNode,
        *,
        context: ExecutionContext | None,
        skill_resolution: SkillResolutionResult,
        policy_selection: _TransitionPolicySelection,
        attempt_count: int,
        scheduler_input_source: str,
        estimate: object,
    ) -> ExecutionContext:
        """Return the execution context passed into the adapter for this step."""
        base_context = context or ExecutionContext()
        graph_metadata = {} if self.task_graph is None else dict(self.task_graph.spec.metadata)
        runtime_hints = self._contextual_execution_hints(graph_metadata, node.metadata)
        workflow_metadata = self._workflow_metadata(graph_metadata, node.metadata)
        frame_timestamp_ns = (
            estimate.timestamp_ns if isinstance(estimate, RuntimeObservationFrame) else base_context.frame_timestamp_ns
        )
        return base_context.merged_metadata(
            {
                **runtime_hints,
                **workflow_metadata,
                "graph_id": self.state.graph_id,
                "graph_metadata": graph_metadata,
                "node_id": node.node_id,
                "node_metadata": dict(node.metadata),
                "scheduler_step_index": self._step_index,
                "active_skill_key": node.skill_spec.skill_key,
                "requested_transition_policy_key": policy_selection.requested_policy_key,
                "node_transition_policy_key": policy_selection.requested_policy_key,
                "resolved_transition_policy_key": (
                    policy_selection.resolution.resolved_policy_key
                    or policy_selection.requested_policy_key
                ),
                "transition_policy_source": policy_selection.source,
                "transition_policy_registry_source": policy_selection.resolution.registry_source,
                "transition_policy_resolution_error": policy_selection.resolution.error,
                "graph_transition_policy_key": policy_selection.graph_policy_key,
                "node_max_attempts": node.max_attempts,
                "node_retry_limit": node.max_attempts,
                "node_retry_budget": node.retry_budget,
                "node_attempt_count": attempt_count,
                "skill_registry_source": skill_resolution.registry_source,
                "skill_resolution_found": skill_resolution.found,
                "resolved_skill_key": skill_resolution.resolved_skill_key or node.skill_spec.skill_key,
                "resolved_skill_metadata": skill_resolution.metadata,
                "scheduler_input_source": scheduler_input_source,
            }
        ).with_updates(
            input_source=base_context.input_source or scheduler_input_source,
            frame_timestamp_ns=frame_timestamp_ns,
        )

    def _execute_skill(
        self,
        node: TaskNode,
        estimate: object,
        execution_context: ExecutionContext,
    ) -> SkillExecutionResult | None:
        """Execute one node through the adapter while tolerating older test doubles."""
        if isinstance(estimate, RuntimeObservationFrame) and hasattr(self.adapter, "execute_skill_from_frame"):
            method = self.adapter.execute_skill_from_frame
        else:
            method = self.adapter.execute_skill
        parameters = inspect.signature(method).parameters
        kwargs: dict[str, object] = {"context": execution_context}
        if "skill_registry" in parameters:
            kwargs["skill_registry"] = self.skill_registry
        if "pair_registry" in parameters and self.pair_registry is not None:
            kwargs["pair_registry"] = self.pair_registry
        return method(node.skill_spec, estimate, **kwargs)

    def _apply_decision(
        self,
        node: TaskNode,
        skill_result: SkillExecutionResult | None,
        decision: TransitionDecision,
        *,
        attempt_count: int,
        policy_selection: _TransitionPolicySelection,
    ) -> None:
        """Apply one transition decision to the scheduler state."""
        if self.task_graph is not None:
            self.state.graph_id = self.task_graph.spec.graph_id
        self.state.last_skill_key = node.skill_spec.skill_key
        self.state.last_transition_reason = decision.reason
        self.state.node_attempt_counts[node.node_id] = attempt_count
        self.state.last_preconditions_ok = None if skill_result is None else skill_result.preconditions.ok
        self.state.last_completion_done = None if skill_result is None else skill_result.completion.done
        self.state.last_failure_reason = self._last_failure_reason(skill_result, decision)
        self.state.last_retry_scheduled = decision.retry_scheduled
        self.state.last_fallback_used = decision.fallback_used
        self.state.last_transitioned = decision.transitioned
        self.state.last_resolved_transition_policy_key = (
            policy_selection.resolution.resolved_policy_key or policy_selection.requested_policy_key
        )
        self.state.last_transition_policy_source = policy_selection.source
        self.state.last_transition_policy_error = policy_selection.resolution.error
        if not decision.transitioned:
            return
        self.state.previous_node_id = node.node_id
        if node.node_id not in self.state.completed_node_ids:
            self.state.completed_node_ids.append(node.node_id)
        self.state.current_node_id = None if decision.finished else decision.next_node_id
        self.state.is_finished = bool(decision.finished)

    def _build_policy_resolution_error_result(
        self,
        node: TaskNode,
        *,
        policy_selection: _TransitionPolicySelection,
        execution_context: ExecutionContext,
        attempt_count: int,
        scheduler_input_source: str,
    ) -> SchedulerStepResult:
        """Return a structured scheduler step result for policy-resolution errors."""
        error = policy_selection.resolution.error or (
            f"transition_policy_not_registered:{policy_selection.requested_policy_key}"
        )
        self.state.last_skill_key = node.skill_spec.skill_key
        self.state.last_transition_reason = error
        self.state.last_failure_reason = error
        self.state.last_completion_done = None
        self.state.last_preconditions_ok = None
        self.state.last_retry_scheduled = False
        self.state.last_fallback_used = False
        self.state.last_transitioned = False
        self.state.last_resolved_transition_policy_key = None
        self.state.last_transition_policy_source = policy_selection.source
        self.state.last_transition_policy_error = error
        return SchedulerStepResult(
            scheduler_state=deepcopy(self.state),
            node=node,
            skill_result=None,
            transitioned=False,
            transition_reason=error,
            selected_next_node_id=None,
            retry_scheduled=False,
            fallback_used=False,
            transition_decision=None,
            resolved_transition_policy_key=None,
            transition_policy_source=policy_selection.source,
            transition_policy_error=error,
            execution_context_metadata=execution_context.bridge_metadata(),
            diagnostics={
                "graph_id": self.state.graph_id,
                "executed_node_id": node.node_id,
                "current_node_id": self.state.current_node_id,
                "selected_next_node_id": None,
                "transitioned": False,
                "transition_reason": error,
                "retry_scheduled": False,
                "fallback_used": False,
                "skill_key": node.skill_spec.skill_key,
                "resolved_skill_key": node.skill_spec.skill_key,
                "skill_registry_source": self.skill_registry.source_name,
                "pair": f"{node.skill_spec.active_module}->{node.skill_spec.passive_module}",
                "topology_used": bool(execution_context.topology is not None),
                "scheduler_step_index": self._step_index,
                "scheduler_input_source": scheduler_input_source,
                "state_builder_source": execution_context.metadata.get("state_builder_source"),
                "legacy_path_used": scheduler_input_source == "legacy_estimate",
                "context_topology_source": execution_context.topology_source,
                "dispatch_ready": False,
                "attempt_count": attempt_count,
                "node_max_attempts": node.max_attempts,
                "node_retry_budget": node.retry_budget,
                "resolved_transition_policy_key": None,
                "transition_policy_source": policy_selection.source,
                "transition_policy_registry_source": policy_selection.resolution.registry_source,
                "transition_policy_error": error,
                **self._workflow_metadata({}, execution_context.metadata),
            },
        )

    @staticmethod
    def _last_failure_reason(
        skill_result: SkillExecutionResult | None,
        decision: TransitionDecision,
    ) -> str | None:
        """Return the scheduler-visible failure reason for the latest step."""
        if skill_result is None:
            return decision.reason
        if not skill_result.preconditions.ok:
            return decision.reason
        if decision.fallback_used:
            return decision.reason
        if decision.reason and decision.reason.startswith(
            ("forced_failure_after_n_attempts", "max_attempts_exhausted", "fail_fast:")
        ):
            return decision.reason
        return None

    def _step_diagnostics(
        self,
        node: TaskNode,
        skill_result: SkillExecutionResult | None,
        decision: TransitionDecision,
        *,
        attempt_count: int,
        skill_resolution: SkillResolutionResult,
        policy_selection: _TransitionPolicySelection,
        execution_context: ExecutionContext,
    ) -> dict[str, object]:
        """Return stable diagnostics for one scheduler step."""
        pair_label = f"{node.skill_spec.active_module}->{node.skill_spec.passive_module}"
        topology_used = bool(getattr(skill_result, "used_context_topology", False))
        result_diagnostics = getattr(skill_result, "diagnostics", {}) if skill_result else {}
        return {
            "graph_id": self.state.graph_id,
            "executed_node_id": node.node_id,
            "current_node_id": self.state.current_node_id,
            "selected_next_node_id": decision.next_node_id,
            "transitioned": decision.transitioned,
            "transition_reason": decision.reason,
            "retry_scheduled": decision.retry_scheduled,
            "fallback_used": decision.fallback_used,
            "skill_key": node.skill_spec.skill_key,
            "resolved_skill_key": skill_resolution.resolved_skill_key,
            "skill_registry_source": skill_resolution.registry_source,
            "pair": pair_label,
            "topology_used": topology_used,
            "topology_source": result_diagnostics.get("topology_source") if skill_result else None,
            "context_topology_source": result_diagnostics.get("context_topology_source")
            if skill_result
            else execution_context.topology_source,
            "scheduler_step_index": self._step_index,
            "scheduler_input_source": execution_context.input_source,
            "state_builder_source": result_diagnostics.get("state_builder_source")
            if skill_result
            else execution_context.metadata.get("state_builder_source"),
            "legacy_path_used": execution_context.input_source == "legacy_estimate",
            "dispatch_ready": skill_result is not None,
            "attempt_count": attempt_count,
            "node_max_attempts": node.max_attempts,
            "node_retry_limit": node.max_attempts,
            "node_retry_budget": node.retry_budget,
            "requested_transition_policy_key": policy_selection.requested_policy_key,
            "resolved_transition_policy_key": policy_selection.resolution.resolved_policy_key,
            "transition_policy_source": policy_selection.source,
            "transition_policy_registry_source": policy_selection.resolution.registry_source,
            "transition_policy_error": policy_selection.resolution.error,
            "graph_transition_policy_key": execution_context.metadata.get("graph_transition_policy_key"),
            "node_force_failure_after_n_attempts": node.metadata.get("force_failure_after_n_attempts"),
            "geometry_observation_kind": result_diagnostics.get("geometry_observation_kind"),
            "geometry_source_schema": result_diagnostics.get("geometry_source_schema"),
            "selected_primitives": list(getattr(skill_result, "selected_primitives", [])) if skill_result else [],
            **self._workflow_metadata({}, execution_context.metadata),
        }

    @staticmethod
    def _scheduler_input_source(estimate: object) -> str:
        """Return the stable scheduler input-source label for diagnostics."""
        if isinstance(estimate, RuntimeObservationFrame):
            return "runtime_frame"
        if isinstance(estimate, GeometryObservation):
            return "geometry_observation"
        if isinstance(estimate, RelationState):
            return "relation_state"
        return "legacy_estimate"

    @staticmethod
    def _contextual_execution_hints(
        graph_metadata: dict[str, object],
        node_metadata: dict[str, object],
    ) -> dict[str, object]:
        """Return runtime hints that should surface at the execution-context top level."""
        supported_keys = (
            "allow_off_frontier",
            "requires_support_stability",
            "allow_support_breaking",
            "allow_support_breaking_actions",
            "force_failure_after_n_attempts",
        )
        hints: dict[str, object] = {}
        for key in supported_keys:
            if key in graph_metadata:
                hints[key] = graph_metadata[key]
        for key in supported_keys:
            if key in node_metadata:
                hints[key] = node_metadata[key]
        return hints

    @staticmethod
    def _workflow_metadata(
        graph_metadata: dict[str, object],
        node_metadata: dict[str, object],
    ) -> dict[str, object]:
        """Return flattened turn-workflow diagnostics shared across scheduler surfaces."""
        keys = (
            "operator_intent",
            "operator_intent_kind",
            "high_level_task_kind",
            "target_heading_delta_deg",
            "tip_heading_current_deg",
            "tip_heading_target_deg",
            "selected_joint_id",
            "selected_joint_index",
            "direct_front_cooperation",
            "requires_recursive_transfer",
            "planner_mode",
            "current_plan_node_kind",
            "current_active_pair",
            "returning_to_tip_free_growth",
            "plan_kind",
        )
        workflow: dict[str, object] = {}
        for key in keys:
            if key in graph_metadata:
                workflow[key] = graph_metadata[key]
        for key in keys:
            if key in node_metadata:
                workflow[key] = node_metadata[key]
        return workflow

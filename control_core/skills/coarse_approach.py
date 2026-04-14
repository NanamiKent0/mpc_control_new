"""Reusable coarse approach skill for relation-level distance reduction."""

from __future__ import annotations

from ..models.execution_context import ExecutionContext
from ..models.skill_types import PrimitiveReference, SkillCheckResult, SkillCompletionResult, SkillSpec
from ..topology.chain_topology import ChainTopology
from ..topology.relation_state import RelationState
from ..mpc.solvers.scalar_distance_mpc import ScalarDistanceMPC
from .base import RelationSkill


class CoarseApproachSkill(RelationSkill):
    """Template for generic decoupled coarse approach relations."""

    skill_key = "coarse_approach"

    def __init__(
        self,
        solver: ScalarDistanceMPC | None = None,
        *,
        topology: ChainTopology | None = None,
    ) -> None:
        self.solver = solver or ScalarDistanceMPC()
        self.topology = topology

    def build_relation_state(
        self,
        source: object,
        spec: SkillSpec,
        context: ExecutionContext | None = None,
    ) -> RelationState:
        """Build or coerce a relation state for coarse approach execution."""
        del context
        return self._coerce_relation_state(source, spec)

    def _active_module_type(self, relation_state: RelationState, spec: SkillSpec) -> str:
        """Resolve the active module type from spec metadata or naming."""
        metadata_type = str(spec.metadata.get("active_module_type", "")).strip().lower()
        if metadata_type in {"joint", "tip"}:
            return metadata_type
        if relation_state.active_module == "tip":
            return "tip"
        if relation_state.active_module.startswith("joint"):
            return "joint"
        return "unknown"

    def _selected_primary_primitive(self, active_module_type: str) -> str | None:
        """Return the primary primitive family selected for this execution."""
        if active_module_type == "joint":
            return "joint_crawl"
        if active_module_type == "tip":
            return "tip_growth"
        return None

    def check_preconditions(
        self,
        relation_state: RelationState,
        spec: SkillSpec,
        context: ExecutionContext | None = None,
    ) -> SkillCheckResult:
        """Reject invalid entry when geometry is missing, coupled, or topology blocks the pair."""
        active_module_type = self._active_module_type(relation_state, spec)
        selected_primary_primitive = self._selected_primary_primitive(active_module_type)
        topology_gate = self._evaluate_topology_gate(relation_state, spec, context)
        diagnostics = {
            "current_distance_mm": relation_state.distance_mm,
            "distance_threshold_mm": spec.distance_done_mm,
            "distance_ref_mm": spec.float_param("distance_ref_mm", spec.distance_done_mm),
            "coupled": relation_state.coupled,
            "active_module_type": active_module_type,
            "selected_primary_primitive": selected_primary_primitive,
            "topology_pair_allowed": topology_gate["pair_allowed"],
            "topology_block_reason": topology_gate["block_reason"],
            "allow_off_frontier": topology_gate["allow_off_frontier"],
            "requires_support_stability": topology_gate["requires_support_stability"],
            "allow_support_breaking": topology_gate["allow_support_breaking"],
            "invalid_entry": False,
            "invalid_entry_reason": None,
        }
        notes: list[str] = []
        if selected_primary_primitive is None:
            reason = f"unsupported_active_module_type:{relation_state.active_module}"
            diagnostics["invalid_entry"] = True
            diagnostics["invalid_entry_reason"] = reason
            self.push_note(notes, "coarse approach does not know how to drive this active module")
            return SkillCheckResult(
                passed=False,
                blocking_reason=reason,
                block_reason=reason,
                used_context_topology=topology_gate["used_context_topology"],
                topology_snapshot=topology_gate["topology_snapshot"],
                frontier_snapshot=topology_gate["frontier_snapshot"],
                support_snapshot=topology_gate["support_snapshot"],
                diagnostics=diagnostics,
                notes=notes,
            )
        if not topology_gate["pair_allowed"]:
            reason = topology_gate["block_reason"] or "pair_blocked_by_topology"
            diagnostics["invalid_entry"] = True
            diagnostics["invalid_entry_reason"] = reason
            self.push_note(notes, "topology blocked the requested pair")
            return SkillCheckResult(
                passed=False,
                blocking_reason=reason,
                blocked_by_topology=True,
                block_reason=reason,
                used_context_topology=topology_gate["used_context_topology"],
                topology_snapshot=topology_gate["topology_snapshot"],
                frontier_snapshot=topology_gate["frontier_snapshot"],
                support_snapshot=topology_gate["support_snapshot"],
                diagnostics=diagnostics,
                notes=notes,
            )
        if not relation_state.observation_valid or relation_state.distance_mm is None:
            reason = f"{spec.passive_module}_{spec.active_module}_distance_unavailable"
            diagnostics["invalid_entry"] = True
            diagnostics["invalid_entry_reason"] = reason
            self.push_note(notes, "relation observation unavailable")
            return SkillCheckResult(
                passed=False,
                blocking_reason=reason,
                block_reason=reason,
                used_context_topology=topology_gate["used_context_topology"],
                topology_snapshot=topology_gate["topology_snapshot"],
                frontier_snapshot=topology_gate["frontier_snapshot"],
                support_snapshot=topology_gate["support_snapshot"],
                diagnostics=diagnostics,
                notes=notes,
            )
        if relation_state.coupled is True:
            reason = f"{spec.passive_module}_{spec.active_module}_still_coupled"
            diagnostics["invalid_entry"] = True
            diagnostics["invalid_entry_reason"] = reason
            self.push_note(notes, "relation is still coupled")
            return SkillCheckResult(
                passed=False,
                blocking_reason=reason,
                block_reason=reason,
                used_context_topology=topology_gate["used_context_topology"],
                topology_snapshot=topology_gate["topology_snapshot"],
                frontier_snapshot=topology_gate["frontier_snapshot"],
                support_snapshot=topology_gate["support_snapshot"],
                diagnostics=diagnostics,
                notes=notes,
            )
        return SkillCheckResult(
            passed=True,
            used_context_topology=topology_gate["used_context_topology"],
            topology_snapshot=topology_gate["topology_snapshot"],
            frontier_snapshot=topology_gate["frontier_snapshot"],
            support_snapshot=topology_gate["support_snapshot"],
            diagnostics=diagnostics,
            notes=notes,
        )

    def generate_primitive_references(
        self,
        relation_state: RelationState,
        spec: SkillSpec,
        context: ExecutionContext | None = None,
    ) -> list[PrimitiveReference]:
        """Produce a type-aware primary approach command and nominal joint holds."""
        active_module_type = self._active_module_type(relation_state, spec)
        selected_primary_primitive = self._selected_primary_primitive(active_module_type)
        if selected_primary_primitive is None:
            return []
        distance_mm = float(relation_state.distance_mm or 0.0)
        threshold_mm = float(spec.distance_done_mm)
        distance_ref_mm = float(spec.float_param("distance_ref_mm", spec.distance_done_mm) or spec.distance_done_mm)
        context_dt = spec.solver_dt if context is None or context.dt is None else float(context.dt)
        axis_name = "crawl" if active_module_type == "joint" else "growth"
        if distance_mm <= threshold_mm:
            primary_reference = PrimitiveReference(
                module_id=relation_state.active_module,
                primitive_name=selected_primary_primitive,
                axis=axis_name,
                reference_kind="velocity",
                reference_value=0.0,
                units="mm/s",
                primary=True,
                semantic="zero_hold",
                target_value=distance_ref_mm,
                metadata={
                    "reason": "within_distance_threshold",
                    "active_module_type": active_module_type,
                },
            )
        else:
            solver_output = self.solver.compute(
                current_distance_mm=distance_mm,
                distance_ref_mm=distance_ref_mm,
                dt=context_dt,
                limits=spec.limits,
                config=spec.config,
            )
            primary_reference = PrimitiveReference(
                module_id=relation_state.active_module,
                primitive_name=selected_primary_primitive,
                axis=axis_name,
                reference_kind="velocity",
                reference_value=solver_output.crawl_velocity_mm_s,
                units="mm/s",
                primary=True,
                semantic="reduce_distance",
                target_value=distance_ref_mm,
                metadata={
                    "active_module_type": active_module_type,
                    "distance_error_mm": solver_output.distance_error_mm,
                    "predicted_terminal_distance_mm": solver_output.predicted_terminal_distance_mm,
                    "solver_notes": ",".join(solver_output.notes) or None,
                },
            )
        if active_module_type == "tip":
            return [primary_reference]
        return [
            primary_reference,
            PrimitiveReference(
                module_id=relation_state.active_module,
                primitive_name="joint_rotate_hold",
                axis="rotate",
                reference_kind="velocity",
                reference_value=0.0,
                units="deg/s",
                primary=False,
                semantic="nominal_hold",
                target_value=0.0,
                metadata={"role": "secondary"},
            ),
            PrimitiveReference(
                module_id=relation_state.active_module,
                primitive_name="joint_bend_hold",
                axis="bend",
                reference_kind="velocity",
                reference_value=0.0,
                units="deg/s",
                primary=False,
                semantic="nominal_hold",
                target_value=0.0,
                metadata={"role": "secondary"},
            ),
        ]

    def check_completion(
        self,
        relation_state: RelationState,
        spec: SkillSpec,
        context: ExecutionContext | None = None,
    ) -> SkillCompletionResult:
        """Treat already-near decoupled relations as complete when topology allows the pair."""
        active_module_type = self._active_module_type(relation_state, spec)
        selected_primary_primitive = self._selected_primary_primitive(active_module_type)
        topology_gate = self._evaluate_topology_gate(relation_state, spec, context)
        done = (
            selected_primary_primitive is not None
            and relation_state.distance_mm is not None
            and float(relation_state.distance_mm) <= float(spec.distance_done_mm)
            and relation_state.coupled is False
            and topology_gate["pair_allowed"]
        )
        reason = "within_distance_threshold" if done else None
        notes: list[str] = []
        if done:
            self.push_note(notes, "already inside coarse approach entry band")
        elif not topology_gate["pair_allowed"]:
            self.push_note(notes, "completion blocked by topology")
        return SkillCompletionResult(
            done=done,
            completion_reason=reason,
            blocked_by_topology=not topology_gate["pair_allowed"],
            block_reason=topology_gate["block_reason"] if not topology_gate["pair_allowed"] else None,
            used_context_topology=topology_gate["used_context_topology"],
            topology_snapshot=topology_gate["topology_snapshot"],
            frontier_snapshot=topology_gate["frontier_snapshot"],
            support_snapshot=topology_gate["support_snapshot"],
            diagnostics={
                "current_distance_mm": relation_state.distance_mm,
                "distance_threshold_mm": spec.distance_done_mm,
                "distance_ref_mm": spec.float_param("distance_ref_mm", spec.distance_done_mm),
                "coupled": relation_state.coupled,
                "active_module_type": active_module_type,
                "selected_primary_primitive": selected_primary_primitive,
                "topology_pair_allowed": topology_gate["pair_allowed"],
                "topology_block_reason": topology_gate["block_reason"],
                "allow_off_frontier": topology_gate["allow_off_frontier"],
                "requires_support_stability": topology_gate["requires_support_stability"],
                "allow_support_breaking": topology_gate["allow_support_breaking"],
            },
            notes=notes,
        )

"""Reusable fine docking skill for relation-level distance/orientation closure."""

from __future__ import annotations

from ..models.execution_context import ExecutionContext
from ..models.skill_types import PrimitiveReference, SkillCheckResult, SkillCompletionResult, SkillSpec
from ..topology.chain_topology import ChainTopology
from ..topology.relation_state import RelationState
from ..mpc.solvers.pair_dock_mpc import PairDockMPC
from .base import RelationSkill


class FineDockSkill(RelationSkill):
    """Template for pair-generic fine docking with explicit axis support."""

    skill_key = "fine_dock"

    def __init__(
        self,
        solver: PairDockMPC | None = None,
        *,
        topology: ChainTopology | None = None,
    ) -> None:
        self.solver = solver or PairDockMPC()
        self.topology = topology

    def build_relation_state(
        self,
        source: object,
        spec: SkillSpec,
        context: ExecutionContext | None = None,
    ) -> RelationState:
        """Build or coerce a relation state for fine docking execution."""
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

    def _supported_axes(self, active_module_type: str) -> tuple[str, ...]:
        """Return the docking axes supported by the active module family."""
        if active_module_type == "joint":
            return ("distance", "orientation")
        if active_module_type == "tip":
            return ("distance",)
        return ()

    def _required_axes(self, spec: SkillSpec) -> tuple[str, ...]:
        """Return the requested docking axes for this execution."""
        configured = tuple(
            axis
            for axis in (axis.strip().lower() for axis in spec.list_param("required_axes"))
            if axis in {"distance", "orientation"}
        )
        if configured:
            return configured
        required_axes = ["distance"]
        if spec.float_param("orientation_done_deg", None) is not None or spec.float_param("orientation_ref_deg", None) is not None:
            required_axes.append("orientation")
        return tuple(required_axes)

    def _selected_primary_primitives(
        self,
        active_module_type: str,
        required_axes: tuple[str, ...],
    ) -> tuple[str, ...]:
        """Return the primary primitive families selected for supported requested axes."""
        primitives: list[str] = []
        if "distance" in required_axes:
            if active_module_type == "joint":
                primitives.append("joint_crawl")
            elif active_module_type == "tip":
                primitives.append("tip_growth")
        if "orientation" in required_axes and active_module_type == "joint":
            primitives.append("joint_rotate")
        return tuple(primitives)

    def _docking_targets(self, spec: SkillSpec) -> tuple[float, float, float, float]:
        """Return distance/orientation targets and completion tolerances."""
        distance_ref_mm = float(spec.float_param("distance_ref_mm", 0.0) or 0.0)
        orientation_ref_deg = float(spec.float_param("orientation_ref_deg", 0.0) or 0.0)
        distance_done_mm = float(spec.float_param("distance_done_mm", spec.distance_done_mm) or spec.distance_done_mm)
        orientation_done_deg = float(spec.float_param("orientation_done_deg", spec.orientation_done_deg or 0.0) or 0.0)
        return distance_ref_mm, orientation_ref_deg, distance_done_mm, orientation_done_deg

    def _diagnostics(
        self,
        relation_state: RelationState,
        spec: SkillSpec,
        topology_gate: dict[str, object],
    ) -> dict[str, object]:
        """Build shared diagnostics for fine docking checks and completion."""
        active_module_type = self._active_module_type(relation_state, spec)
        supported_axes = self._supported_axes(active_module_type)
        required_axes = self._required_axes(spec)
        unsupported_axes = tuple(axis for axis in required_axes if axis not in supported_axes)
        selected_primary_primitives = self._selected_primary_primitives(active_module_type, required_axes)
        distance_ref_mm, orientation_ref_deg, distance_done_mm, orientation_done_deg = self._docking_targets(spec)
        return {
            "current_distance_mm": relation_state.distance_mm,
            "current_orientation_error_deg": relation_state.orientation_error_deg,
            "distance_ref_mm": distance_ref_mm,
            "orientation_ref_deg": orientation_ref_deg,
            "distance_done_mm": distance_done_mm,
            "orientation_done_deg": orientation_done_deg,
            "coupled": relation_state.coupled,
            "active_module_type": active_module_type,
            "required_axes": ",".join(required_axes) or None,
            "supported_axes": ",".join(supported_axes) or None,
            "unsupported_axes": ",".join(unsupported_axes) or None,
            "selected_primary_primitives": ",".join(selected_primary_primitives) or None,
            "orientation_control_supported": "orientation" in supported_axes,
            "orientation_signal_source": relation_state.diagnostics.get("orientation_signal_source"),
            "topology_pair_allowed": topology_gate["pair_allowed"],
            "topology_block_reason": topology_gate["block_reason"],
            "allow_off_frontier": topology_gate["allow_off_frontier"],
            "requires_support_stability": topology_gate["requires_support_stability"],
            "allow_support_breaking": topology_gate["allow_support_breaking"],
        }

    def check_preconditions(
        self,
        relation_state: RelationState,
        spec: SkillSpec,
        context: ExecutionContext | None = None,
    ) -> SkillCheckResult:
        """Require legal topology and only the axes that the active module can actually control."""
        topology_gate = self._evaluate_topology_gate(relation_state, spec, context)
        diagnostics = self._diagnostics(relation_state, spec, topology_gate)
        active_module_type = self._active_module_type(relation_state, spec)
        supported_axes = self._supported_axes(active_module_type)
        required_axes = self._required_axes(spec)
        unsupported_axes = tuple(axis for axis in required_axes if axis not in supported_axes)
        notes: list[str] = []
        if not topology_gate["pair_allowed"]:
            reason = str(topology_gate["block_reason"] or "pair_blocked_by_topology")
            self.push_note(notes, "topology blocked the requested fine docking pair")
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
        if not supported_axes:
            reason = f"unsupported_active_module_type:{relation_state.active_module}"
            self.push_note(notes, "fine dock does not know how to drive this active module")
            return SkillCheckResult(
                passed=False,
                blocking_reason=reason,
                not_fully_supported=True,
                block_reason=reason,
                used_context_topology=topology_gate["used_context_topology"],
                topology_snapshot=topology_gate["topology_snapshot"],
                frontier_snapshot=topology_gate["frontier_snapshot"],
                support_snapshot=topology_gate["support_snapshot"],
                diagnostics=diagnostics,
                notes=notes,
            )
        if unsupported_axes:
            reason = f"unsupported_required_axes:{','.join(unsupported_axes)}"
            self.push_note(notes, "requested docking axes are not supported by the active module in Phase 3")
            return SkillCheckResult(
                passed=False,
                blocking_reason=reason,
                not_fully_supported=True,
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
            self.push_note(notes, "fine docking is waiting for distance geometry")
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
        if "orientation" in required_axes and relation_state.orientation_error_deg is None:
            reason = f"{spec.passive_module}_{spec.active_module}_orientation_unavailable"
            self.push_note(notes, "fine docking is waiting for orientation geometry")
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
        """Produce only the docking primitives that the active module actually supports."""
        active_module_type = self._active_module_type(relation_state, spec)
        required_axes = self._required_axes(spec)
        supported_axes = self._supported_axes(active_module_type)
        distance_ref_mm, orientation_ref_deg, distance_done_mm, orientation_done_deg = self._docking_targets(spec)
        selected_primary_primitives = self._selected_primary_primitives(active_module_type, required_axes)
        if not selected_primary_primitives:
            return []

        distance_mm = float(relation_state.distance_mm or 0.0)
        current_orientation_error_deg = relation_state.orientation_error_deg
        orientation_error_deg = float(current_orientation_error_deg or 0.0)
        distance_within_tolerance = abs(distance_mm - distance_ref_mm) <= distance_done_mm
        orientation_within_tolerance = (
            "orientation" not in required_axes
            or (
                current_orientation_error_deg is not None
                and abs(float(current_orientation_error_deg) - orientation_ref_deg) <= orientation_done_deg
            )
        )
        already_docked = relation_state.coupled is True or (distance_within_tolerance and orientation_within_tolerance)
        context_dt = spec.solver_dt if context is None or context.dt is None else float(context.dt)

        if active_module_type == "joint":
            solver_output = self.solver.compute(
                current_distance_mm=distance_mm,
                current_orientation_error_deg=orientation_error_deg,
                distance_ref_mm=distance_ref_mm,
                orientation_ref_deg=orientation_ref_deg,
                dt=context_dt,
                limits=spec.limits,
                config=spec.config,
            )
            references: list[PrimitiveReference] = []
            if "distance" in required_axes and "distance" in supported_axes:
                references.append(
                    PrimitiveReference(
                        module_id=relation_state.active_module,
                        primitive_name="joint_crawl",
                        axis="crawl",
                        reference_kind="velocity",
                        reference_value=0.0 if already_docked else solver_output.crawl_velocity_mm_s,
                        units="mm/s",
                        primary=True,
                        semantic="zero_hold" if already_docked else "reduce_distance",
                        target_value=distance_ref_mm,
                        metadata={
                            "distance_error_mm": solver_output.distance_error_mm,
                            "reason": "already_docked" if already_docked else None,
                            "selected_primary_primitives": ",".join(selected_primary_primitives) or None,
                            "solver_notes": ",".join(solver_output.notes) or None,
                        },
                    )
                )
            if "orientation" in required_axes and "orientation" in supported_axes:
                references.append(
                    PrimitiveReference(
                        module_id=relation_state.active_module,
                        primitive_name="joint_rotate",
                        axis="rotate",
                        reference_kind="velocity",
                        reference_value=0.0 if already_docked else solver_output.rotate_velocity_deg_s,
                        units="deg/s",
                        primary=True,
                        semantic="zero_hold" if already_docked else "reduce_orientation",
                        target_value=orientation_ref_deg,
                        metadata={
                            "orientation_error_deg": solver_output.orientation_error_deg,
                            "reason": "already_docked" if already_docked else None,
                            "selected_primary_primitives": ",".join(selected_primary_primitives) or None,
                            "solver_notes": ",".join(solver_output.notes) or None,
                        },
                    )
                )
            references.append(
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
                )
            )
            return references

        if active_module_type == "tip" and "distance" in required_axes and "distance" in supported_axes:
            if already_docked:
                reference_value = 0.0
                semantic = "zero_hold"
            else:
                distance_error_mm = distance_mm - distance_ref_mm
                gain = float(spec.config.get("distance_gain", spec.config.get("gain", 0.15)))
                growth_limit = abs(float(spec.limits.get("growth_mm_s", spec.limits.get("crawl_mm_s", 3.0))))
                deadband_mm = abs(float(spec.config.get("distance_deadband_mm", spec.config.get("deadband_mm", 0.5))))
                if distance_error_mm <= deadband_mm:
                    reference_value = 0.0
                else:
                    reference_value = min(growth_limit, max(0.0, gain * distance_error_mm))
                semantic = "reduce_distance"
            return [
                PrimitiveReference(
                    module_id=relation_state.active_module,
                    primitive_name="tip_growth",
                    axis="growth",
                    reference_kind="velocity",
                    reference_value=reference_value,
                    units="mm/s",
                    primary=True,
                    semantic=semantic,
                    target_value=distance_ref_mm,
                    metadata={
                        "reason": "already_docked" if already_docked else None,
                        "selected_primary_primitives": ",".join(selected_primary_primitives) or None,
                        "orientation_control_supported": False,
                    },
                )
            ]
        return []

    def check_completion(
        self,
        relation_state: RelationState,
        spec: SkillSpec,
        context: ExecutionContext | None = None,
    ) -> SkillCompletionResult:
        """Complete when the pair is legally coupled or inside the requested docking tolerances."""
        topology_gate = self._evaluate_topology_gate(relation_state, spec, context)
        diagnostics = self._diagnostics(relation_state, spec, topology_gate)
        active_module_type = self._active_module_type(relation_state, spec)
        supported_axes = self._supported_axes(active_module_type)
        required_axes = self._required_axes(spec)
        unsupported_axes = tuple(axis for axis in required_axes if axis not in supported_axes)
        distance_ref_mm, orientation_ref_deg, distance_done_mm, orientation_done_deg = self._docking_targets(spec)
        coupled_done = relation_state.coupled is True and topology_gate["pair_allowed"]
        distance_done = (
            relation_state.distance_mm is not None
            and abs(float(relation_state.distance_mm) - distance_ref_mm) <= distance_done_mm
        )
        orientation_required = "orientation" in required_axes
        orientation_done = (
            not orientation_required
            or (
                relation_state.orientation_error_deg is not None
                and abs(float(relation_state.orientation_error_deg) - orientation_ref_deg) <= orientation_done_deg
            )
        )
        tolerance_done = (
            not unsupported_axes
            and distance_done
            and orientation_done
            and topology_gate["pair_allowed"]
        )
        done = coupled_done or tolerance_done
        reason = "already_coupled" if coupled_done else "within_docking_tolerances" if tolerance_done else None
        notes: list[str] = []
        if coupled_done:
            self.push_note(notes, "relation already coupled")
        elif tolerance_done:
            self.push_note(notes, "relation already inside docking tolerances")
        elif not topology_gate["pair_allowed"]:
            self.push_note(notes, "completion blocked by topology")
        elif unsupported_axes:
            self.push_note(notes, "completion unavailable because required docking axes are unsupported")
        elif orientation_required and relation_state.orientation_error_deg is None:
            self.push_note(notes, "completion unavailable while orientation geometry is missing")
        return SkillCompletionResult(
            done=done,
            completion_reason=reason,
            blocked_by_topology=not topology_gate["pair_allowed"],
            block_reason=topology_gate["block_reason"] if not topology_gate["pair_allowed"] else None,
            used_context_topology=topology_gate["used_context_topology"],
            topology_snapshot=topology_gate["topology_snapshot"],
            frontier_snapshot=topology_gate["frontier_snapshot"],
            support_snapshot=topology_gate["support_snapshot"],
            diagnostics=diagnostics,
            notes=notes,
        )

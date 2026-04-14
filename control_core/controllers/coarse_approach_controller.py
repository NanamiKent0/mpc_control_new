"""Generic coarse-approach controller for pair skill families."""

from __future__ import annotations

from ..models.execution_context import ExecutionContext
from ..models.skill_types import PrimitiveReference, SkillSpec
from ..mpc.scalar_feed_mpc import ScalarFeedMPC
from ..topology.chain_topology import ChainTopology
from ..topology.relation_state import RelationState
from .pair_controller_support import PairControllerResult, PairControllerSupportMixin


class CoarseApproachController(PairControllerSupportMixin):
    """Build scalar approach references without hardcoding any joint index."""

    def __init__(
        self,
        solver: ScalarFeedMPC | None = None,
        *,
        topology: ChainTopology | None = None,
    ) -> None:
        self.solver = solver or ScalarFeedMPC()
        self.topology = topology

    def control(
        self,
        relation_state: RelationState,
        spec: SkillSpec,
        *,
        context: ExecutionContext | None = None,
    ) -> PairControllerResult:
        """Compute a generic pair approach action for the current relation."""
        controller_name = "coarse_approach_controller"
        selected_mpc = "scalar_feed_mpc"
        notes: list[str] = []
        topology_gate = self._evaluate_topology_gate(
            relation_state,
            spec,
            topology=self.topology,
            context=context,
        )
        active_module_type = self._active_module_type(relation_state, spec)
        if active_module_type == "joint":
            primitive_name = "joint_crawl"
            axis_name = "crawl"
            enabled_axes = ("crawl", "rotate", "bend")
        elif active_module_type == "tip":
            primitive_name = "tip_growth"
            axis_name = "growth"
            enabled_axes = ("growth",)
        else:
            return self._blocked_result(
                family="pair_generic",
                stage="coarse_approach",
                relation_state=relation_state,
                controller_name=controller_name,
                selected_mpc=selected_mpc,
                enabled_axes=(),
                diagnostics={"active_module_type": active_module_type},
                notes=notes,
                block_reason=f"unsupported_active_module_type:{relation_state.active_module}",
            )

        diagnostics = self._merge_diagnostics(
            {
                "active_module_type": active_module_type,
                "topology_pair_allowed": topology_gate["pair_allowed"],
                "topology_block_reason": topology_gate["block_reason"],
                "current_distance_mm": relation_state.distance_mm,
            },
            topology_gate["topology_snapshot"],
            topology_gate["frontier_snapshot"],
            topology_gate["support_snapshot"],
        )
        if not topology_gate["pair_allowed"]:
            self.push_note(notes, "topology blocked the requested pair")
            return self._blocked_result(
                family="pair_generic",
                stage="coarse_approach",
                relation_state=relation_state,
                controller_name=controller_name,
                selected_mpc=selected_mpc,
                enabled_axes=enabled_axes,
                diagnostics=diagnostics,
                notes=notes,
                block_reason=str(topology_gate["block_reason"] or "pair_blocked_by_topology"),
            )
        if relation_state.coupled is True:
            self.push_note(notes, "coarse approach only runs on decoupled pairs")
            return self._blocked_result(
                family="pair_generic",
                stage="coarse_approach",
                relation_state=relation_state,
                controller_name=controller_name,
                selected_mpc=selected_mpc,
                enabled_axes=enabled_axes,
                diagnostics=diagnostics,
                notes=notes,
                block_reason=f"{relation_state.passive_module}_{relation_state.active_module}_still_coupled",
            )
        if not relation_state.observation_valid or relation_state.distance_mm is None:
            self.push_note(notes, "distance geometry is required for coarse approach")
            return self._blocked_result(
                family="pair_generic",
                stage="coarse_approach",
                relation_state=relation_state,
                controller_name=controller_name,
                selected_mpc=selected_mpc,
                enabled_axes=enabled_axes,
                diagnostics=diagnostics,
                notes=notes,
                block_reason=f"{relation_state.passive_module}_{relation_state.active_module}_distance_unavailable",
            )

        distance_ref_mm = float(spec.float_param("distance_ref_mm", spec.distance_done_mm) or spec.distance_done_mm)
        dt = spec.solver_dt if context is None or context.dt is None else float(context.dt)
        solver_output = self.solver.compute(
            current_distance_mm=float(relation_state.distance_mm),
            distance_ref_mm=distance_ref_mm,
            dt=dt,
            limits=spec.limits,
            config=spec.config,
        )
        within_tolerance = abs(float(relation_state.distance_mm) - distance_ref_mm) <= float(spec.distance_done_mm)
        primary_reference = PrimitiveReference(
            module_id=relation_state.active_module,
            primitive_name=primitive_name,
            axis=axis_name,
            reference_kind="velocity",
            reference_value=0.0 if within_tolerance else solver_output.feed_velocity_mm_s,
            units="mm/s",
            primary=True,
            semantic="zero_hold" if within_tolerance else "reduce_distance",
            target_value=distance_ref_mm,
            metadata={
                "distance_error_mm": solver_output.distance_error_mm,
                "predicted_terminal_distance_mm": solver_output.predicted_terminal_distance_mm,
                "solver_notes": ",".join(solver_output.notes) or None,
            },
        )
        references = [primary_reference]
        if active_module_type == "joint":
            references.extend(self._joint_hold_references(relation_state.active_module))
        diagnostics.update(
            {
                "distance_ref_mm": distance_ref_mm,
                "selected_primitive": primitive_name,
                "enabled_axes": ",".join(enabled_axes),
            }
        )
        if within_tolerance:
            self.push_note(notes, "pair already inside coarse approach tolerance")
        return PairControllerResult(
            family="pair_generic",
            stage="coarse_approach",
            status="done" if within_tolerance else "active",
            active_module=relation_state.active_module,
            passive_module=relation_state.passive_module,
            relation_state=relation_state,
            selected_controller=controller_name,
            selected_mpc=selected_mpc,
            primitive_references=references,
            enabled_axes=enabled_axes,
            completion_reason="within_distance_tolerance" if within_tolerance else None,
            diagnostics=diagnostics,
            notes=notes,
        )

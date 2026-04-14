"""Generic fine-dock controller for pair skill families."""

from __future__ import annotations

from ..models.execution_context import ExecutionContext
from ..models.skill_types import PrimitiveReference, SkillSpec
from ..mpc.pair_dock_mpc import PairDockMPC
from ..topology.chain_topology import ChainTopology
from ..topology.relation_state import RelationState
from .pair_controller_support import PairControllerResult, PairControllerSupportMixin


class FineDockController(PairControllerSupportMixin):
    """Build fine docking references without hardcoding a specific pair mode."""

    def __init__(
        self,
        solver: PairDockMPC | None = None,
        *,
        topology: ChainTopology | None = None,
    ) -> None:
        self.solver = solver or PairDockMPC()
        self.topology = topology

    def control(
        self,
        relation_state: RelationState,
        spec: SkillSpec,
        *,
        context: ExecutionContext | None = None,
    ) -> PairControllerResult:
        """Compute generic fine docking references for the current pair."""
        controller_name = "fine_dock_controller"
        selected_mpc = "pair_dock_mpc"
        notes: list[str] = []
        topology_gate = self._evaluate_topology_gate(
            relation_state,
            spec,
            topology=self.topology,
            context=context,
        )
        active_module_type = self._active_module_type(relation_state, spec)
        if active_module_type == "joint":
            feed_primitive = "joint_crawl"
            feed_axis = "crawl"
            enabled_axes = ("crawl", "rotate", "bend")
            supported_axes = ("distance", "orientation")
        elif active_module_type == "tip":
            feed_primitive = "tip_growth"
            feed_axis = "growth"
            enabled_axes = ("growth",)
            supported_axes = ("distance",)
        else:
            return self._blocked_result(
                family="pair_generic",
                stage="fine_dock",
                relation_state=relation_state,
                controller_name=controller_name,
                selected_mpc=selected_mpc,
                enabled_axes=(),
                diagnostics={"active_module_type": active_module_type},
                notes=notes,
                block_reason=f"unsupported_active_module_type:{relation_state.active_module}",
            )

        configured_axes = tuple(
            axis
            for axis in (axis.strip().lower() for axis in spec.list_param("required_axes"))
            if axis in {"distance", "orientation"}
        )
        if configured_axes:
            required_axes = configured_axes
        elif active_module_type == "joint":
            required_axes = ("distance", "orientation")
        else:
            required_axes = ("distance",)
        unsupported_axes = tuple(axis for axis in required_axes if axis not in supported_axes)
        diagnostics = self._merge_diagnostics(
            {
                "active_module_type": active_module_type,
                "required_axes": ",".join(required_axes),
                "supported_axes": ",".join(supported_axes),
                "unsupported_axes": ",".join(unsupported_axes) or None,
                "current_distance_mm": relation_state.distance_mm,
                "current_orientation_error_deg": relation_state.orientation_error_deg,
                "topology_pair_allowed": topology_gate["pair_allowed"],
                "topology_block_reason": topology_gate["block_reason"],
            },
            topology_gate["topology_snapshot"],
            topology_gate["frontier_snapshot"],
            topology_gate["support_snapshot"],
        )
        if not topology_gate["pair_allowed"]:
            self.push_note(notes, "topology blocked the requested pair")
            return self._blocked_result(
                family="pair_generic",
                stage="fine_dock",
                relation_state=relation_state,
                controller_name=controller_name,
                selected_mpc=selected_mpc,
                enabled_axes=enabled_axes,
                diagnostics=diagnostics,
                notes=notes,
                block_reason=str(topology_gate["block_reason"] or "pair_blocked_by_topology"),
            )
        if unsupported_axes:
            self.push_note(notes, "requested docking axes are unsupported for the active module")
            return self._blocked_result(
                family="pair_generic",
                stage="fine_dock",
                relation_state=relation_state,
                controller_name=controller_name,
                selected_mpc=selected_mpc,
                enabled_axes=enabled_axes,
                diagnostics=diagnostics,
                notes=notes,
                block_reason=f"unsupported_required_axes:{','.join(unsupported_axes)}",
            )
        if not relation_state.observation_valid or relation_state.distance_mm is None:
            self.push_note(notes, "distance geometry is required for fine docking")
            return self._blocked_result(
                family="pair_generic",
                stage="fine_dock",
                relation_state=relation_state,
                controller_name=controller_name,
                selected_mpc=selected_mpc,
                enabled_axes=enabled_axes,
                diagnostics=diagnostics,
                notes=notes,
                block_reason=f"{relation_state.passive_module}_{relation_state.active_module}_distance_unavailable",
            )
        if "orientation" in required_axes and relation_state.orientation_error_deg is None:
            self.push_note(notes, "orientation geometry is required for fine docking")
            return self._blocked_result(
                family="pair_generic",
                stage="fine_dock",
                relation_state=relation_state,
                controller_name=controller_name,
                selected_mpc=selected_mpc,
                enabled_axes=enabled_axes,
                diagnostics=diagnostics,
                notes=notes,
                block_reason=f"{relation_state.passive_module}_{relation_state.active_module}_orientation_unavailable",
            )

        distance_ref_mm = float(spec.float_param("distance_ref_mm", 0.0) or 0.0)
        orientation_ref_deg = float(spec.float_param("orientation_ref_deg", 0.0) or 0.0)
        distance_done_mm = float(spec.float_param("distance_done_mm", spec.distance_done_mm) or spec.distance_done_mm)
        orientation_done_deg = float(spec.float_param("orientation_done_deg", spec.orientation_done_deg or 0.0) or 0.0)
        distance_done = abs(float(relation_state.distance_mm) - distance_ref_mm) <= distance_done_mm
        orientation_done = (
            "orientation" not in required_axes
            or (
                relation_state.orientation_error_deg is not None
                and abs(float(relation_state.orientation_error_deg) - orientation_ref_deg) <= orientation_done_deg
            )
        )
        already_docked = relation_state.coupled is True or (distance_done and orientation_done)
        dt = spec.solver_dt if context is None or context.dt is None else float(context.dt)

        if active_module_type == "tip":
            error_mm = float(relation_state.distance_mm) - distance_ref_mm
            growth_limit = abs(float(spec.limits.get("growth_mm_s", spec.limits.get("feed_mm_s", 3.0))))
            gain = float(spec.config.get("distance_gain", spec.config.get("gain", 0.15)))
            deadband_mm = abs(float(spec.config.get("distance_deadband_mm", spec.config.get("deadband_mm", 0.5))))
            feed_velocity = 0.0 if already_docked or error_mm <= deadband_mm else min(growth_limit, max(0.0, gain * error_mm))
            references = [
                PrimitiveReference(
                    module_id=relation_state.active_module,
                    primitive_name=feed_primitive,
                    axis=feed_axis,
                    reference_kind="velocity",
                    reference_value=feed_velocity,
                    units="mm/s",
                    primary=True,
                    semantic="zero_hold" if already_docked else "reduce_distance",
                    target_value=distance_ref_mm,
                    metadata={"orientation_control_supported": False},
                )
            ]
        else:
            solver_output = self.solver.compute(
                current_distance_mm=float(relation_state.distance_mm),
                current_orientation_error_deg=float(relation_state.orientation_error_deg or 0.0),
                distance_ref_mm=distance_ref_mm,
                orientation_ref_deg=orientation_ref_deg,
                dt=dt,
                limits=spec.limits,
                config=spec.config,
            )
            references = [
                PrimitiveReference(
                    module_id=relation_state.active_module,
                    primitive_name=feed_primitive,
                    axis=feed_axis,
                    reference_kind="velocity",
                    reference_value=0.0 if already_docked else solver_output.feed_velocity_mm_s,
                    units="mm/s",
                    primary=True,
                    semantic="zero_hold" if already_docked else "reduce_distance",
                    target_value=distance_ref_mm,
                    metadata={"distance_error_mm": solver_output.distance_error_mm},
                )
            ]
            if "orientation" in required_axes:
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
                        metadata={"orientation_error_deg": solver_output.orientation_error_deg},
                    )
                )
            references.extend(
                self._joint_hold_references(
                    relation_state.active_module,
                    include_rotate="orientation" not in required_axes,
                    include_bend=True,
                )
            )

        diagnostics.update(
            {
                "distance_ref_mm": distance_ref_mm,
                "orientation_ref_deg": orientation_ref_deg,
                "distance_done_mm": distance_done_mm,
                "orientation_done_deg": orientation_done_deg,
                "enabled_axes": ",".join(enabled_axes),
            }
        )
        if already_docked:
            self.push_note(notes, "pair already inside docking tolerance or coupled")
        return PairControllerResult(
            family="pair_generic",
            stage="fine_dock",
            status="done" if already_docked else "active",
            active_module=relation_state.active_module,
            passive_module=relation_state.passive_module,
            relation_state=relation_state,
            selected_controller=controller_name,
            selected_mpc=selected_mpc,
            primitive_references=references,
            enabled_axes=enabled_axes,
            completion_reason="already_docked" if already_docked else None,
            diagnostics=diagnostics,
            notes=notes,
        )

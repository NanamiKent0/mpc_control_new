"""Generic front-cooperate controller for joint-to-tip pair families."""

from __future__ import annotations

from ..models.execution_context import ExecutionContext
from ..models.skill_types import PrimitiveReference, SkillSpec
from ..mpc.cooperate_mpc import CooperateMPC
from ..mpc.posture_adjust_mpc import PostureAdjustMPC
from ..topology.chain_topology import ChainTopology
from ..topology.relation_state import RelationState
from .coarse_approach_controller import CoarseApproachController
from .fine_dock_controller import FineDockController
from .local_transfer_controller import LocalTransferController
from .pair_controller_support import PairControllerResult, PairControllerSupportMixin
from .turn_reference_mapper import resolve_front_cooperate_heading_reference


class FrontCooperateController(PairControllerSupportMixin):
    """Compose generic controller/MPC templates into the front cooperation family."""

    def __init__(
        self,
        *,
        coarse_controller: CoarseApproachController | None = None,
        fine_controller: FineDockController | None = None,
        cooperate_mpc: CooperateMPC | None = None,
        posture_adjust_mpc: PostureAdjustMPC | None = None,
        topology: ChainTopology | None = None,
    ) -> None:
        self.topology = topology
        self.coarse_controller = coarse_controller or CoarseApproachController(topology=topology)
        self.fine_controller = fine_controller or FineDockController(topology=topology)
        self._local_helper = LocalTransferController(
            coarse_controller=self.coarse_controller,
            fine_controller=self.fine_controller,
            cooperate_mpc=cooperate_mpc,
            posture_adjust_mpc=posture_adjust_mpc,
            topology=topology,
        )

    @property
    def cooperate_mpc(self) -> CooperateMPC:
        """Expose the shared cooperate MPC used by this family."""
        return self._local_helper.cooperate_mpc

    @property
    def posture_adjust_mpc(self) -> PostureAdjustMPC:
        """Expose the shared posture-adjust MPC used by this family."""
        return self._local_helper.posture_adjust_mpc

    def control(
        self,
        relation_state: RelationState,
        spec: SkillSpec,
        *,
        context: ExecutionContext | None = None,
    ) -> PairControllerResult:
        """Route one joint-tip pair through release, adjust, approach, dock, cooperate."""
        if not relation_state.active_module.startswith("joint") or relation_state.passive_module != "tip":
            return self._blocked_result(
                family="front_cooperate",
                stage="invalid_pair",
                relation_state=relation_state,
                controller_name="front_cooperate_controller",
                selected_mpc=None,
                enabled_axes=(),
                diagnostics={},
                notes=[],
                block_reason="front_cooperate_requires_joint_tip_pair",
            )
        topology_gate = self._evaluate_topology_gate(
            relation_state,
            spec,
            topology=self.topology,
            context=context,
        )
        diagnostics = self._merge_diagnostics(
            {
                "skill_family": "front_cooperate",
                "topology_pair_allowed": topology_gate["pair_allowed"],
                "topology_block_reason": topology_gate["block_reason"],
            },
            topology_gate["topology_snapshot"],
            topology_gate["frontier_snapshot"],
            topology_gate["support_snapshot"],
        )
        if not topology_gate["pair_allowed"]:
            return self._blocked_result(
                family="front_cooperate",
                stage="invalid_pair",
                relation_state=relation_state,
                controller_name="front_cooperate_controller",
                selected_mpc=None,
                enabled_axes=("crawl", "rotate", "bend"),
                diagnostics=diagnostics,
                notes=["topology blocked front cooperation"],
                block_reason=str(topology_gate["block_reason"] or "pair_blocked_by_topology"),
            )

        release_complete = self._spec_or_diagnostic_bool(spec, relation_state, "release_complete", False)
        if not release_complete:
            return self._local_helper._posture_stage_result(
                family="front_cooperate",
                stage="joint1_release_adjust",
                relation_state=relation_state,
                spec=spec,
                context=context,
                orientation_ref_key="release_orientation_ref_deg",
                bend_ref_key="release_bend_ref_deg",
                orientation_done_key="release_orientation_done_deg",
                bend_done_key="release_bend_done_deg",
                completion_reason="release_aligned",
            )

        posture_complete = self._local_helper._resolve_posture_complete(
            relation_state,
            spec,
            orientation_done_key="posture_orientation_done_deg",
            bend_done_key="posture_bend_done_deg",
            explicit_flag="posture_complete",
        )
        if not posture_complete:
            return self._local_helper._posture_stage_result(
                family="front_cooperate",
                stage="joint1_adjust",
                relation_state=relation_state,
                spec=spec,
                context=context,
                orientation_ref_key="posture_orientation_ref_deg",
                bend_ref_key="posture_bend_ref_deg",
                orientation_done_key="posture_orientation_done_deg",
                bend_done_key="posture_bend_done_deg",
                completion_reason="posture_aligned",
            )

        dock_trigger_mm = float(
            spec.float_param(
                "dock_trigger_distance_mm",
                max(spec.distance_done_mm * 4.0, spec.distance_done_mm + 10.0),
            )
            or max(spec.distance_done_mm * 4.0, spec.distance_done_mm + 10.0)
        )
        if relation_state.coupled is not True and relation_state.distance_mm is not None and float(relation_state.distance_mm) > dock_trigger_mm:
            delegated = self.coarse_controller.control(relation_state, spec, context=context)
            return self._local_helper._wrap_delegate_result(
                delegated,
                family="front_cooperate",
                stage="joint1_approach_tip",
            )
        if relation_state.coupled is not True:
            delegated = self.fine_controller.control(relation_state, spec, context=context)
            return self._local_helper._wrap_delegate_result(
                delegated,
                family="front_cooperate",
                stage="joint1_dock_tip",
            )

        heading_reference = resolve_front_cooperate_heading_reference(
            relation_state,
            spec,
            context_metadata=None if context is None else context.metadata,
            rotate_reference_deg_s=0.0,
            bend_reference_deg_s=0.0,
        )
        cooperate_complete = self._heading_cooperate_complete(
            relation_state,
            spec,
            context=context,
            heading_reference=heading_reference,
        )
        if not cooperate_complete:
            if heading_reference is not None:
                return self._heading_cooperate_stage_result(
                    family="front_cooperate",
                    stage="joint1_cooperate_with_tip",
                    relation_state=relation_state,
                    spec=spec,
                    context=context,
                    returnable_to_free_growth=False,
                    heading_reference=heading_reference,
                    diagnostics=diagnostics,
                )
            return self._local_helper._cooperate_stage_result(
                family="front_cooperate",
                stage="joint1_cooperate_with_tip",
                relation_state=relation_state,
                spec=spec,
                context=context,
                returnable_to_free_growth=False,
            )

        return PairControllerResult(
            family="front_cooperate",
            stage="returnable_to_free_growth",
            status="done",
            active_module=relation_state.active_module,
            passive_module=relation_state.passive_module,
            relation_state=relation_state,
            selected_controller="front_cooperate_controller",
            selected_mpc=None,
            primitive_references=self._joint_zero_hold_references(relation_state.active_module),
            enabled_axes=("crawl", "rotate", "bend"),
            completion_reason="cooperate_complete",
            returnable_to_free_growth=True,
            diagnostics=self._merge_diagnostics(
                diagnostics,
                {
                    "returnable_to_free_growth": True,
                    **self._heading_completion_diagnostics(heading_reference),
                },
            ),
            notes=["front cooperation complete; caller may return to free growth"],
        )

    def _heading_cooperate_complete(
        self,
        relation_state: RelationState,
        spec: SkillSpec,
        *,
        context: ExecutionContext | None,
        heading_reference: object | None,
    ) -> bool:
        """Resolve whether front cooperation has closed the requested tip heading."""
        if heading_reference is None:
            return self._local_helper._cooperate_complete(relation_state, spec)
        heading_done_deg = abs(
            float(
                self._context_or_spec_float(
                    spec,
                    context,
                    "cooperate_heading_done_deg",
                    default=2.0,
                )
                or 2.0
            )
        )
        return bool(
            relation_state.coupled is True
            and abs(float(heading_reference.heading_error_deg)) <= heading_done_deg
        )

    def _heading_cooperate_stage_result(
        self,
        *,
        family: str,
        stage: str,
        relation_state: RelationState,
        spec: SkillSpec,
        context: ExecutionContext | None,
        returnable_to_free_growth: bool,
        heading_reference: object,
        diagnostics: dict[str, object],
    ) -> PairControllerResult:
        """Run the heading-driven front cooperation stage using tip heading targets."""
        dt = spec.solver_dt if context is None or context.dt is None else float(context.dt)
        bend_ref_deg = float(
            self._context_or_spec_float(
                spec,
                context,
                "cooperate_bend_ref_deg",
                default=float(heading_reference.joint1_bend_current_deg),
            )
            or float(heading_reference.joint1_bend_current_deg)
        )
        solver_output = self.cooperate_mpc.compute(
            feed_forward_mm_s=float(spec.float_param("cooperate_feed_velocity_mm_s", 2.0) or 2.0),
            current_distance_mm=float(self.safe_float(relation_state.distance_mm, 0.0) or 0.0),
            distance_ref_mm=float(spec.float_param("distance_ref_mm", 0.0) or 0.0),
            current_orientation_error_deg=float(heading_reference.tip_heading_current_deg),
            orientation_ref_deg=float(heading_reference.tip_heading_target_deg),
            current_bend_error_deg=float(heading_reference.joint1_bend_current_deg),
            bend_ref_deg=bend_ref_deg,
            dt=dt,
            limits=spec.limits,
            config=spec.config,
        )
        resolved_heading_reference = resolve_front_cooperate_heading_reference(
            relation_state,
            spec,
            context_metadata=None if context is None else context.metadata,
            rotate_reference_deg_s=solver_output.rotate_velocity_deg_s,
            bend_reference_deg_s=solver_output.bend_velocity_deg_s,
        )
        assert resolved_heading_reference is not None
        references = [
            PrimitiveReference(
                module_id=relation_state.active_module,
                primitive_name="joint_crawl",
                axis="crawl",
                reference_kind="velocity",
                reference_value=solver_output.feed_velocity_mm_s,
                units="mm/s",
                primary=True,
                semantic="cooperate_feed",
                target_value=float(spec.float_param("distance_ref_mm", 0.0) or 0.0),
                metadata={"distance_error_mm": solver_output.distance_error_mm},
            ),
            PrimitiveReference(
                module_id=relation_state.active_module,
                primitive_name="joint_rotate",
                axis="rotate",
                reference_kind="velocity",
                reference_value=solver_output.rotate_velocity_deg_s,
                units="deg/s",
                primary=True,
                semantic="track_tip_heading",
                target_value=float(resolved_heading_reference.tip_heading_target_deg),
                metadata={
                    "heading_error_deg": resolved_heading_reference.heading_error_deg,
                    "tip_heading_current_deg": resolved_heading_reference.tip_heading_current_deg,
                    "tip_heading_target_deg": resolved_heading_reference.tip_heading_target_deg,
                },
            ),
            PrimitiveReference(
                module_id=relation_state.active_module,
                primitive_name="joint_bend",
                axis="bend",
                reference_kind="velocity",
                reference_value=solver_output.bend_velocity_deg_s,
                units="deg/s",
                primary=True,
                semantic="cooperate_stabilize",
                target_value=bend_ref_deg,
                metadata={"bend_error_deg": solver_output.bend_error_deg},
            ),
        ]
        return PairControllerResult(
            family=family,
            stage=stage,
            status="active",
            active_module=relation_state.active_module,
            passive_module=relation_state.passive_module,
            relation_state=relation_state,
            selected_controller="front_cooperate_controller",
            selected_mpc="cooperate_mpc",
            primitive_references=references,
            enabled_axes=("crawl", "rotate", "bend"),
            returnable_to_free_growth=returnable_to_free_growth,
            diagnostics=self._merge_diagnostics(
                diagnostics,
                {
                    "cooperate_feed_velocity_mm_s": solver_output.feed_velocity_mm_s,
                    "cooperate_distance_error_mm": solver_output.distance_error_mm,
                    "cooperate_orientation_error_deg": solver_output.orientation_error_deg,
                    "cooperate_bend_error_deg": solver_output.bend_error_deg,
                    **self._heading_completion_diagnostics(resolved_heading_reference),
                },
            ),
            notes=list(solver_output.notes),
        )

    def _heading_completion_diagnostics(self, heading_reference: object | None) -> dict[str, float | None]:
        """Return stable diagnostics for heading-driven front cooperation."""
        if heading_reference is None:
            return {}
        return {
            "tip_heading_current_deg": float(heading_reference.tip_heading_current_deg),
            "tip_heading_target_deg": float(heading_reference.tip_heading_target_deg),
            "heading_error_deg": float(heading_reference.heading_error_deg),
            "joint1_heading_current_deg": (
                None
                if heading_reference.joint1_heading_current_deg is None
                else float(heading_reference.joint1_heading_current_deg)
            ),
            "joint1_bend_current_deg": float(heading_reference.joint1_bend_current_deg),
            "joint1_rotate_reference": float(heading_reference.joint1_rotate_reference),
            "joint1_bend_reference": float(heading_reference.joint1_bend_reference),
        }

    def _context_or_spec_float(
        self,
        spec: SkillSpec,
        context: ExecutionContext | None,
        name: str,
        *,
        default: float | None = None,
    ) -> float | None:
        """Resolve one float from context metadata first, then spec params/metadata."""
        if context is not None and name in context.metadata:
            value = self.safe_float(context.metadata.get(name), None)
            if value is not None:
                return value
        if name in spec.params:
            value = self.safe_float(spec.params.get(name), None)
            if value is not None:
                return value
        if name in spec.metadata:
            value = self.safe_float(spec.metadata.get(name), None)
            if value is not None:
                return value
        return default

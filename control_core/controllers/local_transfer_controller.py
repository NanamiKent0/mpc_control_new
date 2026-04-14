"""Generic local-transfer controller for adjacent joint-to-joint pairs."""

from __future__ import annotations

from ..models.execution_context import ExecutionContext
from ..models.skill_types import PrimitiveReference, SkillSpec
from ..mpc.cooperate_mpc import CooperateMPC
from ..mpc.posture_adjust_mpc import PostureAdjustMPC
from ..topology.chain_topology import ChainTopology
from ..topology.relation_state import RelationState
from .coarse_approach_controller import CoarseApproachController
from .fine_dock_controller import FineDockController
from .pair_controller_support import PairControllerResult, PairControllerSupportMixin


class LocalTransferController(PairControllerSupportMixin):
    """Compose generic controller/MPC templates into the local transfer family."""

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
        self.cooperate_mpc = cooperate_mpc or CooperateMPC()
        self.posture_adjust_mpc = posture_adjust_mpc or PostureAdjustMPC()

    def control(
        self,
        relation_state: RelationState,
        spec: SkillSpec,
        *,
        context: ExecutionContext | None = None,
    ) -> PairControllerResult:
        """Route one joint-joint pair through release, adjust, approach, dock, cooperate."""
        if not relation_state.active_module.startswith("joint") or not relation_state.passive_module.startswith("joint"):
            return self._blocked_result(
                family="local_transfer",
                stage="invalid_pair",
                relation_state=relation_state,
                controller_name="local_transfer_controller",
                selected_mpc=None,
                enabled_axes=(),
                diagnostics={},
                notes=[],
                block_reason="local_transfer_requires_joint_joint_pair",
            )
        topology_gate = self._evaluate_topology_gate(
            relation_state,
            spec,
            topology=self.topology,
            context=context,
        )
        diagnostics = self._merge_diagnostics(
            {
                "skill_family": "local_transfer",
                "topology_pair_allowed": topology_gate["pair_allowed"],
                "topology_block_reason": topology_gate["block_reason"],
            },
            topology_gate["topology_snapshot"],
            topology_gate["frontier_snapshot"],
            topology_gate["support_snapshot"],
        )
        if not topology_gate["pair_allowed"]:
            return self._blocked_result(
                family="local_transfer",
                stage="invalid_pair",
                relation_state=relation_state,
                controller_name="local_transfer_controller",
                selected_mpc=None,
                enabled_axes=("crawl", "rotate", "bend"),
                diagnostics=diagnostics,
                notes=["topology blocked local transfer"],
                block_reason=str(topology_gate["block_reason"] or "pair_blocked_by_topology"),
            )

        release_complete = self._spec_or_diagnostic_bool(spec, relation_state, "release_complete", False)
        if not release_complete:
            return self._posture_stage_result(
                family="local_transfer",
                stage="active_joint_release",
                relation_state=relation_state,
                spec=spec,
                context=context,
                orientation_ref_key="release_orientation_ref_deg",
                bend_ref_key="release_bend_ref_deg",
                orientation_done_key="release_orientation_done_deg",
                bend_done_key="release_bend_done_deg",
                completion_reason="release_aligned",
            )

        posture_complete = self._resolve_posture_complete(
            relation_state,
            spec,
            orientation_done_key="posture_orientation_done_deg",
            bend_done_key="posture_bend_done_deg",
            explicit_flag="posture_complete",
        )
        if not posture_complete:
            return self._posture_stage_result(
                family="local_transfer",
                stage="local_posture_adjust",
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
            return self._wrap_delegate_result(
                delegated,
                family="local_transfer",
                stage="approach_passive_joint",
            )
        if relation_state.coupled is not True:
            delegated = self.fine_controller.control(relation_state, spec, context=context)
            return self._wrap_delegate_result(
                delegated,
                family="local_transfer",
                stage="dock_with_passive_joint",
            )

        cooperate_complete = self._cooperate_complete(relation_state, spec)
        if not cooperate_complete:
            return self._cooperate_stage_result(
                family="local_transfer",
                stage="cooperate_with_passive_joint",
                relation_state=relation_state,
                spec=spec,
                context=context,
                returnable_to_free_growth=False,
            )

        return PairControllerResult(
            family="local_transfer",
            stage="local_transfer_complete",
            status="done",
            active_module=relation_state.active_module,
            passive_module=relation_state.passive_module,
            relation_state=relation_state,
            selected_controller="local_transfer_controller",
            selected_mpc=None,
            primitive_references=self._joint_zero_hold_references(relation_state.active_module),
            enabled_axes=("crawl", "rotate", "bend"),
            completion_reason="cooperate_complete",
            diagnostics=self._merge_diagnostics(diagnostics, {"returnable_to_free_growth": False}),
            notes=["local transfer complete"],
        )

    def _resolve_posture_complete(
        self,
        relation_state: RelationState,
        spec: SkillSpec,
        *,
        orientation_done_key: str,
        bend_done_key: str,
        explicit_flag: str,
    ) -> bool:
        """Resolve whether the local posture stage is already aligned."""
        if explicit_flag in spec.params or explicit_flag in relation_state.diagnostics:
            return self._spec_or_diagnostic_bool(spec, relation_state, explicit_flag, False)
        orientation_error = self.safe_float(relation_state.orientation_error_deg, 0.0) or 0.0
        bend_error = self._spec_or_diagnostic_float(spec, relation_state, "bend_error_deg", 0.0) or 0.0
        orientation_done = abs(
            orientation_error - float(spec.float_param("posture_orientation_ref_deg", 0.0) or 0.0)
        ) <= abs(float(spec.float_param(orientation_done_key, 2.0) or 2.0))
        bend_done = abs(
            bend_error - float(spec.float_param("posture_bend_ref_deg", 0.0) or 0.0)
        ) <= abs(float(spec.float_param(bend_done_key, 2.0) or 2.0))
        return bool(orientation_done and bend_done)

    def _posture_stage_result(
        self,
        *,
        family: str,
        stage: str,
        relation_state: RelationState,
        spec: SkillSpec,
        context: ExecutionContext | None,
        orientation_ref_key: str,
        bend_ref_key: str,
        orientation_done_key: str,
        bend_done_key: str,
        completion_reason: str,
    ) -> PairControllerResult:
        """Run one release/posture stage through the generic posture-adjust MPC."""
        orientation_ref_deg = float(spec.float_param(orientation_ref_key, 0.0) or 0.0)
        bend_ref_deg = float(spec.float_param(bend_ref_key, 0.0) or 0.0)
        orientation_error_deg = float(self.safe_float(relation_state.orientation_error_deg, 0.0) or 0.0)
        bend_error_deg = float(self._spec_or_diagnostic_float(spec, relation_state, "bend_error_deg", 0.0) or 0.0)
        dt = spec.solver_dt if context is None or context.dt is None else float(context.dt)
        solver_output = self.posture_adjust_mpc.compute(
            current_orientation_error_deg=orientation_error_deg,
            orientation_ref_deg=orientation_ref_deg,
            current_bend_error_deg=bend_error_deg,
            bend_ref_deg=bend_ref_deg,
            dt=dt,
            limits=spec.limits,
            config=spec.config,
        )
        orientation_done_deg = abs(float(spec.float_param(orientation_done_key, 2.0) or 2.0))
        bend_done_deg = abs(float(spec.float_param(bend_done_key, 2.0) or 2.0))
        done = abs(solver_output.orientation_error_deg) <= orientation_done_deg and abs(solver_output.bend_error_deg) <= bend_done_deg
        references = [
            PrimitiveReference(
                module_id=relation_state.active_module,
                primitive_name="joint_rotate",
                axis="rotate",
                reference_kind="velocity",
                reference_value=0.0 if done else solver_output.rotate_velocity_deg_s,
                units="deg/s",
                primary=True,
                semantic="zero_hold" if done else stage,
                target_value=orientation_ref_deg,
                metadata={"orientation_error_deg": solver_output.orientation_error_deg},
            ),
            PrimitiveReference(
                module_id=relation_state.active_module,
                primitive_name="joint_bend",
                axis="bend",
                reference_kind="velocity",
                reference_value=0.0 if done else solver_output.bend_velocity_deg_s,
                units="deg/s",
                primary=True,
                semantic="zero_hold" if done else stage,
                target_value=bend_ref_deg,
                metadata={"bend_error_deg": solver_output.bend_error_deg},
            ),
            PrimitiveReference(
                module_id=relation_state.active_module,
                primitive_name="joint_crawl",
                axis="crawl",
                reference_kind="velocity",
                reference_value=0.0,
                units="mm/s",
                primary=False,
                semantic="nominal_hold",
                target_value=0.0,
                metadata={"role": "secondary"},
            ),
        ]
        return PairControllerResult(
            family=family,
            stage=stage,
            status="done" if done else "active",
            active_module=relation_state.active_module,
            passive_module=relation_state.passive_module,
            relation_state=relation_state,
            selected_controller=f"{family}_controller",
            selected_mpc="posture_adjust_mpc",
            primitive_references=references,
            enabled_axes=("crawl", "rotate", "bend"),
            completion_reason=completion_reason if done else None,
            diagnostics=self._merge_diagnostics(
                {
                    "orientation_ref_deg": orientation_ref_deg,
                    "bend_ref_deg": bend_ref_deg,
                    "orientation_done_deg": orientation_done_deg,
                    "bend_done_deg": bend_done_deg,
                }
            ),
            notes=list(solver_output.notes),
        )

    def _cooperate_complete(self, relation_state: RelationState, spec: SkillSpec) -> bool:
        """Resolve whether the cooperation stage has already finished."""
        if "cooperate_complete" in spec.params or "cooperate_complete" in relation_state.diagnostics:
            return self._spec_or_diagnostic_bool(spec, relation_state, "cooperate_complete", False)
        progress = self._spec_or_diagnostic_float(spec, relation_state, "cooperate_progress", 0.0) or 0.0
        target = float(spec.float_param("cooperate_target_progress", 1.0) or 1.0)
        return bool(progress >= target)

    def _cooperate_stage_result(
        self,
        *,
        family: str,
        stage: str,
        relation_state: RelationState,
        spec: SkillSpec,
        context: ExecutionContext | None,
        returnable_to_free_growth: bool,
    ) -> PairControllerResult:
        """Run one cooperation stage through the generic cooperate MPC."""
        dt = spec.solver_dt if context is None or context.dt is None else float(context.dt)
        solver_output = self.cooperate_mpc.compute(
            feed_forward_mm_s=float(spec.float_param("cooperate_feed_velocity_mm_s", 2.0) or 2.0),
            current_distance_mm=float(self.safe_float(relation_state.distance_mm, 0.0) or 0.0),
            distance_ref_mm=float(spec.float_param("distance_ref_mm", 0.0) or 0.0),
            current_orientation_error_deg=float(self.safe_float(relation_state.orientation_error_deg, 0.0) or 0.0),
            orientation_ref_deg=float(spec.float_param("cooperate_orientation_ref_deg", 0.0) or 0.0),
            current_bend_error_deg=float(self._spec_or_diagnostic_float(spec, relation_state, "bend_error_deg", 0.0) or 0.0),
            bend_ref_deg=float(spec.float_param("cooperate_bend_ref_deg", 0.0) or 0.0),
            dt=dt,
            limits=spec.limits,
            config=spec.config,
        )
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
                semantic="cooperate_stabilize",
                target_value=float(spec.float_param("cooperate_orientation_ref_deg", 0.0) or 0.0),
                metadata={"orientation_error_deg": solver_output.orientation_error_deg},
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
                target_value=float(spec.float_param("cooperate_bend_ref_deg", 0.0) or 0.0),
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
            selected_controller=f"{family}_controller",
            selected_mpc="cooperate_mpc",
            primitive_references=references,
            enabled_axes=("crawl", "rotate", "bend"),
            returnable_to_free_growth=returnable_to_free_growth,
            diagnostics=self._merge_diagnostics(
                {
                    "cooperate_feed_velocity_mm_s": solver_output.feed_velocity_mm_s,
                    "cooperate_distance_error_mm": solver_output.distance_error_mm,
                    "cooperate_orientation_error_deg": solver_output.orientation_error_deg,
                    "cooperate_bend_error_deg": solver_output.bend_error_deg,
                }
            ),
            notes=list(solver_output.notes),
        )

    def _wrap_delegate_result(
        self,
        delegated: PairControllerResult,
        *,
        family: str,
        stage: str,
    ) -> PairControllerResult:
        """Relabel one generic pair-controller result under the local-transfer stage."""
        return PairControllerResult(
            family=family,
            stage=stage,
            status=delegated.status,
            active_module=delegated.active_module,
            passive_module=delegated.passive_module,
            relation_state=delegated.relation_state,
            selected_controller=delegated.selected_controller,
            selected_mpc=delegated.selected_mpc,
            primitive_references=list(delegated.primitive_references),
            enabled_axes=delegated.enabled_axes,
            completion_reason=delegated.completion_reason,
            diagnostics=self._merge_diagnostics(delegated.diagnostics, {"skill_family": family}),
            notes=list(delegated.notes),
        )

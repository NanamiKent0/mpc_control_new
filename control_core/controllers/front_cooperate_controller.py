"""Generic front-cooperate controller for joint-to-tip pair families."""

from __future__ import annotations

from ..models.execution_context import ExecutionContext
from ..models.skill_types import SkillSpec
from ..mpc.cooperate_mpc import CooperateMPC
from ..mpc.posture_adjust_mpc import PostureAdjustMPC
from ..topology.chain_topology import ChainTopology
from ..topology.relation_state import RelationState
from .coarse_approach_controller import CoarseApproachController
from .fine_dock_controller import FineDockController
from .local_transfer_controller import LocalTransferController
from .pair_controller_support import PairControllerResult, PairControllerSupportMixin


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

        cooperate_complete = self._local_helper._cooperate_complete(relation_state, spec)
        if not cooperate_complete:
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
            diagnostics=self._merge_diagnostics(diagnostics, {"returnable_to_free_growth": True}),
            notes=["front cooperation complete; caller may return to free growth"],
        )

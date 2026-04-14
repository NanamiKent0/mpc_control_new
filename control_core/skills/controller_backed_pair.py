"""Thin skill wrappers around pair-controller families used by turn workflows."""

from __future__ import annotations

from ..controllers import FrontCooperateController, LocalTransferController, PairControllerResult
from ..models.execution_context import ExecutionContext
from ..models.module_state import ModuleState
from ..models.skill_types import (
    SkillCheckResult,
    SkillCompletionResult,
    SkillExecutionResult,
    SkillSpec,
)
from ..topology.chain_topology import ChainTopology
from ..topology.relation_state import RelationState


class _ControllerBackedPairSkill:
    """Adapt a controller-family `control(...)` call into a scheduler skill."""

    skill_key = ""

    def __init__(self, *, topology: ChainTopology | None = None) -> None:
        self.topology = topology
        self.controller = self._build_controller(topology=topology)

    def _build_controller(self, *, topology: ChainTopology | None) -> object:
        raise NotImplementedError

    def execute(
        self,
        source: object,
        spec: SkillSpec,
        *,
        module_states: dict[str, ModuleState] | None = None,
        context: ExecutionContext | None = None,
    ) -> SkillExecutionResult:
        """Execute the wrapped controller and normalize its result."""
        relation_state = _coerce_relation_state(source, spec)
        resolved_topology = self.topology if context is None or context.topology is None else context.topology
        _bind_topology(self.controller, resolved_topology)
        controller_result = self.controller.control(
            relation_state,
            spec,
            context=context,
        )
        return _skill_result_from_controller_result(
            controller_result,
            spec=spec,
            relation_state=relation_state,
            module_states=module_states,
            topology=resolved_topology,
            used_context_topology=bool(context is not None and context.topology is not None),
        )


class LocalTransferSkill(_ControllerBackedPairSkill):
    """Scheduler-executable wrapper for the adjacent joint-transfer controller."""

    skill_key = "local_transfer"

    def _build_controller(self, *, topology: ChainTopology | None) -> LocalTransferController:
        return LocalTransferController(topology=topology)


class FrontCooperateSkill(_ControllerBackedPairSkill):
    """Scheduler-executable wrapper for the `joint1->tip` cooperation controller."""

    skill_key = "front_cooperate"

    def _build_controller(self, *, topology: ChainTopology | None) -> FrontCooperateController:
        return FrontCooperateController(topology=topology)


def _coerce_relation_state(source: object, spec: SkillSpec) -> RelationState:
    """Return a relation state even when older callers provide a placeholder source."""
    if isinstance(source, RelationState):
        return source
    return RelationState(
        active_module=spec.active_module,
        passive_module=spec.passive_module,
        relation_type=spec.relation_type,
        distance_mm=None,
        orientation_error_deg=None,
        coupled=None,
        observation_valid=False,
        diagnostics={"controller_backed_skill_source": type(source).__name__},
    )


def _bind_topology(controller: object, topology: ChainTopology | None) -> None:
    """Push the resolved topology onto nested controller helpers when present."""
    if hasattr(controller, "topology"):
        controller.topology = topology
    for attribute_name in ("coarse_controller", "fine_controller", "_local_helper"):
        nested = getattr(controller, attribute_name, None)
        if nested is not None and hasattr(nested, "topology"):
            nested.topology = topology
            if attribute_name == "_local_helper":
                _bind_topology(nested, topology)


def _skill_result_from_controller_result(
    controller_result: PairControllerResult,
    *,
    spec: SkillSpec,
    relation_state: RelationState,
    module_states: dict[str, ModuleState] | None,
    topology: ChainTopology | None,
    used_context_topology: bool,
) -> SkillExecutionResult:
    """Normalize one controller-family result into the scheduler skill surface."""
    topology_snapshot = {} if topology is None else topology.snapshot()
    frontier_snapshot = {} if topology is None else topology.frontier_snapshot()
    support_snapshot = {} if topology is None else topology.support_snapshot()
    blocked = controller_result.status == "blocked"
    done = controller_result.status == "done"
    block_reason = controller_result.completion_reason if blocked else None
    topology_block_reason = controller_result.diagnostics.get("topology_block_reason")
    blocked_by_topology = blocked and topology_block_reason not in {None, "", False}
    preconditions = SkillCheckResult(
        passed=not blocked,
        blocking_reason=block_reason,
        blocked_by_topology=blocked_by_topology,
        block_reason=block_reason,
        used_context_topology=used_context_topology,
        topology_snapshot=topology_snapshot,
        frontier_snapshot=frontier_snapshot,
        support_snapshot=support_snapshot,
        diagnostics={
            "controller_stage": controller_result.stage,
            "selected_controller": controller_result.selected_controller,
            "selected_mpc": controller_result.selected_mpc,
            "skill_family": controller_result.family,
            "returnable_to_free_growth": controller_result.returnable_to_free_growth,
        },
        notes=list(controller_result.notes),
    )
    completion = SkillCompletionResult(
        done=done,
        completion_reason=controller_result.completion_reason if done else None,
        blocked_by_topology=blocked_by_topology,
        block_reason=block_reason if blocked else None,
        used_context_topology=used_context_topology,
        topology_snapshot=topology_snapshot,
        frontier_snapshot=frontier_snapshot,
        support_snapshot=support_snapshot,
        diagnostics={
            "controller_stage": controller_result.stage,
            "selected_controller": controller_result.selected_controller,
            "selected_mpc": controller_result.selected_mpc,
            "returnable_to_free_growth": controller_result.returnable_to_free_growth,
        },
        notes=list(controller_result.notes),
    )
    diagnostics = {
        **dict(relation_state.diagnostics),
        **dict(controller_result.diagnostics),
        "skill_key": spec.skill_key,
        "skill_family": controller_result.family,
        "controller_stage": controller_result.stage,
        "selected_controller": controller_result.selected_controller,
        "selected_mpc": controller_result.selected_mpc,
        "returnable_to_free_growth": controller_result.returnable_to_free_growth,
        "status": controller_result.status,
        "blocked_by_topology": blocked_by_topology,
        "block_reason": block_reason,
        "used_context_topology": used_context_topology,
    }
    return SkillExecutionResult(
        skill_spec=spec,
        relation_state=relation_state,
        skill_key=spec.skill_key,
        active_module=relation_state.active_module,
        passive_module=relation_state.passive_module,
        module_states=dict(module_states or {}),
        preconditions=preconditions,
        completion=completion,
        primitive_references=list(controller_result.primitive_references),
        status=controller_result.status,
        blocked_by_topology=blocked_by_topology,
        not_fully_supported=False,
        block_reason=block_reason,
        used_context_topology=used_context_topology,
        topology_snapshot=topology_snapshot,
        frontier_snapshot=frontier_snapshot,
        support_snapshot=support_snapshot,
        diagnostics=diagnostics,
        notes=list(controller_result.notes),
    )


__all__ = [
    "FrontCooperateSkill",
    "LocalTransferSkill",
]

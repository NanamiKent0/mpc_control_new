"""Fixed-range turn planner for chain transfer into `joint1-tip` cooperation."""

from __future__ import annotations

from typing import TYPE_CHECKING

from .front_cooperation_plan import FRONT_COOPERATION_PLAN_KIND, FrontCooperationPlan
from .joint_selection_result import JointSelectionResult
from .plan_nodes import (
    FrontCooperateNode,
    LocalTransferNode,
    ReturnToFreeGrowthNode,
    TurnPlanNode,
)
from .selection_policy import FIXED_FRONT_SELECTION_JOINT_IDS, select_front_joint_candidate

if TYPE_CHECKING:
    from ..estimation import EstimateBundle


TURN_PLANNER_SOURCE = "control_core.supervisor.turn_planner"
SUPPORTED_TURN_PLANNER_JOINT_IDS = FIXED_FRONT_SELECTION_JOINT_IDS
SUPPORTED_TURN_PLANNER_JOINT_INDEXES = tuple(
    range(1, len(SUPPORTED_TURN_PLANNER_JOINT_IDS) + 1)
)


def plan_turn_workflow(estimate_bundle: "EstimateBundle") -> FrontCooperationPlan:
    """Build the structured turn plan from the current semantic estimate bundle."""
    selection_result = select_front_joint_candidate(estimate_bundle)
    if selection_result.selected_joint_id == "joint1":
        return build_direct_front_cooperation_plan(selection_result=selection_result)
    if selection_result.selected_joint_index in {2, 3, 4, 5}:
        return build_recursive_transfer_plan(
            selected_joint_index=int(selection_result.selected_joint_index),
            selection_result=selection_result,
        )
    return _build_invariant_violation_plan(selection_result=selection_result)


def build_direct_front_cooperation_plan(
    *,
    selection_result: JointSelectionResult | None = None,
) -> FrontCooperationPlan:
    """Build the direct `joint1-tip` plan when `joint1` is already idle."""
    resolved_selection = selection_result or JointSelectionResult(
        selected_joint_id="joint1",
        selected_joint_index=1,
        direct_front_cooperation=True,
        requires_recursive_transfer=False,
        selection_reason="joint1_idle_direct_front_cooperation",
        diagnostics={},
    )
    nodes: list[TurnPlanNode] = [
        FrontCooperateNode(),
        ReturnToFreeGrowthNode(),
    ]
    plan = FrontCooperationPlan(
        plan_kind=FRONT_COOPERATION_PLAN_KIND,
        selected_joint_id=resolved_selection.selected_joint_id,
        selected_joint_index=resolved_selection.selected_joint_index,
        selection_reason=resolved_selection.selection_reason,
        direct_front_cooperation=True,
        requires_recursive_transfer=False,
        ordered_nodes=nodes,
    )
    return _finalize_plan(plan, selection_result=resolved_selection)


def build_recursive_transfer_plan(
    selected_joint_index: int,
    *,
    selection_result: JointSelectionResult | None = None,
) -> FrontCooperationPlan:
    """Build the full local-transfer chain that converges into `joint1-tip`."""
    if selected_joint_index not in {2, 3, 4, 5}:
        raise ValueError(f"turn_planner_recursive_transfer_index_unsupported:{selected_joint_index}")
    selected_joint_id = _joint_id_for_index(selected_joint_index)
    resolved_selection = selection_result or JointSelectionResult(
        selected_joint_id=selected_joint_id,
        selected_joint_index=selected_joint_index,
        direct_front_cooperation=False,
        requires_recursive_transfer=True,
        selection_reason="first_idle_joint_selected_requires_recursive_transfer",
        diagnostics={},
    )
    nodes: list[TurnPlanNode] = []
    for joint_index in range(selected_joint_index, 1, -1):
        active_module = _joint_id_for_index(joint_index)
        passive_module = _joint_id_for_index(joint_index - 1)
        nodes.append(
            LocalTransferNode(
                active_module,
                passive_module,
                metadata={
                    "transfer_from_index": joint_index,
                    "transfer_to_index": joint_index - 1,
                },
            )
        )
    nodes.extend(
        [
            FrontCooperateNode(),
            ReturnToFreeGrowthNode(),
        ]
    )
    plan = FrontCooperationPlan(
        plan_kind=FRONT_COOPERATION_PLAN_KIND,
        selected_joint_id=resolved_selection.selected_joint_id,
        selected_joint_index=resolved_selection.selected_joint_index,
        selection_reason=resolved_selection.selection_reason,
        direct_front_cooperation=False,
        requires_recursive_transfer=True,
        ordered_nodes=nodes,
    )
    return _finalize_plan(plan, selection_result=resolved_selection)


def assert_turn_planner_invariant(plan: FrontCooperationPlan) -> None:
    """Raise when the plan does not satisfy the fixed-range workflow invariant."""
    errors = _collect_turn_planner_invariant_errors(plan)
    if errors:
        raise AssertionError(
            "turn_planner_invariant_violated:" + "|".join(errors)
        )


def _build_invariant_violation_plan(
    *,
    selection_result: JointSelectionResult | None = None,
) -> FrontCooperationPlan:
    resolved_selection = selection_result or JointSelectionResult(
        selected_joint_id=None,
        selected_joint_index=None,
        direct_front_cooperation=False,
        requires_recursive_transfer=False,
        selection_reason="no_idle_joint_found",
        diagnostics={},
    )
    plan = FrontCooperationPlan(
        plan_kind=FRONT_COOPERATION_PLAN_KIND,
        selected_joint_id=None,
        selected_joint_index=None,
        selection_reason=resolved_selection.selection_reason,
        direct_front_cooperation=False,
        requires_recursive_transfer=False,
        ordered_nodes=[],
    )
    return _finalize_plan(
        plan,
        selection_result=resolved_selection,
        error_code="invariant_violated",
    )


def _finalize_plan(
    plan: FrontCooperationPlan,
    *,
    selection_result: JointSelectionResult,
    error_code: str | None = None,
) -> FrontCooperationPlan:
    errors = _collect_turn_planner_invariant_errors(plan)
    diagnostics = _build_plan_diagnostics(
        plan,
        selection_result=selection_result,
        error_code=error_code,
        invariant_errors=errors,
    )
    plan.diagnostics = diagnostics
    plan.invariant_ok = not errors
    return plan


def _build_plan_diagnostics(
    plan: FrontCooperationPlan,
    *,
    selection_result: JointSelectionResult,
    error_code: str | None,
    invariant_errors: list[str],
) -> dict[str, object]:
    ordered_node_ids = [node.node_id for node in plan.ordered_nodes]
    ordered_node_kinds = [node.node_kind for node in plan.ordered_nodes]
    transfer_chain = [
        f"{node.active_module}->{node.passive_module}"
        for node in plan.ordered_nodes
        if isinstance(node, LocalTransferNode)
    ]
    diagnostics: dict[str, object] = {
        "planner_source": TURN_PLANNER_SOURCE,
        "joint_order": list(SUPPORTED_TURN_PLANNER_JOINT_IDS),
        "selection_reason": plan.selection_reason,
        "selection_diagnostics": dict(selection_result.diagnostics),
        "selected_joint_id": plan.selected_joint_id,
        "selected_joint_index": plan.selected_joint_index,
        "ordered_node_ids": ordered_node_ids,
        "ordered_node_kinds": ordered_node_kinds,
        "transfer_chain": transfer_chain,
        "contains_front_cooperate": _contains_front_cooperate(plan.ordered_nodes),
        "returns_to_tip_free_growth": _returns_to_tip_free_growth(plan.ordered_nodes),
        "invariant_errors": list(invariant_errors),
    }
    if error_code is not None:
        diagnostics["error_code"] = error_code
    return diagnostics


def _collect_turn_planner_invariant_errors(plan: FrontCooperationPlan) -> list[str]:
    errors: list[str] = []
    selected_joint_id = plan.selected_joint_id
    selected_joint_index = plan.selected_joint_index
    if selected_joint_id is None or selected_joint_index is None:
        if selected_joint_id is not None or selected_joint_index is not None:
            errors.append("partial_selection_state")
        if plan.ordered_nodes:
            errors.append("error_plan_must_not_emit_nodes")
        if plan.diagnostics.get("error_code") not in {None, "invariant_violated"}:
            errors.append("error_plan_invalid_error_code")
        if plan.diagnostics.get("error_code") is None and plan.selection_reason != "no_idle_joint_found":
            errors.append("error_plan_missing_structured_reason")
        return errors or ["no_idle_joint_found"]

    if selected_joint_index not in SUPPORTED_TURN_PLANNER_JOINT_INDEXES:
        errors.append("selected_joint_index_out_of_supported_range")
    else:
        expected_joint_id = _joint_id_for_index(selected_joint_index)
        if selected_joint_id != expected_joint_id:
            errors.append("selected_joint_id_index_mismatch")
    if plan.direct_front_cooperation is not (selected_joint_index == 1):
        errors.append("direct_front_cooperation_flag_mismatch")
    if plan.requires_recursive_transfer is not (selected_joint_index in {2, 3, 4, 5}):
        errors.append("requires_recursive_transfer_flag_mismatch")
    if not _contains_front_cooperate(plan.ordered_nodes):
        errors.append("missing_joint1_tip_front_cooperate")
    if not _returns_to_tip_free_growth(plan.ordered_nodes):
        errors.append("missing_return_to_tip_free_growth")

    local_transfer_pairs = [
        (node.active_module, node.passive_module)
        for node in plan.ordered_nodes
        if isinstance(node, LocalTransferNode)
    ]
    if selected_joint_index == 1:
        if local_transfer_pairs:
            errors.append("direct_plan_must_not_contain_local_transfer_nodes")
    else:
        expected_pairs = _expected_transfer_pairs(selected_joint_index)
        if local_transfer_pairs != expected_pairs:
            errors.append("recursive_transfer_chain_mismatch")
    return errors


def _contains_front_cooperate(nodes: list[TurnPlanNode]) -> bool:
    return any(
        isinstance(node, FrontCooperateNode)
        and node.active_module == "joint1"
        and node.passive_module == "tip"
        for node in nodes
    )


def _returns_to_tip_free_growth(nodes: list[TurnPlanNode]) -> bool:
    return bool(nodes) and isinstance(nodes[-1], ReturnToFreeGrowthNode)


def _expected_transfer_pairs(selected_joint_index: int) -> list[tuple[str | None, str | None]]:
    return [
        (_joint_id_for_index(joint_index), _joint_id_for_index(joint_index - 1))
        for joint_index in range(selected_joint_index, 1, -1)
    ]


def _joint_id_for_index(joint_index: int) -> str:
    if joint_index not in SUPPORTED_TURN_PLANNER_JOINT_INDEXES:
        raise ValueError(f"turn_planner_joint_index_unsupported:{joint_index}")
    return SUPPORTED_TURN_PLANNER_JOINT_IDS[joint_index - 1]


__all__ = [
    "SUPPORTED_TURN_PLANNER_JOINT_IDS",
    "SUPPORTED_TURN_PLANNER_JOINT_INDEXES",
    "TURN_PLANNER_SOURCE",
    "assert_turn_planner_invariant",
    "build_direct_front_cooperation_plan",
    "build_recursive_transfer_plan",
    "plan_turn_workflow",
]

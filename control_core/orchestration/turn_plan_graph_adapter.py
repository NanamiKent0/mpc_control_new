"""Convert structured turn plans into scheduler-executable task graphs."""

from __future__ import annotations

from ..models.skill_types import SkillSpec
from ..models.task_types import TIP_FREE_GROWTH, TIP_TURN_AUTONOMOUS, TaskGraphSpec, TaskNode
from ..supervisor.front_cooperation_plan import FrontCooperationPlan
from ..supervisor.plan_nodes import (
    FrontCooperateNode,
    LocalTransferNode,
    ReturnToFreeGrowthNode,
    TipFreeGrowthNode,
    TurnPlanNode,
)
from ..supervisor.turn_planner import assert_turn_planner_invariant
from .graph_builder import TaskGraphBuilder

TURN_GRAPH_ADAPTER_SOURCE = "control_core.orchestration.turn_plan_graph_adapter"
TURN_WORKFLOW_ORDERED_MODULES = ("tip", "joint1", "joint2", "joint3", "joint4", "joint5")
TURN_RELATION_DISTANCE_DONE_MM = 2.0
TURN_RELATION_ORIENTATION_DONE_DEG = 2.0


def build_turn_autonomous_graph_from_plan(
    plan: FrontCooperationPlan,
    *,
    graph_id: str | None = None,
    metadata: dict[str, object] | None = None,
) -> TaskGraphSpec:
    """Compile one planner output into a linear turn-workflow task graph."""
    assert_turn_planner_invariant(plan)
    if not plan.ordered_nodes:
        raise ValueError("turn_plan_graph_adapter_plan_has_no_nodes")
    resolved_graph_id = graph_id or _default_graph_id(plan)
    builder = TaskGraphBuilder(
        resolved_graph_id,
        metadata=_graph_metadata(plan, metadata=metadata),
    )
    previous_node_id: str | None = None
    for plan_node in plan.ordered_nodes:
        task_node = _task_node_from_plan_node(plan_node, plan=plan)
        builder.add_node(task_node)
        if previous_node_id is not None:
            builder.link_linear(previous_node_id, task_node.node_id)
        previous_node_id = task_node.node_id
    tip_free_growth_node = _tip_free_growth_task_node(plan=plan)
    builder.add_node(tip_free_growth_node)
    assert previous_node_id is not None
    builder.link_linear(previous_node_id, tip_free_growth_node.node_id)
    return builder.build(start_node_id=plan.ordered_nodes[0].node_id)


def build_tip_free_growth_graph(
    *,
    graph_id: str = "tip_free_growth",
    metadata: dict[str, object] | None = None,
) -> TaskGraphSpec:
    """Build the minimal single-node steady-state free-growth graph."""
    plan = FrontCooperationPlan(
        planner_mode=None,
        selected_joint_id=None,
        selected_joint_index=None,
        selection_reason="tip_free_growth_terminal",
        direct_front_cooperation=False,
        requires_recursive_transfer=False,
    )
    builder = TaskGraphBuilder(
        graph_id,
        metadata={
            "graph_family": "tip_free_growth",
            "graph_template": "tip_free_growth",
            "graph_label": "Tip Free Growth",
            "graph_name": "Tip Free Growth",
            "graph_display_name": "Tip Free Growth",
            "high_level_task_kind": TIP_FREE_GROWTH,
            "transition_policy_key": "default",
            "default_transition_policy_key": "default",
            "selected_joint_id": None,
            "selected_joint_index": None,
            "direct_front_cooperation": False,
            "requires_recursive_transfer": False,
            "planner_mode": None,
            "returns_to_tip_free_growth": True,
            "ordered_node_ids": ["tip_free_growth"],
            "ordered_node_kinds": ["tip_free_growth"],
            "current_active_pair": None,
            "allow_off_frontier": True,
            "requires_support_stability": False,
            "ordered_modules": list(TURN_WORKFLOW_ORDERED_MODULES),
            "turn_graph_adapter_source": TURN_GRAPH_ADAPTER_SOURCE,
            **dict(metadata or {}),
        },
    )
    builder.add_node(
        _terminal_task_node(
            node_id="tip_free_growth",
            skill_key="tip_free_growth",
            plan_node_kind="tip_free_growth",
            plan=plan,
            current_active_pair=None,
            returning_to_tip_free_growth=True,
        )
    )
    return builder.build(start_node_id="tip_free_growth")


def _default_graph_id(plan: FrontCooperationPlan) -> str:
    planner_mode = plan.planner_mode or "unknown"
    selected_joint_id = plan.selected_joint_id or "unresolved"
    return f"turn_autonomous_{planner_mode}_{selected_joint_id}"


def _graph_metadata(
    plan: FrontCooperationPlan,
    *,
    metadata: dict[str, object] | None,
) -> dict[str, object]:
    ordered_node_ids = [node.node_id for node in plan.ordered_nodes] + ["tip_free_growth"]
    ordered_node_kinds = [node.node_kind for node in plan.ordered_nodes] + ["tip_free_growth"]
    return {
        **dict(plan.diagnostics),
        "graph_family": "turn_autonomous",
        "graph_template": "turn_autonomous",
        "graph_label": "Turn Autonomous",
        "graph_name": "Turn Autonomous",
        "graph_display_name": "Turn Autonomous",
        "high_level_task_kind": TIP_TURN_AUTONOMOUS,
        "transition_policy_key": "default",
        "default_transition_policy_key": "default",
        "plan_kind": plan.plan_kind,
        "planner_mode": plan.planner_mode,
        "selected_joint_id": plan.selected_joint_id,
        "selected_joint_index": plan.selected_joint_index,
        "direct_front_cooperation": plan.direct_front_cooperation,
        "requires_recursive_transfer": plan.requires_recursive_transfer,
        "returns_to_tip_free_growth": bool(plan.diagnostics.get("returns_to_tip_free_growth")),
        "selection_reason": plan.selection_reason,
        "ordered_node_ids": ordered_node_ids,
        "ordered_node_kinds": ordered_node_kinds,
        "ordered_modules": list(TURN_WORKFLOW_ORDERED_MODULES),
        "allow_off_frontier": True,
        "requires_support_stability": False,
        "turn_graph_adapter_source": TURN_GRAPH_ADAPTER_SOURCE,
        **dict(metadata or {}),
    }


def _task_node_from_plan_node(
    plan_node: TurnPlanNode,
    *,
    plan: FrontCooperationPlan,
) -> TaskNode:
    if isinstance(plan_node, LocalTransferNode):
        current_active_pair = f"{plan_node.active_module}->{plan_node.passive_module}"
        return TaskNode(
            node_id=plan_node.node_id,
            skill_spec=_local_transfer_spec(plan_node),
            metadata=_node_metadata(
                plan_node,
                plan=plan,
                current_active_pair=current_active_pair,
                returning_to_tip_free_growth=False,
            ),
        )
    if isinstance(plan_node, FrontCooperateNode):
        current_active_pair = f"{plan_node.active_module}->{plan_node.passive_module}"
        return TaskNode(
            node_id=plan_node.node_id,
            skill_spec=_front_cooperate_spec(plan_node),
            metadata=_node_metadata(
                plan_node,
                plan=plan,
                current_active_pair=current_active_pair,
                returning_to_tip_free_growth=False,
            ),
        )
    if isinstance(plan_node, ReturnToFreeGrowthNode):
        return _terminal_task_node(
            node_id=plan_node.node_id,
            skill_key="return_to_free_growth",
            plan_node_kind=plan_node.node_kind,
            plan=plan,
            current_active_pair=None,
            returning_to_tip_free_growth=True,
        )
    raise ValueError(f"turn_plan_graph_adapter_node_kind_unsupported:{plan_node.node_kind}")


def _local_transfer_spec(plan_node: LocalTransferNode) -> SkillSpec:
    return SkillSpec(
        skill_key="local_transfer",
        active_module=str(plan_node.active_module),
        passive_module=str(plan_node.passive_module),
        relation_type="joint_joint",
        distance_done_mm=TURN_RELATION_DISTANCE_DONE_MM,
        orientation_done_deg=TURN_RELATION_ORIENTATION_DONE_DEG,
        limits={
            "feed_mm_s": 4.0,
            "crawl_mm_s": 4.0,
            "rotate_deg_s": 6.0,
            "bend_deg_s": 5.0,
        },
        config={
            "gain": 0.2,
            "distance_gain": 0.2,
            "orientation_gain": 0.25,
            "bend_gain": 0.2,
        },
        params={
            "release_complete": True,
            "posture_complete": True,
            "distance_ref_mm": 0.0,
            "orientation_ref_deg": 0.0,
            "cooperate_target_progress": 1.0,
        },
        metadata={
            "active_module_type": "joint",
            "pair": f"{plan_node.active_module}->{plan_node.passive_module}",
            "graph_template": "turn_autonomous",
            "plan_node_kind": plan_node.node_kind,
        },
    )


def _front_cooperate_spec(plan_node: FrontCooperateNode) -> SkillSpec:
    return SkillSpec(
        skill_key="front_cooperate",
        active_module=str(plan_node.active_module),
        passive_module=str(plan_node.passive_module),
        relation_type="tip_joint",
        distance_done_mm=TURN_RELATION_DISTANCE_DONE_MM,
        orientation_done_deg=TURN_RELATION_ORIENTATION_DONE_DEG,
        limits={
            "feed_mm_s": 4.0,
            "crawl_mm_s": 4.0,
            "rotate_deg_s": 6.0,
            "bend_deg_s": 5.0,
        },
        config={
            "gain": 0.2,
            "distance_gain": 0.2,
            "orientation_gain": 0.25,
            "bend_gain": 0.2,
        },
        params={
            "release_complete": True,
            "posture_complete": True,
            "distance_ref_mm": 0.0,
            "orientation_ref_deg": 0.0,
            "required_axes": ["distance"],
            "cooperate_bend_ref_deg": 0.0,
            "cooperate_heading_done_deg": 2.0,
            "cooperate_target_progress": 1.0,
        },
        metadata={
            "active_module_type": "joint",
            "pair": f"{plan_node.active_module}->{plan_node.passive_module}",
            "graph_template": "turn_autonomous",
            "plan_node_kind": plan_node.node_kind,
        },
    )


def _terminal_task_node(
    *,
    node_id: str,
    skill_key: str,
    plan_node_kind: str,
    plan: FrontCooperationPlan,
    current_active_pair: str | None,
    returning_to_tip_free_growth: bool,
) -> TaskNode:
    return TaskNode(
        node_id=node_id,
        skill_spec=SkillSpec(
            skill_key=skill_key,
            active_module="tip",
            passive_module="joint1",
            relation_type="tip_joint",
            metadata={
                "active_module_type": "tip",
                "graph_template": "turn_autonomous",
                "plan_node_kind": plan_node_kind,
            },
        ),
        metadata=_node_metadata(
            TipFreeGrowthNode() if plan_node_kind == "tip_free_growth" else ReturnToFreeGrowthNode(),
            plan=plan,
            current_active_pair=current_active_pair,
            returning_to_tip_free_growth=returning_to_tip_free_growth,
        ),
    )


def _tip_free_growth_task_node(*, plan: FrontCooperationPlan) -> TaskNode:
    return _terminal_task_node(
        node_id="tip_free_growth",
        skill_key="tip_free_growth",
        plan_node_kind="tip_free_growth",
        plan=plan,
        current_active_pair=None,
        returning_to_tip_free_growth=True,
    )


def _node_metadata(
    plan_node: TurnPlanNode,
    *,
    plan: FrontCooperationPlan,
    current_active_pair: str | None,
    returning_to_tip_free_growth: bool,
) -> dict[str, object]:
    metadata = {
        "plan_node_id": plan_node.node_id,
        "plan_node_kind": plan_node.node_kind,
        "current_plan_node_kind": plan_node.node_kind,
        "current_active_pair": current_active_pair,
        "selected_joint_id": plan.selected_joint_id,
        "selected_joint_index": plan.selected_joint_index,
        "direct_front_cooperation": plan.direct_front_cooperation,
        "requires_recursive_transfer": plan.requires_recursive_transfer,
        "planner_mode": plan.planner_mode,
        "returning_to_tip_free_growth": returning_to_tip_free_growth,
        "allow_off_frontier": True,
        "requires_support_stability": False,
        **dict(plan_node.metadata),
    }
    for key in (
        "operator_intent",
        "operator_intent_kind",
        "target_heading_delta_deg",
        "tip_heading_current_deg",
        "tip_heading_target_deg",
    ):
        value = plan.diagnostics.get(key)
        if value is not None:
            metadata[key] = value
    return metadata


__all__ = [
    "TURN_GRAPH_ADAPTER_SOURCE",
    "TURN_WORKFLOW_ORDERED_MODULES",
    "build_tip_free_growth_graph",
    "build_turn_autonomous_graph_from_plan",
]

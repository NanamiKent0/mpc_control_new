"""Structured turn-plan result for front cooperation workflows."""

from __future__ import annotations

from dataclasses import dataclass, field

from .plan_nodes import TurnPlanNode


FRONT_COOPERATION_PLAN_KIND = "front_cooperation_turn"


@dataclass(slots=True)
class FrontCooperationPlan:
    """Planner output for the current fixed-range turn workflow."""

    plan_kind: str = FRONT_COOPERATION_PLAN_KIND
    planner_mode: str | None = None
    selected_joint_id: str | None = None
    selected_joint_index: int | None = None
    selection_reason: str = "uninitialized"
    direct_front_cooperation: bool = False
    requires_recursive_transfer: bool = False
    ordered_nodes: list[TurnPlanNode] = field(default_factory=list)
    diagnostics: dict[str, object] = field(default_factory=dict)
    invariant_ok: bool = False


__all__ = [
    "FRONT_COOPERATION_PLAN_KIND",
    "FrontCooperationPlan",
]

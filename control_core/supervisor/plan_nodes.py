"""Structured planner nodes for the fixed-range turn workflow."""

from __future__ import annotations

from dataclasses import dataclass, field

SUPPORTED_TURN_PLAN_JOINT_IDS = ("joint1", "joint2", "joint3", "joint4", "joint5")
_JOINT_INDEX = {
    joint_id: index for index, joint_id in enumerate(SUPPORTED_TURN_PLAN_JOINT_IDS, start=1)
}


def _require_supported_joint_id(joint_id: str) -> str:
    resolved_joint_id = str(joint_id)
    if resolved_joint_id not in _JOINT_INDEX:
        raise ValueError(f"turn_plan_joint_unsupported:{resolved_joint_id}")
    return resolved_joint_id


def _require_adjacent_local_transfer(active_module: str, passive_module: str) -> tuple[str, str]:
    resolved_active = _require_supported_joint_id(active_module)
    resolved_passive = _require_supported_joint_id(passive_module)
    if _JOINT_INDEX[resolved_active] < 2:
        raise ValueError(f"turn_plan_local_transfer_active_unsupported:{resolved_active}")
    if _JOINT_INDEX[resolved_active] != _JOINT_INDEX[resolved_passive] + 1:
        raise ValueError(
            "turn_plan_local_transfer_pair_invalid:"
            f"{resolved_active}->{resolved_passive}"
        )
    return (resolved_active, resolved_passive)


@dataclass(slots=True)
class TurnPlanNode:
    """One planner-level task node emitted by the turn planner."""

    node_id: str
    node_kind: str
    active_module: str | None
    passive_module: str | None
    relation_type: str | None
    metadata: dict[str, object] = field(default_factory=dict)


class TipFreeGrowthNode(TurnPlanNode):
    """Represent the steady-state tip free-growth mode."""

    def __init__(self, *, metadata: dict[str, object] | None = None) -> None:
        super().__init__(
            node_id="tip_free_growth",
            node_kind="tip_free_growth",
            active_module="tip",
            passive_module=None,
            relation_type=None,
            metadata=dict(metadata or {}),
        )


class LocalTransferNode(TurnPlanNode):
    """Represent one adjacent joint-to-joint local transfer task."""

    def __init__(
        self,
        active_module: str,
        passive_module: str,
        *,
        metadata: dict[str, object] | None = None,
    ) -> None:
        resolved_active, resolved_passive = _require_adjacent_local_transfer(
            active_module,
            passive_module,
        )
        super().__init__(
            node_id=f"local_transfer_{resolved_active}_to_{resolved_passive}",
            node_kind="local_transfer",
            active_module=resolved_active,
            passive_module=resolved_passive,
            relation_type="joint_joint",
            metadata=dict(metadata or {}),
        )


class FrontCooperateNode(TurnPlanNode):
    """Represent one `joint1` to `tip` front-cooperation task."""

    def __init__(
        self,
        active_module: str = "joint1",
        passive_module: str = "tip",
        *,
        metadata: dict[str, object] | None = None,
    ) -> None:
        resolved_active = _require_supported_joint_id(active_module)
        if resolved_active != "joint1" or str(passive_module) != "tip":
            raise ValueError(
                "turn_plan_front_cooperate_pair_invalid:"
                f"{resolved_active}->{passive_module}"
            )
        super().__init__(
            node_id=f"front_cooperate_{resolved_active}_to_tip",
            node_kind="front_cooperate",
            active_module=resolved_active,
            passive_module="tip",
            relation_type="tip_joint",
            metadata=dict(metadata or {}),
        )


class ReturnToFreeGrowthNode(TurnPlanNode):
    """Represent the explicit handoff back to tip free growth."""

    def __init__(self, *, metadata: dict[str, object] | None = None) -> None:
        super().__init__(
            node_id="return_to_tip_free_growth",
            node_kind="return_to_free_growth",
            active_module="tip",
            passive_module=None,
            relation_type=None,
            metadata=dict(metadata or {}),
        )


__all__ = [
    "FrontCooperateNode",
    "LocalTransferNode",
    "ReturnToFreeGrowthNode",
    "SUPPORTED_TURN_PLAN_JOINT_IDS",
    "TipFreeGrowthNode",
    "TurnPlanNode",
]

"""Structured planner nodes for the fixed-range turn workflow."""

from __future__ import annotations

from dataclasses import dataclass, field


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
        super().__init__(
            node_id=f"local_transfer_{active_module}_to_{passive_module}",
            node_kind="local_transfer",
            active_module=str(active_module),
            passive_module=str(passive_module),
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
        super().__init__(
            node_id=f"front_cooperate_{active_module}_to_{passive_module}",
            node_kind="front_cooperate",
            active_module=str(active_module),
            passive_module=str(passive_module),
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
    "TipFreeGrowthNode",
    "TurnPlanNode",
]

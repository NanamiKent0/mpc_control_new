"""Operator-intent dataclasses exposed by the high-level runtime surface."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal, TypeAlias


OperatorIntentKind = Literal["TIP_FREE_GROWTH", "TIP_TURN"]


@dataclass(frozen=True, slots=True)
class TipFreeGrowthIntent:
    """Request that the system enter or remain in tip free-growth mode."""

    intent_kind: Literal["TIP_FREE_GROWTH"] = "TIP_FREE_GROWTH"


@dataclass(frozen=True, slots=True)
class TipTurnIntent:
    """Request that the system rotate the tip heading by a signed delta."""

    target_heading_delta_deg: float
    intent_kind: Literal["TIP_TURN"] = "TIP_TURN"

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "target_heading_delta_deg",
            float(self.target_heading_delta_deg),
        )


OperatorIntent: TypeAlias = TipFreeGrowthIntent | TipTurnIntent


__all__ = [
    "OperatorIntent",
    "OperatorIntentKind",
    "TipFreeGrowthIntent",
    "TipTurnIntent",
]

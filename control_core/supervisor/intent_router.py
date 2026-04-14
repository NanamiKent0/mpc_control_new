"""Route high-level operator intents onto internal scheduler task requests."""

from __future__ import annotations

from collections.abc import Mapping

from ..models.operator_intent_types import OperatorIntent, TipFreeGrowthIntent, TipTurnIntent
from ..models.task_types import HighLevelTaskRequest, TIP_FREE_GROWTH, TIP_TURN_AUTONOMOUS
from .operator_intent import normalize_operator_intent


def route_operator_intent(
    intent: OperatorIntent | Mapping[str, object] | str,
    *,
    metadata: Mapping[str, object] | None = None,
    target_heading_delta_deg: float | None = None,
) -> HighLevelTaskRequest:
    """Translate one validated operator intent into an internal task request."""
    resolved_intent = normalize_operator_intent(
        intent,
        target_heading_delta_deg=target_heading_delta_deg,
    )
    routed_metadata = dict(metadata or {})
    if isinstance(resolved_intent, TipFreeGrowthIntent):
        routed_metadata.update(
            {
                "operator_intent": resolved_intent.intent_kind,
                "operator_intent_kind": resolved_intent.intent_kind,
                "target_heading_delta_deg": None,
            }
        )
        return HighLevelTaskRequest(
            task_kind=TIP_FREE_GROWTH,
            metadata=routed_metadata,
        )
    if isinstance(resolved_intent, TipTurnIntent):
        routed_metadata.update(
            {
                "operator_intent": resolved_intent.intent_kind,
                "operator_intent_kind": resolved_intent.intent_kind,
                "target_heading_delta_deg": float(resolved_intent.target_heading_delta_deg),
            }
        )
        return HighLevelTaskRequest(
            task_kind=TIP_TURN_AUTONOMOUS,
            metadata=routed_metadata,
        )
    raise TypeError(f"unsupported_routed_operator_intent:{type(resolved_intent)!r}")


__all__ = ["route_operator_intent"]

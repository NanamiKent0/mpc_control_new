"""Helpers for constructing and validating high-level operator intents."""

from __future__ import annotations

from collections.abc import Mapping

from ..models.operator_intent_types import OperatorIntent, TipFreeGrowthIntent, TipTurnIntent


def tip_free_growth_intent() -> TipFreeGrowthIntent:
    """Return the canonical free-growth intent object."""
    return TipFreeGrowthIntent()


def tip_turn_intent(target_heading_delta_deg: float) -> TipTurnIntent:
    """Return the canonical tip-turn intent object."""
    return TipTurnIntent(target_heading_delta_deg=float(target_heading_delta_deg))


def normalize_operator_intent(
    intent: OperatorIntent | Mapping[str, object] | str,
    *,
    target_heading_delta_deg: float | None = None,
) -> OperatorIntent:
    """Normalize an operator-intent-like input into a validated dataclass."""
    if isinstance(intent, (TipFreeGrowthIntent, TipTurnIntent)):
        return intent
    if isinstance(intent, str):
        normalized = intent.strip().upper()
        if normalized == "TIP_FREE_GROWTH":
            return tip_free_growth_intent()
        if normalized == "TIP_TURN":
            if target_heading_delta_deg is None:
                raise ValueError("operator_intent_tip_turn_requires_target_heading_delta_deg")
            return tip_turn_intent(target_heading_delta_deg)
        raise ValueError(f"operator_intent_kind_unsupported:{intent}")
    if isinstance(intent, Mapping):
        intent_kind = str(intent.get("intent_kind", "")).strip().upper()
        if intent_kind == "TIP_FREE_GROWTH":
            return tip_free_growth_intent()
        if intent_kind == "TIP_TURN":
            resolved_delta = intent.get("target_heading_delta_deg", target_heading_delta_deg)
            if resolved_delta is None:
                raise ValueError("operator_intent_tip_turn_requires_target_heading_delta_deg")
            return tip_turn_intent(float(resolved_delta))
        raise ValueError(f"operator_intent_kind_unsupported:{intent_kind or type(intent).__name__}")
    raise TypeError(f"unsupported_operator_intent_type:{type(intent)!r}")


__all__ = [
    "normalize_operator_intent",
    "tip_free_growth_intent",
    "tip_turn_intent",
]

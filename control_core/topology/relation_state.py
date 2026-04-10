"""Relation-centric state used by reusable skill templates."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Literal

RelationType = Literal["tip_joint", "joint_joint"]
DiagnosticValue = float | bool | str | None


@dataclass(slots=True)
class RelationState:
    """Debug-friendly state for one active/passive relation."""

    active_module: str
    passive_module: str
    relation_type: RelationType
    distance_mm: float | None
    orientation_error_deg: float | None
    coupled: bool | None
    observation_valid: bool
    diagnostics: dict[str, DiagnosticValue] = field(default_factory=dict)

    def pair(self) -> tuple[str, str]:
        """Return the relation endpoints in active/passive order."""
        return (self.active_module, self.passive_module)

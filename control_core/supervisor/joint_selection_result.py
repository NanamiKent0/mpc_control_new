"""Structured result emitted by the fixed-range joint selection policy."""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(slots=True)
class JointSelectionResult:
    """Scheduler-facing selection result for the current front-joint candidate."""

    selected_joint_id: str | None = None
    selected_joint_index: int | None = None
    direct_front_cooperation: bool = False
    requires_recursive_transfer: bool = False
    selection_reason: str = "uninitialized"
    diagnostics: dict[str, object] = field(default_factory=dict)

    @property
    def has_selection(self) -> bool:
        """Return whether the policy resolved one concrete joint candidate."""
        return self.selected_joint_id is not None and self.selected_joint_index is not None


__all__ = ["JointSelectionResult"]

"""Module-centric state snapshots for the Phase-3 architecture."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Literal

ModuleType = Literal["tip", "joint"]
ModuleDiagnosticValue = float | bool | str | int | None


def _float_map(values: dict[str, float | int | None] | None) -> dict[str, float]:
    """Return a float-only mapping with null entries removed."""
    cleaned: dict[str, float] = {}
    for key, value in (values or {}).items():
        if value is None:
            continue
        cleaned[str(key)] = float(value)
    return cleaned


@dataclass(slots=True)
class ModuleState:
    """Serializable module-level state used by relation skills and adapters."""

    module_id: str
    module_type: ModuleType
    dofs: dict[str, float] = field(default_factory=dict)
    velocities: dict[str, float] = field(default_factory=dict)
    attach_state: dict[str, bool | None] = field(default_factory=dict)
    metadata: dict[str, ModuleDiagnosticValue] = field(default_factory=dict)
    notes: list[str] = field(default_factory=list)


def make_tip_module_state(
    module_id: str = "tip",
    *,
    growth_mm: float | None = None,
    growth_mm_s: float | None = None,
    dofs: dict[str, float | int | None] | None = None,
    velocities: dict[str, float | int | None] | None = None,
    attach_state: dict[str, bool | None] | None = None,
    metadata: dict[str, ModuleDiagnosticValue] | None = None,
    notes: list[str] | None = None,
) -> ModuleState:
    """Build a `ModuleState` for the tip module."""
    merged_dofs = _float_map(dofs)
    merged_velocities = _float_map(velocities)
    if growth_mm is not None:
        merged_dofs["growth_mm"] = float(growth_mm)
    if growth_mm_s is not None:
        merged_velocities["growth_mm_s"] = float(growth_mm_s)
    return ModuleState(
        module_id=module_id,
        module_type="tip",
        dofs=merged_dofs,
        velocities=merged_velocities,
        attach_state=dict(attach_state or {}),
        metadata=dict(metadata or {}),
        notes=list(notes or []),
    )


def make_joint_module_state(
    module_id: str,
    *,
    crawl_mm: float | None = None,
    rotate_deg: float | None = None,
    bend_deg: float | None = None,
    crawl_mm_s: float | None = None,
    rotate_deg_s: float | None = None,
    bend_deg_s: float | None = None,
    dofs: dict[str, float | int | None] | None = None,
    velocities: dict[str, float | int | None] | None = None,
    attach_state: dict[str, bool | None] | None = None,
    metadata: dict[str, ModuleDiagnosticValue] | None = None,
    notes: list[str] | None = None,
) -> ModuleState:
    """Build a `ModuleState` for a joint module."""
    merged_dofs = _float_map(dofs)
    merged_velocities = _float_map(velocities)
    if crawl_mm is not None:
        merged_dofs["crawl_mm"] = float(crawl_mm)
    if rotate_deg is not None:
        merged_dofs["rotate_deg"] = float(rotate_deg)
    if bend_deg is not None:
        merged_dofs["bend_deg"] = float(bend_deg)
    if crawl_mm_s is not None:
        merged_velocities["crawl_mm_s"] = float(crawl_mm_s)
    if rotate_deg_s is not None:
        merged_velocities["rotate_deg_s"] = float(rotate_deg_s)
    if bend_deg_s is not None:
        merged_velocities["bend_deg_s"] = float(bend_deg_s)
    return ModuleState(
        module_id=module_id,
        module_type="joint",
        dofs=merged_dofs,
        velocities=merged_velocities,
        attach_state=dict(attach_state or {}),
        metadata=dict(metadata or {}),
        notes=list(notes or []),
    )

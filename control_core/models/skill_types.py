"""Lightweight skill dataclasses used across the Phase-5 scaffold."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from .module_state import ModuleState
from ..topology.relation_state import DiagnosticValue, RelationState, RelationType


@dataclass(slots=True)
class SkillSpec:
    """Declarative configuration for one relation-skill execution."""

    skill_key: str
    active_module: str
    passive_module: str
    relation_type: RelationType
    distance_done_mm: float = 0.0
    orientation_done_deg: float | None = None
    solver_dt: float = 0.1
    limits: dict[str, float] = field(default_factory=dict)
    config: dict[str, DiagnosticValue] = field(default_factory=dict)
    params: dict[str, object] = field(default_factory=dict)
    metadata: dict[str, DiagnosticValue] = field(default_factory=dict)

    def __post_init__(self) -> None:
        """Normalize the Phase-3 generic parameter surface."""
        normalized = dict(self.params or {})
        normalized.setdefault("distance_done_mm", float(self.distance_done_mm))
        normalized.setdefault("solver_dt", float(self.solver_dt))
        if self.orientation_done_deg is not None:
            normalized.setdefault("orientation_done_deg", float(self.orientation_done_deg))
        self.params = normalized

    def pair_key(self) -> tuple[str, str]:
        """Return the requested active/passive pair."""
        return (self.active_module, self.passive_module)

    def get_param(self, name: str, default: object | None = None) -> object | None:
        """Return a generic parameter with legacy top-level fallback."""
        if name in self.params:
            return self.params[name]
        legacy_values: dict[str, object | None] = {
            "distance_done_mm": self.distance_done_mm,
            "orientation_done_deg": self.orientation_done_deg,
            "solver_dt": self.solver_dt,
        }
        if name in legacy_values and legacy_values[name] is not None:
            return legacy_values[name]
        return default

    def float_param(self, name: str, default: float | None = None) -> float | None:
        """Return one float-like parameter with nullable fallback."""
        value = self.get_param(name, default)
        if value is None:
            return default
        try:
            return float(value)
        except (TypeError, ValueError):
            return default

    def bool_param(self, name: str, default: bool = False) -> bool:
        """Return one bool-like parameter with conservative fallback."""
        value = self.get_param(name, default)
        if isinstance(value, bool):
            return value
        if value is None:
            return default
        if isinstance(value, str):
            normalized = value.strip().lower()
            if normalized in {"1", "true", "yes", "on"}:
                return True
            if normalized in {"0", "false", "no", "off"}:
                return False
            return default
        if isinstance(value, (int, float)):
            return bool(value)
        return default

    def list_param(self, name: str, default: tuple[str, ...] = ()) -> tuple[str, ...]:
        """Return one list-like parameter normalized as a string tuple."""
        value = self.get_param(name, None)
        if value is None:
            return default
        if isinstance(value, str):
            return (value,)
        if isinstance(value, (list, tuple, set)):
            return tuple(str(item) for item in value)
        return default

    def merged_params(self) -> dict[str, object]:
        """Return a copy of the normalized generic parameter mapping."""
        return dict(self.params)


@dataclass(slots=True)
class SkillDescriptor:
    """Descriptor stored by the skill registry for one registered skill."""

    skill_key: str
    metadata: dict[str, object] = field(default_factory=dict)
    registry_source: str | None = None


@dataclass(slots=True)
class SkillResolutionResult:
    """Structured skill-registry lookup result used by adapters/schedulers."""

    skill_key: str
    found: bool
    skill_instance: Any | None = None
    descriptor: SkillDescriptor | None = None
    registry_source: str | None = None
    error: str | None = None

    @property
    def metadata(self) -> dict[str, object]:
        """Return resolved descriptor metadata with a stable empty fallback."""
        if self.descriptor is None:
            return {}
        return dict(self.descriptor.metadata)

    @property
    def resolved_skill_key(self) -> str | None:
        """Return the resolved skill key when the lookup succeeded."""
        if self.descriptor is None:
            return None
        return self.descriptor.skill_key


@dataclass(slots=True)
class SkillCheckResult:
    """Outcome of skill precondition checks."""

    passed: bool
    blocking_reason: str | None = None
    blocked_by_topology: bool = False
    not_fully_supported: bool = False
    block_reason: str | None = None
    used_context_topology: bool = False
    topology_snapshot: dict[str, DiagnosticValue] = field(default_factory=dict)
    frontier_snapshot: dict[str, DiagnosticValue] = field(default_factory=dict)
    support_snapshot: dict[str, DiagnosticValue] = field(default_factory=dict)
    diagnostics: dict[str, DiagnosticValue] = field(default_factory=dict)
    notes: list[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        """Compatibility alias used by higher-level schedulers."""
        return self.passed


@dataclass(slots=True)
class SkillCompletionResult:
    """Outcome of skill completion checks."""

    done: bool
    completion_reason: str | None = None
    blocked_by_topology: bool = False
    block_reason: str | None = None
    used_context_topology: bool = False
    topology_snapshot: dict[str, DiagnosticValue] = field(default_factory=dict)
    frontier_snapshot: dict[str, DiagnosticValue] = field(default_factory=dict)
    support_snapshot: dict[str, DiagnosticValue] = field(default_factory=dict)
    diagnostics: dict[str, DiagnosticValue] = field(default_factory=dict)
    notes: list[str] = field(default_factory=list)


@dataclass(slots=True)
class PrimitiveReference:
    """One skill-produced primitive reference for a module axis."""

    module_id: str
    axis: str
    reference_kind: str
    reference_value: float
    units: str
    primary: bool
    semantic: str
    primitive_name: str | None = None
    target_value: float | None = None
    metadata: dict[str, DiagnosticValue] = field(default_factory=dict)


@dataclass(slots=True)
class SkillExecutionResult:
    """Debug-friendly output of one skill execution."""

    skill_spec: SkillSpec
    relation_state: RelationState
    skill_key: str = ""
    active_module: str = ""
    passive_module: str = ""
    module_states: dict[str, ModuleState] = field(default_factory=dict)
    preconditions: SkillCheckResult = field(default_factory=lambda: SkillCheckResult(passed=False))
    completion: SkillCompletionResult = field(default_factory=lambda: SkillCompletionResult(done=False))
    primitive_references: list[PrimitiveReference] = field(default_factory=list)
    selected_primitives: list[str] = field(default_factory=list)
    status: str = "pending"
    blocked_by_topology: bool = False
    not_fully_supported: bool = False
    block_reason: str | None = None
    used_context_topology: bool = False
    topology_snapshot: dict[str, DiagnosticValue] = field(default_factory=dict)
    frontier_snapshot: dict[str, DiagnosticValue] = field(default_factory=dict)
    support_snapshot: dict[str, DiagnosticValue] = field(default_factory=dict)
    diagnostics: dict[str, DiagnosticValue] = field(default_factory=dict)
    notes: list[str] = field(default_factory=list)

    def __post_init__(self) -> None:
        """Backfill scheduler-friendly aliases from the bound skill spec."""
        if not self.skill_key:
            self.skill_key = self.skill_spec.skill_key
        if not self.active_module:
            self.active_module = self.skill_spec.active_module
        if not self.passive_module:
            self.passive_module = self.skill_spec.passive_module
        if not self.selected_primitives:
            seen: set[str] = set()
            for reference in self.primitive_references:
                primitive_name = reference.primitive_name or f"{reference.module_id}:{reference.axis}"
                if primitive_name in seen:
                    continue
                seen.add(primitive_name)
                self.selected_primitives.append(primitive_name)

    def pair(self) -> tuple[str, str]:
        """Return the active/passive pair recorded by this result."""
        return (self.active_module, self.passive_module)

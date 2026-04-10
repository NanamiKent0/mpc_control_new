"""Runtime-facing observation dataclasses used by providers and sessions."""

from __future__ import annotations

from dataclasses import dataclass, field

from ..control_core.topology.relation_state import RelationType

RuntimeObservationScalar = bool | float | int | str | None


def module_key(module_id: str) -> str:
    """Return the stable dictionary key for one module observation."""
    return str(module_id)


def pair_key(active_module: str, passive_module: str) -> str:
    """Return the stable dictionary key for one directed pair observation."""
    return f"{active_module}->{passive_module}"


def _coerce_float_map(values: dict[str, float | int] | None) -> dict[str, float]:
    """Return a float-normalized mapping with invalid entries removed."""
    normalized: dict[str, float] = {}
    for key, value in (values or {}).items():
        try:
            normalized[str(key)] = float(value)
        except (TypeError, ValueError):
            continue
    return normalized


@dataclass(slots=True)
class ModuleObservation:
    """Raw per-module runtime observation emitted by sim/live providers."""

    module_id: str
    module_type: str | None = None
    dofs: dict[str, float] = field(default_factory=dict)
    velocities: dict[str, float] = field(default_factory=dict)
    attach_state: dict[str, bool | None] = field(default_factory=dict)
    diagnostics: dict[str, object] = field(default_factory=dict)
    source_name: str = "runtime"

    def __post_init__(self) -> None:
        """Normalize dictionaries into predictable serializable shapes."""
        self.module_id = module_key(self.module_id)
        self.module_type = None if self.module_type is None else str(self.module_type)
        self.dofs = _coerce_float_map(self.dofs)
        self.velocities = _coerce_float_map(self.velocities)
        self.attach_state = {str(key): value for key, value in dict(self.attach_state).items()}
        self.diagnostics = dict(self.diagnostics)
        self.source_name = str(self.source_name)


@dataclass(slots=True)
class PairObservation:
    """Raw runtime observation for one active/passive relation pair."""

    active_module: str
    passive_module: str
    relation_type: RelationType | str | None = None
    distance_mm: float | None = None
    orientation_error_deg: float | None = None
    coupled: bool | None = None
    observation_valid: bool = False
    diagnostics: dict[str, object] = field(default_factory=dict)
    source_name: str = "runtime"

    def __post_init__(self) -> None:
        """Normalize scalar metadata while preserving graceful degradation."""
        self.active_module = module_key(self.active_module)
        self.passive_module = module_key(self.passive_module)
        self.relation_type = None if self.relation_type is None else str(self.relation_type)
        self.distance_mm = _coerce_optional_float(self.distance_mm)
        self.orientation_error_deg = _coerce_optional_float(self.orientation_error_deg)
        self.coupled = _coerce_optional_bool(self.coupled)
        self.observation_valid = bool(self.observation_valid)
        self.diagnostics = dict(self.diagnostics)
        self.source_name = str(self.source_name)

    def key(self) -> str:
        """Return the stable pair key for this observation."""
        return pair_key(self.active_module, self.passive_module)


@dataclass(slots=True)
class RuntimeObservationFrame:
    """Unified runtime input object for the new architecture main path."""

    timestamp_ns: int
    module_observations: dict[str, ModuleObservation] = field(default_factory=dict)
    pair_observations: dict[str, PairObservation] = field(default_factory=dict)
    topology_hint: dict[str, object] | None = None
    metadata: dict[str, object] = field(default_factory=dict)

    def __post_init__(self) -> None:
        """Normalize keys so providers may pass partially shaped mappings."""
        self.timestamp_ns = int(self.timestamp_ns)
        normalized_modules: dict[str, ModuleObservation] = {}
        for key, observation in dict(self.module_observations).items():
            resolved = observation if isinstance(observation, ModuleObservation) else ModuleObservation(**dict(observation))
            normalized_modules[module_key(resolved.module_id or str(key))] = resolved
        normalized_pairs: dict[str, PairObservation] = {}
        for key, observation in dict(self.pair_observations).items():
            resolved = observation if isinstance(observation, PairObservation) else PairObservation(**dict(observation))
            normalized_pairs[pair_key(resolved.active_module, resolved.passive_module)] = resolved
        self.module_observations = normalized_modules
        self.pair_observations = normalized_pairs
        self.topology_hint = None if self.topology_hint is None else dict(self.topology_hint)
        self.metadata = dict(self.metadata)

    def get_module_observation(self, module_id: str) -> ModuleObservation | None:
        """Return one module observation using the stable module key."""
        return self.module_observations.get(module_key(module_id))

    def get_pair_observation(self, active_module: str, passive_module: str) -> PairObservation | None:
        """Return one directed pair observation with a reverse-order fallback."""
        direct = self.pair_observations.get(pair_key(active_module, passive_module))
        if direct is not None:
            return direct
        reverse = self.pair_observations.get(pair_key(passive_module, active_module))
        if reverse is None:
            return None
        return PairObservation(
            active_module=active_module,
            passive_module=passive_module,
            relation_type=reverse.relation_type,
            distance_mm=reverse.distance_mm,
            orientation_error_deg=reverse.orientation_error_deg,
            coupled=reverse.coupled,
            observation_valid=reverse.observation_valid,
            diagnostics={
                **reverse.diagnostics,
                "pair_reordered_from": reverse.key(),
                "pair_reordered_to": pair_key(active_module, passive_module),
            },
            source_name=reverse.source_name,
        )


def _coerce_optional_float(value: object) -> float | None:
    """Convert a scalar-like value into float while preserving missing data."""
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _coerce_optional_bool(value: object) -> bool | None:
    """Convert a scalar-like value into bool while preserving missing data."""
    if isinstance(value, bool):
        return value
    if value is None:
        return None
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {"1", "true", "yes", "on"}:
            return True
        if normalized in {"0", "false", "no", "off"}:
            return False
        return None
    if isinstance(value, (int, float)):
        return bool(value)
    return None


__all__ = [
    "ModuleObservation",
    "PairObservation",
    "RuntimeObservationFrame",
    "RuntimeObservationScalar",
    "module_key",
    "pair_key",
]

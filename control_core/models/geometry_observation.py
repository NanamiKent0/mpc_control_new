"""Geometry-observation boundary between runtime observations and relation state."""

from __future__ import annotations

from dataclasses import dataclass, field

from ..topology.relation_state import DiagnosticValue, RelationState, RelationType


@dataclass(slots=True)
class GeometryObservation:
    """Normalized geometry observation derived from raw runtime or legacy inputs."""

    observation_kind: str
    pair: tuple[str, str]
    relation_type: RelationType
    distance_mm: float | None
    orientation_error_deg: float | None
    coupled: bool | None
    observation_valid: bool
    source_schema: str = "legacy.geometry.v1"
    source_fields: dict[str, str | None] = field(default_factory=dict)
    metrics: dict[str, DiagnosticValue] = field(default_factory=dict)
    notes: list[str] = field(default_factory=list)
    diagnostics: dict[str, DiagnosticValue] = field(default_factory=dict)
    source_name: str | None = None
    provider_source: str | None = None
    frame_origin: str | None = None
    observation_origin: str = "legacy_estimate"
    upstream_observation_kind: str | None = None
    frame_timestamp_ns: int | None = None

    @property
    def active_module(self) -> str:
        """Return the active module identifier."""
        return self.pair[0]

    @property
    def passive_module(self) -> str:
        """Return the passive module identifier."""
        return self.pair[1]

    @property
    def pair_key(self) -> str:
        """Return the stable active/passive pair key."""
        return f"{self.active_module}->{self.passive_module}"

    def to_relation_diagnostics(
        self,
        extra_diagnostics: dict[str, DiagnosticValue] | None = None,
    ) -> dict[str, DiagnosticValue]:
        """Flatten observation metadata into relation-state diagnostics."""
        source_field_diagnostics = {
            f"source_field_{name}": value
            for name, value in self.source_fields.items()
        }
        metric_diagnostics = {
            f"observation_metric_{name}": value
            for name, value in self.metrics.items()
        }
        return {
            "geometry_observation_kind": self.observation_kind,
            "geometry_source_schema": self.source_schema,
            "geometry_source_name": self.source_name,
            "geometry_provider_source": self.provider_source,
            "geometry_frame_origin": self.frame_origin,
            "geometry_observation_origin": self.observation_origin,
            "geometry_upstream_observation_kind": self.upstream_observation_kind,
            "geometry_frame_timestamp_ns": (
                None if self.frame_timestamp_ns is None else float(self.frame_timestamp_ns)
            ),
            "observation_valid": self.observation_valid,
            "observation_pair": self.pair_key,
            "observation_notes": " | ".join(self.notes) or None,
            **source_field_diagnostics,
            **metric_diagnostics,
            **dict(self.diagnostics),
            **dict(extra_diagnostics or {}),
        }

    def to_relation_state(
        self,
        extra_diagnostics: dict[str, DiagnosticValue] | None = None,
    ) -> RelationState:
        """Convert the observation into the skill-facing relation snapshot."""
        return RelationState(
            active_module=self.active_module,
            passive_module=self.passive_module,
            relation_type=self.relation_type,
            distance_mm=self.distance_mm,
            orientation_error_deg=self.orientation_error_deg,
            coupled=self.coupled,
            observation_valid=self.observation_valid,
            diagnostics=self.to_relation_diagnostics(extra_diagnostics=extra_diagnostics),
        )

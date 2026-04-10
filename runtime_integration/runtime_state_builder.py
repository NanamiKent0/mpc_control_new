"""Builders that translate runtime frames into skill-facing state models."""

from __future__ import annotations

from copy import deepcopy

from ..control_core.models.geometry_observation import GeometryObservation
from ..control_core.models.module_state import ModuleState, make_joint_module_state, make_tip_module_state
from ..control_core.topology.chain_topology import ChainTopology
from ..control_core.topology.relation_state import DiagnosticValue, RelationState, RelationType
from ..controllers.adapters.legacy_extractors import infer_module_type, infer_relation_type, is_joint_module
from .observation_types import ModuleObservation, PairObservation, RuntimeObservationFrame, pair_key

RUNTIME_STATE_BUILDER_SOURCE = "runtime_state_builder"
RUNTIME_GEOMETRY_SCHEMA = "runtime.frame.v1"


def build_module_state_map(frame: RuntimeObservationFrame) -> dict[str, ModuleState]:
    """Build normalized module states for every observed runtime module."""
    return {
        module_id: _build_module_state_from_observation(observation)
        for module_id, observation in frame.module_observations.items()
    }


def build_module_state_for_module(frame: RuntimeObservationFrame, module_id: str) -> ModuleState:
    """Return one module state, creating a placeholder when the module is absent."""
    observation = frame.get_module_observation(module_id)
    if observation is None:
        return _placeholder_module_state(module_id, note="runtime_module_observation_missing")
    return _build_module_state_from_observation(observation)


def build_geometry_observation_from_frame(
    frame: RuntimeObservationFrame,
    *,
    active_module: str,
    passive_module: str,
    relation_type: RelationType | None = None,
) -> GeometryObservation:
    """Build a geometry observation directly from the runtime-facing frame."""
    resolved_relation_type = _resolved_relation_type(
        active_module=active_module,
        passive_module=passive_module,
        relation_type=relation_type,
    )
    observation = frame.get_pair_observation(active_module, passive_module)
    if observation is None:
        return GeometryObservation(
            observation_kind=resolved_relation_type,
            pair=(active_module, passive_module),
            relation_type=resolved_relation_type,
            distance_mm=None,
            orientation_error_deg=None,
            coupled=None,
            observation_valid=False,
            source_schema=RUNTIME_GEOMETRY_SCHEMA,
            source_name=str(frame.metadata.get("source_name", "runtime_frame")),
            provider_source=_frame_provider_source(frame),
            frame_origin=_frame_origin(frame),
            source_fields={},
            metrics={"missing_pair_observation": True},
            notes=["pair_observation_missing"],
            diagnostics={
                "pair_key": pair_key(active_module, passive_module),
                "state_builder_source": RUNTIME_STATE_BUILDER_SOURCE,
                "provider_source": _frame_provider_source(frame),
                "frame_origin": _frame_origin(frame),
                "runtime_frame_timestamp_ns": _diagnostic_value(frame.timestamp_ns),
            },
            observation_origin="runtime_frame",
            upstream_observation_kind="pair_observation_missing",
            frame_timestamp_ns=frame.timestamp_ns,
        )
    return _geometry_observation_from_pair_observation(
        observation,
        frame=frame,
        relation_type=resolved_relation_type,
    )


def build_relation_state_from_frame(
    frame: RuntimeObservationFrame,
    *,
    active_module: str,
    passive_module: str,
    relation_type: RelationType | None = None,
) -> RelationState:
    """Build a relation state directly from the runtime-facing frame."""
    geometry = build_geometry_observation_from_frame(
        frame,
        active_module=active_module,
        passive_module=passive_module,
        relation_type=relation_type,
    )
    return geometry.to_relation_state(
        extra_diagnostics={
            "state_builder_source": RUNTIME_STATE_BUILDER_SOURCE,
            "runtime_pair_key": pair_key(active_module, passive_module),
            "runtime_frame_timestamp_ns": _diagnostic_value(frame.timestamp_ns),
        }
    )


def build_topology_from_frame(
    frame: RuntimeObservationFrame,
    *,
    fallback_topology: ChainTopology | None = None,
) -> ChainTopology:
    """Build a chain topology from runtime hints while preserving a fallback shell."""
    topology = deepcopy(fallback_topology) if fallback_topology is not None else ChainTopology()
    topology_hint = dict(frame.topology_hint or {})
    ordered_modules = topology_hint.get("ordered_modules")
    if isinstance(ordered_modules, (list, tuple)):
        topology.ordered_modules = [str(module_id) for module_id in ordered_modules]
    elif not topology.ordered_modules:
        topology.ordered_modules = list(frame.module_observations)

    for left, right in _edge_pairs(topology_hint.get("blocked_edges")):
        topology.set_blocked_edge(left, right, True)
    for module_id in _module_set(topology_hint.get("support_modules")):
        topology.set_support_module(module_id, True)
    for module_id in _module_set(topology_hint.get("grounded_modules")):
        topology.grounded_modules.add(module_id)

    active_frontier = topology_hint.get("active_frontier")
    if isinstance(active_frontier, (list, tuple)) and len(active_frontier) == 2:
        topology.set_active_frontier(str(active_frontier[0]), str(active_frontier[1]))

    for observation in frame.pair_observations.values():
        if observation.coupled is None:
            continue
        topology.set_coupled(observation.active_module, observation.passive_module, observation.coupled)
    return topology


def _build_module_state_from_observation(observation: ModuleObservation) -> ModuleState:
    """Convert one runtime module observation into a normalized module state."""
    resolved_module_type = infer_module_type(observation.module_id, observed_module_type=observation.module_type)
    metadata = {
        "module_source_name": observation.source_name,
        "observed_module_type": observation.module_type,
        "state_builder_source": RUNTIME_STATE_BUILDER_SOURCE,
    }
    metadata.update(_flatten_diagnostics(observation.diagnostics))
    notes: list[str] = []
    if resolved_module_type == "joint":
        return make_joint_module_state(
            module_id=observation.module_id,
            dofs=observation.dofs,
            velocities=observation.velocities,
            attach_state=observation.attach_state,
            metadata=metadata,
            notes=notes,
        )
    if resolved_module_type != "tip":
        notes.append("module_type_inferred_as_tip_fallback")
    return make_tip_module_state(
        module_id=observation.module_id,
        dofs=observation.dofs,
        velocities=observation.velocities,
        attach_state=observation.attach_state,
        metadata=metadata,
        notes=notes,
    )


def _placeholder_module_state(module_id: str, *, note: str) -> ModuleState:
    """Return a minimal placeholder module state when runtime data is absent."""
    resolved_module_type = infer_module_type(module_id)
    metadata = {
        "state_builder_source": RUNTIME_STATE_BUILDER_SOURCE,
        "runtime_module_placeholder": True,
    }
    if resolved_module_type == "joint":
        return make_joint_module_state(
            module_id=module_id,
            metadata=metadata,
            notes=[note],
        )
    return make_tip_module_state(
        module_id=module_id,
        metadata=metadata,
        notes=[note],
    )


def _geometry_observation_from_pair_observation(
    observation: PairObservation,
    *,
    frame: RuntimeObservationFrame,
    relation_type: RelationType,
) -> GeometryObservation:
    """Convert one raw pair observation into the geometry boundary object."""
    pair_name = pair_key(observation.active_module, observation.passive_module)
    source_fields = {
        "distance_mm": f"runtime_frame.pair_observations.{pair_name}.distance_mm",
        "orientation_error_deg": f"runtime_frame.pair_observations.{pair_name}.orientation_error_deg",
        "coupled": f"runtime_frame.pair_observations.{pair_name}.coupled",
    }
    diagnostics = {
        "pair_key": pair_name,
        "pair_source_name": observation.source_name,
        "state_builder_source": RUNTIME_STATE_BUILDER_SOURCE,
        "runtime_frame_timestamp_ns": _diagnostic_value(frame.timestamp_ns),
        **_flatten_diagnostics(observation.diagnostics),
    }
    return GeometryObservation(
        observation_kind=relation_type,
        pair=(observation.active_module, observation.passive_module),
        relation_type=relation_type,
        distance_mm=observation.distance_mm,
        orientation_error_deg=observation.orientation_error_deg,
        coupled=observation.coupled,
        observation_valid=bool(observation.observation_valid),
        source_schema=RUNTIME_GEOMETRY_SCHEMA,
        source_name=observation.source_name,
        provider_source=_frame_provider_source(frame),
        frame_origin=_frame_origin(frame),
        source_fields=source_fields,
        metrics={
            "distance_mm": observation.distance_mm,
            "orientation_error_deg": observation.orientation_error_deg,
            "coupled": observation.coupled,
        },
        notes=[] if observation.observation_valid else ["pair_observation_invalid"],
        diagnostics=diagnostics,
        observation_origin="runtime_frame",
        upstream_observation_kind="pair_observation",
        frame_timestamp_ns=frame.timestamp_ns,
    )


def _resolved_relation_type(
    *,
    active_module: str,
    passive_module: str,
    relation_type: RelationType | None,
) -> RelationType:
    """Resolve the relation type for one active/passive pair."""
    if relation_type in {"tip_joint", "joint_joint"}:
        return relation_type
    inferred = infer_relation_type(active_module, passive_module)
    if inferred is not None:
        return inferred
    if active_module == "tip" or passive_module == "tip":
        return "tip_joint"
    return "joint_joint"


def _edge_pairs(value: object) -> list[tuple[str, str]]:
    """Normalize topology-hint edge collections."""
    if not isinstance(value, (list, tuple, set)):
        return []
    pairs: list[tuple[str, str]] = []
    for item in value:
        if isinstance(item, (list, tuple)) and len(item) == 2:
            pairs.append((str(item[0]), str(item[1])))
    return pairs


def _module_set(value: object) -> set[str]:
    """Normalize a topology-hint module collection."""
    if not isinstance(value, (list, tuple, set)):
        return set()
    return {str(module_id) for module_id in value}


def _frame_provider_source(frame: RuntimeObservationFrame) -> str | None:
    """Return the provider source recorded on a runtime frame."""
    value = frame.metadata.get("provider_source")
    if value is None:
        return None
    return str(value)


def _frame_origin(frame: RuntimeObservationFrame) -> str | None:
    """Return the frame origin recorded on a runtime frame."""
    value = frame.metadata.get("frame_origin")
    if value is None:
        return None
    return str(value)


def _flatten_diagnostics(values: dict[str, object] | None) -> dict[str, DiagnosticValue]:
    """Flatten arbitrary diagnostics into relation-safe scalar values."""
    flattened: dict[str, DiagnosticValue] = {}
    for key, value in dict(values or {}).items():
        flattened[str(key)] = _diagnostic_value(value)
    return flattened


def _diagnostic_value(value: object) -> DiagnosticValue:
    """Convert arbitrary values into supported relation diagnostic scalars."""
    if value is None or isinstance(value, (str, float, bool)):
        return value
    if isinstance(value, int):
        return float(value)
    return str(value)


__all__ = [
    "RUNTIME_GEOMETRY_SCHEMA",
    "RUNTIME_STATE_BUILDER_SOURCE",
    "build_geometry_observation_from_frame",
    "build_module_state_for_module",
    "build_module_state_map",
    "build_relation_state_from_frame",
    "build_topology_from_frame",
]

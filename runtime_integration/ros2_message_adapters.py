"""Adapters between ROS2-style payloads and runtime-session boundary objects."""

from __future__ import annotations

from typing import Any

from ..control_core.models.runtime_bridge_types import SchedulerDispatchEnvelope
from .observation_types import ModuleObservation, PairObservation, RuntimeObservationFrame
from .ros2_interfaces import (
    Ros2CommandPayload,
    Ros2ModuleStatePayload,
    Ros2ObservationPayload,
    Ros2PairStatePayload,
    Ros2PrimitiveCommandPayload,
)


def runtime_frame_from_ros2_message(
    message: Ros2ObservationPayload | dict[str, object] | object,
    *,
    source_name: str = "ros2_observation_provider",
    fallback_topology_hint: dict[str, object] | None = None,
) -> RuntimeObservationFrame:
    """Convert one ROS2-style observation payload into a runtime observation frame."""
    payload = normalize_ros2_observation_payload(message)
    modules = {
        module.module_id: ModuleObservation(
            module_id=module.module_id,
            module_type=module.module_type,
            dofs=module.dofs,
            velocities=module.velocities,
            attach_state=module.attach_state,
            diagnostics=module.diagnostics,
            source_name=source_name,
        )
        for module in payload.modules
    }
    pairs = {
        f"{pair.active_module}->{pair.passive_module}": PairObservation(
            active_module=pair.active_module,
            passive_module=pair.passive_module,
            relation_type=pair.relation_type,
            distance_mm=pair.distance_mm,
            orientation_error_deg=pair.orientation_error_deg,
            coupled=pair.coupled,
            observation_valid=pair.observation_valid,
            diagnostics=pair.diagnostics,
            source_name=source_name,
        )
        for pair in payload.pairs
    }
    topology_hint = dict(fallback_topology_hint or {})
    topology_hint.update(payload.topology_hint)
    return RuntimeObservationFrame(
        timestamp_ns=payload.timestamp_ns,
        module_observations=modules,
        pair_observations=pairs,
        topology_hint=topology_hint,
        metadata={
            "source_name": source_name,
            "provider_source": "ros2",
            "frame_origin": "ros2",
            "ros2_diagnostics": dict(payload.diagnostics),
        },
    )


def envelope_to_ros2_command_payload(
    envelope: SchedulerDispatchEnvelope,
    *,
    source_name: str = "ros2_command_dispatcher",
) -> Ros2CommandPayload:
    """Convert one scheduler dispatch envelope into a ROS2-side command payload."""
    scheduled = envelope.scheduled_command
    if scheduled is None:
        return Ros2CommandPayload(
            graph_id=envelope.scheduler_state.graph_id,
            node_id=envelope.scheduler_state.current_node_id,
            skill_key=None,
            noop_requested=True,
            diagnostics={
                "source_name": source_name,
                "dispatch_ready": envelope.dispatch_ready,
                "scheduler_input_source": envelope.diagnostics.get("scheduler_input_source"),
            },
        )

    commands = [
        Ros2PrimitiveCommandPayload(
            module_id=reference.module_id,
            primitive_name=reference.primitive_name or reference.axis,
            axis=reference.axis,
            reference_kind=reference.reference_kind,
            reference_value=float(reference.reference_value),
            units=reference.units,
            semantic=reference.semantic,
            primary=bool(reference.primary),
            target_value=None if reference.target_value is None else float(reference.target_value),
            diagnostics=dict(reference.metadata),
        )
        for reference in scheduled.primitive_references
    ]
    noop_requested = not commands
    return Ros2CommandPayload(
        graph_id=scheduled.graph_id,
        node_id=scheduled.node_id,
        skill_key=scheduled.skill_key,
        noop_requested=noop_requested,
        commands=commands,
        topology_snapshot=dict(scheduled.topology_snapshot),
        frontier_snapshot=dict(scheduled.frontier_snapshot),
        support_snapshot=dict(scheduled.support_snapshot),
        diagnostics={
            "source_name": source_name,
            "scheduler_input_source": envelope.diagnostics.get("scheduler_input_source"),
            "dispatch_ready": envelope.dispatch_ready,
            "input_source": envelope.input_source,
            "dispatch_target": envelope.dispatch_target or "ros2",
            "provider_kind": envelope.provider_kind,
            "dispatcher_kind": envelope.dispatcher_kind,
            "pair": f"{scheduled.active_module}->{scheduled.passive_module}",
            "selected_primitives": list(scheduled.selected_primitives),
            **dict(scheduled.diagnostics),
        },
    )


def normalize_ros2_observation_payload(
    payload: Ros2ObservationPayload | dict[str, object] | object,
) -> Ros2ObservationPayload:
    """Normalize a ROS2 observation payload from a dataclass, dict, or simple object."""
    if isinstance(payload, Ros2ObservationPayload):
        return payload
    modules_raw = _read_value(payload, "modules", default=[])
    pairs_raw = _read_value(payload, "pairs", default=[])
    diagnostics = _coerce_dict(_read_value(payload, "diagnostics", default={}))
    topology_hint = _coerce_dict(_read_value(payload, "topology_hint", default={}))
    modules = [normalize_ros2_module_payload(module) for module in _coerce_sequence(modules_raw)]
    pairs = [normalize_ros2_pair_payload(pair) for pair in _coerce_sequence(pairs_raw)]
    return Ros2ObservationPayload(
        timestamp_ns=int(_read_value(payload, "timestamp_ns", default=0)),
        modules=modules,
        pairs=pairs,
        topology_hint=topology_hint,
        diagnostics=diagnostics,
    )


def normalize_ros2_module_payload(
    payload: Ros2ModuleStatePayload | dict[str, object] | object,
) -> Ros2ModuleStatePayload:
    """Normalize one ROS2 module payload."""
    if isinstance(payload, Ros2ModuleStatePayload):
        return payload
    return Ros2ModuleStatePayload(
        module_id=str(_read_value(payload, "module_id", default="unknown")),
        module_type=_optional_str(_read_value(payload, "module_type", default=None)),
        dofs=_coerce_float_map(_read_value(payload, "dofs", default={})),
        velocities=_coerce_float_map(_read_value(payload, "velocities", default={})),
        attach_state=_coerce_attach_state(_read_value(payload, "attach_state", default={})),
        diagnostics=_coerce_dict(_read_value(payload, "diagnostics", default={})),
    )


def normalize_ros2_pair_payload(
    payload: Ros2PairStatePayload | dict[str, object] | object,
) -> Ros2PairStatePayload:
    """Normalize one ROS2 pair payload."""
    if isinstance(payload, Ros2PairStatePayload):
        return payload
    return Ros2PairStatePayload(
        active_module=str(_read_value(payload, "active_module", default="unknown")),
        passive_module=str(_read_value(payload, "passive_module", default="unknown")),
        relation_type=_optional_str(_read_value(payload, "relation_type", default=None)),
        distance_mm=_optional_float(_read_value(payload, "distance_mm", default=None)),
        orientation_error_deg=_optional_float(_read_value(payload, "orientation_error_deg", default=None)),
        coupled=_optional_bool(_read_value(payload, "coupled", default=None)),
        observation_valid=bool(_read_value(payload, "observation_valid", default=False)),
        diagnostics=_coerce_dict(_read_value(payload, "diagnostics", default={})),
    )


def _read_value(source: Any, key: str, *, default: Any) -> Any:
    """Read a value from a dict-like or attribute-style object."""
    if isinstance(source, dict):
        return source.get(key, default)
    return getattr(source, key, default)


def _coerce_sequence(value: object) -> list[object]:
    """Normalize a sequence-like object into a list."""
    if isinstance(value, list):
        return value
    if isinstance(value, tuple):
        return list(value)
    return []


def _coerce_dict(value: object) -> dict[str, object]:
    """Normalize a dictionary-like object into a plain dict."""
    if isinstance(value, dict):
        return {str(key): item for key, item in value.items()}
    return {}


def _coerce_float_map(value: object) -> dict[str, float]:
    """Normalize a float-like mapping."""
    if not isinstance(value, dict):
        return {}
    normalized: dict[str, float] = {}
    for key, item in value.items():
        try:
            normalized[str(key)] = float(item)
        except (TypeError, ValueError):
            continue
    return normalized


def _coerce_attach_state(value: object) -> dict[str, bool | None]:
    """Normalize an attach-state mapping."""
    if not isinstance(value, dict):
        return {}
    return {str(key): _optional_bool(item) for key, item in value.items()}


def _optional_float(value: object) -> float | None:
    """Convert a scalar-like value into float while preserving missing data."""
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _optional_bool(value: object) -> bool | None:
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


def _optional_str(value: object) -> str | None:
    """Convert a value into an optional string."""
    if value is None:
        return None
    return str(value)

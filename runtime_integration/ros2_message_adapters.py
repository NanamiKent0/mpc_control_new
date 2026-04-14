"""Adapters between ROS2-style payloads and runtime-session boundary objects."""

from __future__ import annotations

from dataclasses import asdict, is_dataclass
import json
from typing import Any

from ..control_core.models.runtime_bridge_types import SchedulerDispatchEnvelope
from .common.encoder_protocol import counts_to_physical, physical_to_counts
from .observation_types import ModuleObservation, PairObservation, RuntimeObservationFrame
from .ros2_interfaces import (
    GUI_ROS2_TOPIC_BINDINGS,
    GUI_ROS2_TOPIC_NAMESPACES,
    GuiRos2MotorCommandPayload,
    GuiRos2MotorFeedbackPayload,
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
            dispatch_target=envelope.dispatch_target or "ros2",
            noop_requested=True,
            diagnostics={
                "source_name": source_name,
                "dispatch_ready": envelope.dispatch_ready,
                "scheduler_input_source": envelope.diagnostics.get("scheduler_input_source"),
                "bridge_source": envelope.bridge_source,
                "command_summary": dict(envelope.command_summary),
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
            diagnostics={"command_index": index, **dict(reference.metadata)},
        )
        for index, reference in enumerate(scheduled.primitive_references)
    ]
    noop_requested = not commands
    return Ros2CommandPayload(
        graph_id=scheduled.graph_id,
        node_id=scheduled.node_id,
        skill_key=scheduled.skill_key,
        dispatch_target=envelope.dispatch_target or "ros2",
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
            "provider_hint": envelope.provider_hint,
            "dispatcher_hint": envelope.dispatcher_hint,
            "bridge_source": envelope.bridge_source,
            "pair": f"{scheduled.active_module}->{scheduled.passive_module}",
            "selected_primitives": list(scheduled.selected_primitives),
            "command_summary": dict(envelope.command_summary or scheduled.command_summary),
            **dict(scheduled.diagnostics),
        },
    )


def normalize_gui_ros2_command_payload(
    payload: GuiRos2MotorCommandPayload | dict[str, object] | object,
    *,
    namespace: str | None = None,
    command_topic: str | None = None,
) -> GuiRos2MotorCommandPayload:
    """Normalize one GUI-facing compact motor command payload."""
    if isinstance(payload, GuiRos2MotorCommandPayload):
        resolved_topic = command_topic or payload.command_topic or _command_topic_for_namespace(
            payload.namespace
        )
        return GuiRos2MotorCommandPayload(
            namespace=payload.namespace,
            motor_id=int(payload.motor_id),
            cmd_type=int(payload.cmd_type),
            param1=int(payload.param1),
            param2=int(payload.param2),
            param3=int(payload.param3),
            command_topic=resolved_topic,
            diagnostics=dict(payload.diagnostics),
        )
    source = _unwrap_payload_source(payload)
    data = _coerce_int_sequence(_read_value(source, "data", default=[]))
    resolved_topic = _optional_str(_read_value(source, "command_topic", default=command_topic))
    resolved_namespace = _resolve_gui_namespace(
        namespace=namespace,
        topic=resolved_topic,
        explicit=_optional_str(_read_value(source, "namespace", default=None)),
    )
    motor_id = int(
        _read_value(
            source,
            "motor_id",
            default=(0 if not data else data[0]),
        )
    )
    cmd_type = int(
        _read_value(
            source,
            "cmd_type",
            default=(0 if len(data) < 2 else data[1]),
        )
    )
    param1 = int(_read_value(source, "param1", default=(0 if len(data) < 3 else data[2])))
    param2 = int(_read_value(source, "param2", default=(0 if len(data) < 4 else data[3])))
    param3 = int(_read_value(source, "param3", default=(0 if len(data) < 5 else data[4])))
    return GuiRos2MotorCommandPayload(
        namespace=resolved_namespace,
        motor_id=motor_id,
        cmd_type=cmd_type,
        param1=param1,
        param2=param2,
        param3=param3,
        command_topic=resolved_topic or _command_topic_for_namespace(resolved_namespace),
        diagnostics=_coerce_dict(_read_value(source, "diagnostics", default={})),
    )


def normalize_gui_ros2_feedback_payload(
    payload: GuiRos2MotorFeedbackPayload | dict[str, object] | object,
    *,
    namespace: str | None = None,
    feedback_topic: str | None = None,
) -> GuiRos2MotorFeedbackPayload:
    """Normalize one GUI-facing feedback payload."""
    if isinstance(payload, GuiRos2MotorFeedbackPayload):
        resolved_topic = feedback_topic or payload.feedback_topic or _feedback_topic_for_namespace(
            payload.namespace
        )
        return GuiRos2MotorFeedbackPayload(
            namespace=payload.namespace,
            motor_id=int(payload.motor_id),
            raw_pulses=int(payload.raw_pulses),
            device_time_ms=payload.device_time_ms,
            seq=payload.seq,
            status_flags=payload.status_flags,
            source_id=payload.source_id,
            is_extended=bool(payload.is_extended),
            feedback_topic=resolved_topic,
            diagnostics=dict(payload.diagnostics),
        )
    source = _unwrap_payload_source(payload)
    data = _coerce_int_sequence(_read_value(source, "data", default=[]))
    resolved_topic = _optional_str(_read_value(source, "feedback_topic", default=feedback_topic))
    resolved_namespace = _resolve_gui_namespace(
        namespace=namespace,
        topic=resolved_topic,
        explicit=_optional_str(_read_value(source, "namespace", default=None)),
    )
    motor_id = int(_read_value(source, "motor_id", default=(0 if not data else data[0])))
    raw_pulses = int(_read_value(source, "raw_pulses", default=(0 if len(data) < 2 else data[1])))
    is_extended = bool(_read_value(source, "is_extended", default=len(data) >= 6))
    return GuiRos2MotorFeedbackPayload(
        namespace=resolved_namespace,
        motor_id=motor_id,
        raw_pulses=raw_pulses,
        device_time_ms=_optional_int(_read_value(source, "device_time_ms", default=(None if len(data) < 3 else data[2]))),
        seq=_optional_int(_read_value(source, "seq", default=(None if len(data) < 4 else data[3]))),
        status_flags=_optional_int(
            _read_value(source, "status_flags", default=(None if len(data) < 5 else data[4]))
        ),
        source_id=_optional_int(_read_value(source, "source_id", default=(None if len(data) < 6 else data[5]))),
        is_extended=is_extended,
        feedback_topic=resolved_topic or _feedback_topic_for_namespace(resolved_namespace),
        diagnostics=_coerce_dict(_read_value(source, "diagnostics", default={})),
    )


def gui_ros2_command_payload_to_array(
    payload: GuiRos2MotorCommandPayload | dict[str, object] | object,
) -> list[int]:
    """Convert one normalized GUI command payload into its compact array form."""
    normalized = normalize_gui_ros2_command_payload(payload)
    return list(normalized.data)


def gui_ros2_feedback_payload_to_array(
    payload: GuiRos2MotorFeedbackPayload | dict[str, object] | object,
) -> list[int]:
    """Convert one normalized GUI feedback payload into its compact array form."""
    normalized = normalize_gui_ros2_feedback_payload(payload)
    return list(normalized.data)


def runtime_frame_from_gui_ros2_feedbacks(
    feedbacks_by_namespace: dict[str, dict[int, GuiRos2MotorFeedbackPayload | dict[str, object] | object]]
    | dict[str, list[GuiRos2MotorFeedbackPayload | dict[str, object] | object]]
    | dict[str, GuiRos2MotorFeedbackPayload | dict[str, object] | object],
    *,
    source_name: str = "gui_ros2_feedback_provider",
    fallback_topology_hint: dict[str, object] | None = None,
) -> RuntimeObservationFrame:
    """Build a runtime frame from the GUI's namespace/topic feedback cache."""
    normalized_feedbacks = _normalize_feedback_cache(feedbacks_by_namespace)
    modules: dict[str, ModuleObservation] = {}
    for namespace in GUI_ROS2_TOPIC_NAMESPACES:
        feedbacks = normalized_feedbacks.get(namespace)
        if not feedbacks:
            continue
        module = _module_observation_from_feedback(namespace, feedbacks, source_name=source_name)
        if module is not None:
            modules[namespace] = module
    pairs = _pair_observations_from_modules(modules, source_name=source_name)
    topology_hint = {
        "ordered_modules": [namespace for namespace in GUI_ROS2_TOPIC_NAMESPACES if namespace in modules],
        "active_frontier": [namespace for namespace in ("joint1", "tip") if namespace in modules],
        "support_modules": [namespace for namespace in ("joint1", "joint2") if namespace in modules],
        **dict(fallback_topology_hint or {}),
    }
    timestamp_ns = _feedback_timestamp_ns(normalized_feedbacks)
    return RuntimeObservationFrame(
        timestamp_ns=timestamp_ns,
        module_observations=modules,
        pair_observations=pairs,
        topology_hint=topology_hint,
        metadata={
            "source_name": source_name,
            "provider_source": "ros2_gui_feedback",
            "frame_origin": "ros2",
            "gui_ros2_compatible": True,
            "topic_namespaces": [namespace for namespace in GUI_ROS2_TOPIC_NAMESPACES if namespace in normalized_feedbacks],
        },
    )


def scheduler_envelope_to_gui_ros2_commands(
    envelope: SchedulerDispatchEnvelope,
    *,
    source_name: str = "ros2_command_dispatcher",
) -> list[GuiRos2MotorCommandPayload]:
    """Map one scheduler envelope onto `gui_ros2.py`-compatible compact motor commands."""
    scheduled = envelope.scheduled_command
    if scheduled is None:
        return []
    commands: list[GuiRos2MotorCommandPayload] = []
    for index, reference in enumerate(scheduled.primitive_references):
        mapped = _primitive_reference_to_gui_command(
            module_id=reference.module_id,
            primitive_name=reference.primitive_name or reference.axis,
            reference_value=float(reference.reference_value),
            source_name=source_name,
            index=index,
        )
        if mapped is not None:
            commands.append(mapped)
    return commands


def build_gui_ros2_emergency_stop_commands(
    namespaces: tuple[str, ...] | list[str] | None = None,
    *,
    source_name: str = "ros2_command_dispatcher",
) -> list[GuiRos2MotorCommandPayload]:
    """Build emergency-stop commands for every configured GUI namespace/motor."""
    resolved_namespaces = GUI_ROS2_TOPIC_NAMESPACES if namespaces is None else tuple(namespaces)
    commands: list[GuiRos2MotorCommandPayload] = []
    for binding in GUI_ROS2_TOPIC_BINDINGS:
        if binding.namespace not in resolved_namespaces:
            continue
        for motor_id in binding.motor_ids:
            commands.append(
                GuiRos2MotorCommandPayload(
                    namespace=binding.namespace,
                    motor_id=int(motor_id),
                    cmd_type=0x01,
                    command_topic=binding.command_topic,
                    diagnostics={
                        "source_name": source_name,
                        "semantic": "emergency_stop",
                    },
                )
            )
    return commands


def normalize_ros2_observation_payload(
    payload: Ros2ObservationPayload | dict[str, object] | object,
) -> Ros2ObservationPayload:
    """Normalize a ROS2 observation payload from a dataclass, dict, JSON string, or message."""
    if isinstance(payload, Ros2ObservationPayload):
        return payload
    source = _unwrap_payload_source(payload)
    modules_raw = _read_value(source, "modules", default=[])
    pairs_raw = _read_value(source, "pairs", default=[])
    diagnostics = _coerce_dict(_read_value(source, "diagnostics", default={}))
    topology_hint = _coerce_dict(_read_value(source, "topology_hint", default={}))
    modules = [normalize_ros2_module_payload(module) for module in _coerce_sequence(modules_raw)]
    pairs = [normalize_ros2_pair_payload(pair) for pair in _coerce_sequence(pairs_raw)]
    return Ros2ObservationPayload(
        timestamp_ns=int(_read_value(source, "timestamp_ns", default=0)),
        modules=modules,
        pairs=pairs,
        topology_hint=topology_hint,
        diagnostics=diagnostics,
    )


def normalize_ros2_command_payload(
    payload: Ros2CommandPayload | dict[str, object] | object,
) -> Ros2CommandPayload:
    """Normalize a ROS2 command payload from a dataclass, dict, JSON string, or message."""
    if isinstance(payload, Ros2CommandPayload):
        return payload
    source = _unwrap_payload_source(payload)
    commands_raw = _read_value(source, "commands", default=[])
    commands = [
        normalize_ros2_primitive_command_payload(command)
        for command in _coerce_sequence(commands_raw)
    ]
    return Ros2CommandPayload(
        graph_id=_optional_str(_read_value(source, "graph_id", default=None)),
        node_id=_optional_str(_read_value(source, "node_id", default=None)),
        skill_key=_optional_str(_read_value(source, "skill_key", default=None)),
        dispatch_target=_optional_str(_read_value(source, "dispatch_target", default="ros2")) or "ros2",
        stop_requested=bool(_read_value(source, "stop_requested", default=False)),
        noop_requested=bool(_read_value(source, "noop_requested", default=False)),
        commands=commands,
        topology_snapshot=_coerce_dict(_read_value(source, "topology_snapshot", default={})),
        frontier_snapshot=_coerce_dict(_read_value(source, "frontier_snapshot", default={})),
        support_snapshot=_coerce_dict(_read_value(source, "support_snapshot", default={})),
        diagnostics=_coerce_dict(_read_value(source, "diagnostics", default={})),
    )


def normalize_ros2_module_payload(
    payload: Ros2ModuleStatePayload | dict[str, object] | object,
) -> Ros2ModuleStatePayload:
    """Normalize one ROS2 module payload."""
    if isinstance(payload, Ros2ModuleStatePayload):
        return payload
    source = _unwrap_payload_source(payload)
    return Ros2ModuleStatePayload(
        module_id=str(_read_value(source, "module_id", default="unknown")),
        module_type=_optional_str(_read_value(source, "module_type", default=None)),
        dofs=_coerce_float_map(_read_value(source, "dofs", default={})),
        velocities=_coerce_float_map(_read_value(source, "velocities", default={})),
        attach_state=_coerce_attach_state(_read_value(source, "attach_state", default={})),
        diagnostics=_coerce_dict(_read_value(source, "diagnostics", default={})),
    )


def normalize_ros2_pair_payload(
    payload: Ros2PairStatePayload | dict[str, object] | object,
) -> Ros2PairStatePayload:
    """Normalize one ROS2 pair payload."""
    if isinstance(payload, Ros2PairStatePayload):
        return payload
    source = _unwrap_payload_source(payload)
    return Ros2PairStatePayload(
        active_module=str(_read_value(source, "active_module", default="unknown")),
        passive_module=str(_read_value(source, "passive_module", default="unknown")),
        relation_type=_optional_str(_read_value(source, "relation_type", default=None)),
        distance_mm=_optional_float(_read_value(source, "distance_mm", default=None)),
        orientation_error_deg=_optional_float(
            _read_value(source, "orientation_error_deg", default=None)
        ),
        coupled=_optional_bool(_read_value(source, "coupled", default=None)),
        observation_valid=bool(_read_value(source, "observation_valid", default=False)),
        diagnostics=_coerce_dict(_read_value(source, "diagnostics", default={})),
    )


def normalize_ros2_primitive_command_payload(
    payload: Ros2PrimitiveCommandPayload | dict[str, object] | object,
) -> Ros2PrimitiveCommandPayload:
    """Normalize one ROS2 primitive command payload."""
    if isinstance(payload, Ros2PrimitiveCommandPayload):
        return payload
    source = _unwrap_payload_source(payload)
    return Ros2PrimitiveCommandPayload(
        module_id=str(_read_value(source, "module_id", default="unknown")),
        primitive_name=str(_read_value(source, "primitive_name", default="noop")),
        axis=str(_read_value(source, "axis", default="noop")),
        reference_kind=str(_read_value(source, "reference_kind", default="velocity")),
        reference_value=float(_read_value(source, "reference_value", default=0.0)),
        units=str(_read_value(source, "units", default="")),
        semantic=str(_read_value(source, "semantic", default="")),
        primary=bool(_read_value(source, "primary", default=False)),
        target_value=_optional_float(_read_value(source, "target_value", default=None)),
        diagnostics=_coerce_dict(_read_value(source, "diagnostics", default={})),
    )


def ros2_observation_payload_to_mapping(
    payload: Ros2ObservationPayload | dict[str, object] | object,
) -> dict[str, object]:
    """Convert an observation payload into a plain dictionary for JSON transport."""
    normalized = normalize_ros2_observation_payload(payload)
    return {
        "timestamp_ns": int(normalized.timestamp_ns),
        "modules": [asdict(module) for module in normalized.modules],
        "pairs": [asdict(pair) for pair in normalized.pairs],
        "topology_hint": dict(normalized.topology_hint),
        "diagnostics": dict(normalized.diagnostics),
    }


def ros2_command_payload_to_mapping(
    payload: Ros2CommandPayload | dict[str, object] | object,
) -> dict[str, object]:
    """Convert a command payload into a plain dictionary for JSON transport."""
    normalized = normalize_ros2_command_payload(payload)
    return {
        "graph_id": normalized.graph_id,
        "node_id": normalized.node_id,
        "skill_key": normalized.skill_key,
        "dispatch_target": normalized.dispatch_target,
        "stop_requested": normalized.stop_requested,
        "noop_requested": normalized.noop_requested,
        "commands": [asdict(command) for command in normalized.commands],
        "topology_snapshot": dict(normalized.topology_snapshot),
        "frontier_snapshot": dict(normalized.frontier_snapshot),
        "support_snapshot": dict(normalized.support_snapshot),
        "diagnostics": dict(normalized.diagnostics),
    }


def ros2_payload_to_json(payload: Ros2ObservationPayload | Ros2CommandPayload | dict[str, object] | object) -> str:
    """Serialize a supported ROS2 payload into a stable JSON string."""
    source = _unwrap_payload_source(payload)
    if isinstance(source, dict):
        return json.dumps(source, sort_keys=True)
    if isinstance(payload, Ros2ObservationPayload):
        return json.dumps(ros2_observation_payload_to_mapping(payload), sort_keys=True)
    return json.dumps(ros2_command_payload_to_mapping(payload), sort_keys=True)


def _unwrap_payload_source(source: object) -> object:
    """Normalize dataclasses and `std_msgs/String`-style wrappers into plain payloads."""
    if is_dataclass(source):
        return asdict(source)
    if isinstance(source, dict):
        return source
    data_value = getattr(source, "data", None)
    if isinstance(data_value, bytes):
        try:
            data_value = data_value.decode("utf-8")
        except Exception:
            data_value = None
    if isinstance(data_value, str):
        try:
            parsed = json.loads(data_value)
        except json.JSONDecodeError:
            return {"data": data_value}
        if isinstance(parsed, dict):
            return parsed
    return source


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


def _coerce_int_sequence(value: object) -> list[int]:
    """Normalize a sequence-like value into integers."""
    if isinstance(value, (str, bytes)) or value is None:
        return []
    if isinstance(value, (list, tuple)):
        raw_items = value
    else:
        try:
            raw_items = list(value)  # type: ignore[arg-type]
        except TypeError:
            return []
    normalized: list[int] = []
    for item in raw_items:
        try:
            normalized.append(int(item))
        except (TypeError, ValueError):
            normalized.append(0)
    return normalized


def _optional_float(value: object) -> float | None:
    """Convert a scalar-like value into float while preserving missing data."""
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _optional_int(value: object) -> int | None:
    """Convert a scalar-like value into int while preserving missing data."""
    if value is None:
        return None
    try:
        return int(value)
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


def _resolve_gui_namespace(
    *,
    namespace: str | None = None,
    topic: str | None = None,
    explicit: str | None = None,
) -> str:
    """Resolve the GUI namespace from explicit metadata or a topic path."""
    if explicit:
        return explicit.strip().strip("/")
    if namespace:
        return namespace.strip().strip("/")
    if topic:
        stripped = topic.strip().strip("/")
        if "/" in stripped:
            return stripped.split("/", 1)[0]
    return "unknown"


def _command_topic_for_namespace(namespace: str) -> str:
    """Return the compact command topic for one GUI namespace."""
    return f"{namespace}/motor_command"


def _feedback_topic_for_namespace(namespace: str) -> str:
    """Return the compact feedback topic for one GUI namespace."""
    return f"{namespace}/motor_feedback"


def _normalize_feedback_cache(
    feedbacks_by_namespace: dict[str, object],
) -> dict[str, dict[int, GuiRos2MotorFeedbackPayload]]:
    """Normalize a mixed feedback cache into namespace -> motor -> payload."""
    normalized: dict[str, dict[int, GuiRos2MotorFeedbackPayload]] = {}
    for namespace, value in dict(feedbacks_by_namespace or {}).items():
        namespace_key = str(namespace)
        namespace_cache: dict[int, GuiRos2MotorFeedbackPayload] = {}
        if isinstance(value, dict):
            items = value.values()
        elif isinstance(value, list):
            items = value
        else:
            items = [value]
        for item in items:
            payload = normalize_gui_ros2_feedback_payload(item, namespace=namespace_key)
            namespace_cache[int(payload.motor_id)] = payload
        if namespace_cache:
            normalized[namespace_key] = namespace_cache
    return normalized


def _module_observation_from_feedback(
    namespace: str,
    feedbacks: dict[int, GuiRos2MotorFeedbackPayload],
    *,
    source_name: str,
) -> ModuleObservation | None:
    """Project one GUI namespace feedback cache into a runtime module observation."""
    if namespace == "tip":
        tip_feedback = feedbacks.get(4)
        if tip_feedback is None:
            return None
        growth_mm, _ = counts_to_physical(4, tip_feedback.raw_pulses)
        return ModuleObservation(
            module_id="tip",
            module_type="tip",
            dofs={"growth_mm": float(growth_mm)},
            velocities={},
            attach_state={"joint1": None},
            diagnostics={
                "feedback_motor_ids": [4],
                "topic_namespace": namespace,
                "gui_ros2_compatible": True,
            },
            source_name=source_name,
        )
    if not namespace.startswith("joint"):
        return None
    dofs: dict[str, float] = {}
    motor1 = feedbacks.get(1)
    motor2 = feedbacks.get(2)
    motor3 = feedbacks.get(3)
    if motor1 is not None:
        dofs["crawl_mm"] = float(counts_to_physical(1, motor1.raw_pulses)[0])
    if motor2 is not None:
        dofs["rotate_deg"] = float(counts_to_physical(2, motor2.raw_pulses)[0])
    if motor3 is not None:
        dofs["bend_deg"] = float(counts_to_physical(3, motor3.raw_pulses)[0])
    return ModuleObservation(
        module_id=namespace,
        module_type="joint",
        dofs=dofs,
        velocities={},
        attach_state={
            "tip": None if namespace != "joint1" else None,
            "parent_joint": None,
        },
        diagnostics={
            "feedback_motor_ids": sorted(feedbacks),
            "topic_namespace": namespace,
            "gui_ros2_compatible": True,
        },
        source_name=source_name,
    )


def _pair_observations_from_modules(
    modules: dict[str, ModuleObservation],
    *,
    source_name: str,
) -> dict[str, PairObservation]:
    """Build minimal pair observations from GUI-module feedbacks."""
    pairs: dict[str, PairObservation] = {}
    ordered_pairs = [
        ("joint1", "tip", "tip_joint"),
        ("joint2", "joint1", "joint_joint"),
        ("joint3", "joint2", "joint_joint"),
        ("joint4", "joint3", "joint_joint"),
        ("joint5", "joint4", "joint_joint"),
    ]
    for active_module, passive_module, relation_type in ordered_pairs:
        active = modules.get(active_module)
        passive = modules.get(passive_module)
        if active is None or passive is None:
            continue
        active_linear = _module_linear_position(active)
        passive_linear = _module_linear_position(passive)
        active_orientation = _module_orientation(active)
        passive_orientation = _module_orientation(passive)
        distance_mm = (
            None
            if active_linear is None or passive_linear is None
            else abs(float(active_linear) - float(passive_linear))
        )
        if active_orientation is None:
            orientation_error_deg = None
        elif passive_orientation is None:
            orientation_error_deg = float(active_orientation)
        else:
            orientation_error_deg = float(active_orientation) - float(passive_orientation)
        coupled = None
        if distance_mm is not None and orientation_error_deg is not None:
            coupled = bool(distance_mm <= 1.0 and abs(float(orientation_error_deg)) <= 2.0)
        pair_key = f"{active_module}->{passive_module}"
        pairs[pair_key] = PairObservation(
            active_module=active_module,
            passive_module=passive_module,
            relation_type=relation_type,
            distance_mm=distance_mm,
            orientation_error_deg=orientation_error_deg,
            coupled=coupled,
            observation_valid=distance_mm is not None,
            diagnostics={
                "provider_source": "ros2_gui_feedback",
                "gui_ros2_compatible": True,
            },
            source_name=source_name,
        )
    return pairs


def _module_linear_position(module: ModuleObservation) -> float | None:
    """Return the scalar linear position used for simple pair-distance heuristics."""
    if "growth_mm" in module.dofs:
        return float(module.dofs["growth_mm"])
    if "crawl_mm" in module.dofs:
        return float(module.dofs["crawl_mm"])
    return None


def _module_orientation(module: ModuleObservation) -> float | None:
    """Return the scalar orientation used for simple pair-orientation heuristics."""
    if "rotate_deg" in module.dofs:
        return float(module.dofs["rotate_deg"])
    if "bend_deg" in module.dofs:
        return float(module.dofs["bend_deg"])
    return None


def _feedback_timestamp_ns(
    feedbacks_by_namespace: dict[str, dict[int, GuiRos2MotorFeedbackPayload]],
) -> int:
    """Resolve a stable frame timestamp from GUI feedback metadata."""
    device_times: list[int] = []
    seqs: list[int] = []
    for namespace_feedbacks in feedbacks_by_namespace.values():
        for payload in namespace_feedbacks.values():
            if payload.device_time_ms is not None:
                device_times.append(int(payload.device_time_ms) * 1_000_000)
            if payload.seq is not None:
                seqs.append(int(payload.seq))
    if device_times:
        return max(device_times)
    if seqs:
        return max(seqs)
    return 0


def _primitive_reference_to_gui_command(
    *,
    module_id: str,
    primitive_name: str,
    reference_value: float,
    source_name: str,
    index: int,
) -> GuiRos2MotorCommandPayload | None:
    """Map one runtime primitive onto the GUI compact command format."""
    namespace = str(module_id)
    primitive = str(primitive_name or "")
    motor_id: int | None = None
    cmd_type = 0x02
    param1 = 0
    if primitive == "tip_growth" and namespace == "tip":
        motor_id = 4
        param1 = int(round(physical_to_counts(4, reference_value)))
    elif primitive == "joint_crawl":
        motor_id = 1
        param1 = int(round(physical_to_counts(1, reference_value)))
    elif primitive == "joint_rotate":
        motor_id = 2
        param1 = int(round(physical_to_counts(2, reference_value)))
    elif primitive == "joint_bend":
        motor_id = 3
        param1 = int(round(physical_to_counts(3, reference_value)))
    elif primitive == "joint_rotate_hold":
        motor_id = 2
        cmd_type = 0x01
    elif primitive == "joint_bend_hold":
        motor_id = 3
        cmd_type = 0x01
    if motor_id is None:
        return None
    return GuiRos2MotorCommandPayload(
        namespace=namespace,
        motor_id=motor_id,
        cmd_type=cmd_type,
        param1=param1,
        param2=0,
        param3=0,
        command_topic=_command_topic_for_namespace(namespace),
        diagnostics={
            "source_name": source_name,
            "primitive_name": primitive,
            "command_index": index,
            "gui_ros2_compatible": True,
        },
    )


__all__ = [
    "build_gui_ros2_emergency_stop_commands",
    "envelope_to_ros2_command_payload",
    "gui_ros2_command_payload_to_array",
    "gui_ros2_feedback_payload_to_array",
    "normalize_ros2_command_payload",
    "normalize_gui_ros2_command_payload",
    "normalize_gui_ros2_feedback_payload",
    "normalize_ros2_module_payload",
    "normalize_ros2_observation_payload",
    "normalize_ros2_pair_payload",
    "normalize_ros2_primitive_command_payload",
    "ros2_command_payload_to_mapping",
    "ros2_observation_payload_to_mapping",
    "ros2_payload_to_json",
    "runtime_frame_from_gui_ros2_feedbacks",
    "runtime_frame_from_ros2_message",
    "scheduler_envelope_to_gui_ros2_commands",
]

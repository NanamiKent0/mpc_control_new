"""Internal ROS2 runtime payload contracts owned by the new architecture."""

from __future__ import annotations

from dataclasses import dataclass, field


ROS2_RUNTIME_OBSERVATION_TOPIC = "/mpc_control_new/runtime/observation"
ROS2_RUNTIME_COMMAND_TOPIC = "/mpc_control_new/runtime/command"


@dataclass(slots=True)
class Ros2ModuleStatePayload:
    """ROS2-side module observation payload used by the runtime provider."""

    module_id: str
    module_type: str | None = None
    dofs: dict[str, float] = field(default_factory=dict)
    velocities: dict[str, float] = field(default_factory=dict)
    attach_state: dict[str, bool | None] = field(default_factory=dict)
    diagnostics: dict[str, object] = field(default_factory=dict)


@dataclass(slots=True)
class Ros2PairStatePayload:
    """ROS2-side pair observation payload used by the runtime provider."""

    active_module: str
    passive_module: str
    relation_type: str | None = None
    distance_mm: float | None = None
    orientation_error_deg: float | None = None
    coupled: bool | None = None
    observation_valid: bool = False
    diagnostics: dict[str, object] = field(default_factory=dict)


@dataclass(slots=True)
class Ros2ObservationPayload:
    """Whole-frame ROS2 observation payload consumed by the provider adapter."""

    timestamp_ns: int
    modules: list[Ros2ModuleStatePayload] = field(default_factory=list)
    pairs: list[Ros2PairStatePayload] = field(default_factory=list)
    topology_hint: dict[str, object] = field(default_factory=dict)
    diagnostics: dict[str, object] = field(default_factory=dict)


@dataclass(slots=True)
class Ros2PrimitiveCommandPayload:
    """One primitive command message emitted toward the ROS2 live runtime."""

    module_id: str
    primitive_name: str
    axis: str
    reference_kind: str
    reference_value: float
    units: str
    semantic: str
    primary: bool = False
    target_value: float | None = None
    diagnostics: dict[str, object] = field(default_factory=dict)


@dataclass(slots=True)
class Ros2CommandPayload:
    """ROS2-side command payload derived from a scheduler dispatch envelope."""

    graph_id: str | None
    node_id: str | None
    skill_key: str | None
    dispatch_target: str = "ros2"
    stop_requested: bool = False
    noop_requested: bool = False
    commands: list[Ros2PrimitiveCommandPayload] = field(default_factory=list)
    topology_snapshot: dict[str, object] = field(default_factory=dict)
    frontier_snapshot: dict[str, object] = field(default_factory=dict)
    support_snapshot: dict[str, object] = field(default_factory=dict)
    diagnostics: dict[str, object] = field(default_factory=dict)


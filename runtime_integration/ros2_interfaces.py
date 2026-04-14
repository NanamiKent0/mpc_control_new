"""Internal ROS2 runtime payload contracts owned by the new architecture."""

from __future__ import annotations

from dataclasses import dataclass, field

from .common.encoder_protocol import NAMESPACE_TO_MOTOR_IDS
from .ros2_topic_config import build_default_ros2_topic_config


DEFAULT_ROS2_TOPIC_CONFIG = build_default_ros2_topic_config()
ROS2_RUNTIME_OBSERVATION_TOPIC = DEFAULT_ROS2_TOPIC_CONFIG.observation.frame_topic
ROS2_RUNTIME_COMMAND_TOPIC = DEFAULT_ROS2_TOPIC_CONFIG.command.command_topic
GUI_ROS2_TOPIC_NAMESPACES: tuple[str, ...] = ("tip", "joint1", "joint2", "joint3", "joint4", "joint5")
GUI_ROS2_COMMAND_TOPIC_SUFFIX = "motor_command"
GUI_ROS2_FEEDBACK_TOPIC_SUFFIX = "motor_feedback"


@dataclass(slots=True, frozen=True)
class Ros2ObservationTopicSemantic:
    """Document the runtime observation topic semantics for the ROS2 boundary."""

    frame_topic: str
    module_observations_field: str = "modules"
    pair_observations_field: str = "pairs"
    timestamp_field: str = "timestamp_ns"
    topology_hint_field: str = "topology_hint"
    diagnostics_field: str = "diagnostics"


@dataclass(slots=True, frozen=True)
class Ros2CommandTopicSemantic:
    """Document the runtime command topic semantics for the ROS2 boundary."""

    command_topic: str
    graph_id_field: str = "graph_id"
    node_id_field: str = "node_id"
    skill_key_field: str = "skill_key"
    primitive_commands_field: str = "commands"
    stop_flag_field: str = "stop_requested"
    noop_flag_field: str = "noop_requested"
    diagnostics_field: str = "diagnostics"


ROS2_OBSERVATION_TOPIC_SEMANTICS = Ros2ObservationTopicSemantic(
    frame_topic=ROS2_RUNTIME_OBSERVATION_TOPIC
)
ROS2_COMMAND_TOPIC_SEMANTICS = Ros2CommandTopicSemantic(
    command_topic=ROS2_RUNTIME_COMMAND_TOPIC
)


@dataclass(slots=True, frozen=True)
class GuiRos2TopicBinding:
    """Describe one GUI-facing namespace topic pair."""

    namespace: str
    command_topic: str
    feedback_topic: str
    motor_ids: tuple[int, ...]


@dataclass(slots=True)
class GuiRos2MotorCommandPayload:
    """One `gui_ros2.py`-compatible compact command payload."""

    namespace: str
    motor_id: int
    cmd_type: int
    param1: int = 0
    param2: int = 0
    param3: int = 0
    command_topic: str = ""
    diagnostics: dict[str, object] = field(default_factory=dict)

    @property
    def data(self) -> list[int]:
        """Return the compact array layout used by the GUI and firmware."""
        return [
            int(self.motor_id),
            int(self.cmd_type),
            int(self.param1),
            int(self.param2),
            int(self.param3),
        ]


@dataclass(slots=True)
class GuiRos2MotorFeedbackPayload:
    """One `gui_ros2.py`-compatible feedback payload."""

    namespace: str
    motor_id: int
    raw_pulses: int
    device_time_ms: int | None = None
    seq: int | None = None
    status_flags: int | None = None
    source_id: int | None = None
    is_extended: bool = False
    feedback_topic: str = ""
    diagnostics: dict[str, object] = field(default_factory=dict)

    @property
    def data(self) -> list[int]:
        """Return the compact or extended feedback array consumed by the GUI."""
        payload = [int(self.motor_id), int(self.raw_pulses)]
        if self.is_extended:
            payload.extend(
                [
                    0 if self.device_time_ms is None else int(self.device_time_ms),
                    0 if self.seq is None else int(self.seq),
                    0 if self.status_flags is None else int(self.status_flags),
                    0 if self.source_id is None else int(self.source_id),
                ]
            )
        return payload


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


def build_gui_ros2_topic_bindings(
    namespaces: tuple[str, ...] | list[str] | None = None,
) -> tuple[GuiRos2TopicBinding, ...]:
    """Build the GUI-facing namespace topic map used by sim/live/runtime adapters."""
    resolved_namespaces = GUI_ROS2_TOPIC_NAMESPACES if namespaces is None else tuple(namespaces)
    return tuple(
        GuiRos2TopicBinding(
            namespace=namespace,
            command_topic=f"{namespace}/{GUI_ROS2_COMMAND_TOPIC_SUFFIX}",
            feedback_topic=f"{namespace}/{GUI_ROS2_FEEDBACK_TOPIC_SUFFIX}",
            motor_ids=tuple(NAMESPACE_TO_MOTOR_IDS.get(namespace, ())),
        )
        for namespace in resolved_namespaces
    )


GUI_ROS2_TOPIC_BINDINGS = build_gui_ros2_topic_bindings()
GUI_ROS2_COMMAND_TOPICS = tuple(binding.command_topic for binding in GUI_ROS2_TOPIC_BINDINGS)
GUI_ROS2_FEEDBACK_TOPICS = tuple(binding.feedback_topic for binding in GUI_ROS2_TOPIC_BINDINGS)


__all__ = [
    "DEFAULT_ROS2_TOPIC_CONFIG",
    "GUI_ROS2_COMMAND_TOPICS",
    "GUI_ROS2_COMMAND_TOPIC_SUFFIX",
    "GUI_ROS2_FEEDBACK_TOPICS",
    "GUI_ROS2_FEEDBACK_TOPIC_SUFFIX",
    "GUI_ROS2_TOPIC_BINDINGS",
    "GUI_ROS2_TOPIC_NAMESPACES",
    "GuiRos2MotorCommandPayload",
    "GuiRos2MotorFeedbackPayload",
    "GuiRos2TopicBinding",
    "ROS2_COMMAND_TOPIC_SEMANTICS",
    "ROS2_OBSERVATION_TOPIC_SEMANTICS",
    "ROS2_RUNTIME_COMMAND_TOPIC",
    "ROS2_RUNTIME_OBSERVATION_TOPIC",
    "Ros2CommandPayload",
    "Ros2CommandTopicSemantic",
    "Ros2ModuleStatePayload",
    "Ros2ObservationPayload",
    "Ros2ObservationTopicSemantic",
    "Ros2PairStatePayload",
    "Ros2PrimitiveCommandPayload",
    "build_gui_ros2_topic_bindings",
]

"""Centralized ROS2 topic configuration for the new runtime path."""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(slots=True)
class Ros2ObservationTopicConfig:
    """Topic names and QoS settings for runtime observations."""

    frame_topic: str = "/mpc_control_new/runtime/observation"
    modules_topic: str = "/mpc_control_new/runtime/observation/modules"
    pairs_topic: str = "/mpc_control_new/runtime/observation/pairs"
    diagnostics_topic: str = "/mpc_control_new/runtime/observation/diagnostics"
    qos_depth: int = 10

    def source_topics(self) -> list[str]:
        """Return the observation topics visible to the provider."""
        return [self.frame_topic, self.modules_topic, self.pairs_topic]


@dataclass(slots=True)
class Ros2CommandTopicConfig:
    """Topic names and QoS settings for runtime commands."""

    command_topic: str = "/mpc_control_new/runtime/command"
    diagnostics_topic: str = "/mpc_control_new/runtime/command/diagnostics"
    stop_topic: str = "/mpc_control_new/runtime/command/stop"
    qos_depth: int = 10

    def target_topics(self) -> list[str]:
        """Return the command topics targeted by the dispatcher."""
        return [self.command_topic, self.diagnostics_topic, self.stop_topic]


@dataclass(slots=True)
class Ros2TopicConfig:
    """Complete ROS2 topic configuration used by provider, dispatcher, and node wiring."""

    observation: Ros2ObservationTopicConfig = field(default_factory=Ros2ObservationTopicConfig)
    command: Ros2CommandTopicConfig = field(default_factory=Ros2CommandTopicConfig)

    def source_topics(self) -> list[str]:
        """Return all configured observation topics."""
        return self.observation.source_topics()

    def target_topics(self) -> list[str]:
        """Return all configured command topics."""
        return self.command.target_topics()

    def to_dict(self) -> dict[str, object]:
        """Convert the topic configuration into a JSON-friendly dictionary."""
        return {
            "observation": {
                "frame_topic": self.observation.frame_topic,
                "modules_topic": self.observation.modules_topic,
                "pairs_topic": self.observation.pairs_topic,
                "diagnostics_topic": self.observation.diagnostics_topic,
                "qos_depth": self.observation.qos_depth,
            },
            "command": {
                "command_topic": self.command.command_topic,
                "diagnostics_topic": self.command.diagnostics_topic,
                "stop_topic": self.command.stop_topic,
                "qos_depth": self.command.qos_depth,
            },
        }


def build_default_ros2_topic_config(
    overrides: dict[str, object] | None = None,
) -> Ros2TopicConfig:
    """Build the default topic configuration with optional nested overrides."""
    config = Ros2TopicConfig()
    payload = dict(overrides or {})
    observation_overrides = payload.get("observation")
    if isinstance(observation_overrides, dict):
        for key, value in observation_overrides.items():
            if hasattr(config.observation, key):
                setattr(config.observation, key, value)
    command_overrides = payload.get("command")
    if isinstance(command_overrides, dict):
        for key, value in command_overrides.items():
            if hasattr(config.command, key):
                setattr(config.command, key, value)
    return config


__all__ = [
    "Ros2CommandTopicConfig",
    "Ros2ObservationTopicConfig",
    "Ros2TopicConfig",
    "build_default_ros2_topic_config",
]

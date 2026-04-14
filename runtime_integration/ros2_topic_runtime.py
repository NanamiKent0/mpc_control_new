"""Helpers for wiring ROS2 topic config, node, provider, and dispatcher together."""

from __future__ import annotations

from dataclasses import dataclass

from .live_command_dispatcher import LiveRuntimeCommandDispatcher
from .live_runtime_provider import LiveRuntimeObservationProvider
from .ros2_runtime_node import Ros2RuntimeNode
from .ros2_topic_config import Ros2TopicConfig, build_default_ros2_topic_config


@dataclass(slots=True)
class Ros2RuntimeComponents:
    """Bundled ROS2 runtime objects used by the new architecture main path."""

    topic_config: Ros2TopicConfig
    runtime_node: Ros2RuntimeNode
    observation_provider: LiveRuntimeObservationProvider
    command_dispatcher: LiveRuntimeCommandDispatcher


def build_ros2_runtime_components(
    *,
    topic_config: Ros2TopicConfig | None = None,
    runtime_node: Ros2RuntimeNode | None = None,
    enable_rclpy: bool = False,
    node_name: str = "mpc_control_new_runtime",
) -> Ros2RuntimeComponents:
    """Build a provider/dispatcher pair that shares one ROS2 runtime node."""
    resolved_topic_config = topic_config or build_default_ros2_topic_config()
    shared_node = runtime_node or Ros2RuntimeNode(
        node_name=node_name,
        topic_config=resolved_topic_config,
        enable_rclpy=enable_rclpy,
    )
    return Ros2RuntimeComponents(
        topic_config=resolved_topic_config,
        runtime_node=shared_node,
        observation_provider=LiveRuntimeObservationProvider(
            runtime_node=shared_node,
            topic_config=resolved_topic_config,
        ),
        command_dispatcher=LiveRuntimeCommandDispatcher(
            runtime_node=shared_node,
            topic_config=resolved_topic_config,
        ),
    )


__all__ = ["Ros2RuntimeComponents", "build_ros2_runtime_components"]

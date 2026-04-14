"""ROS2-backed live runtime dispatcher with GUI-topic compatibility."""

from __future__ import annotations

from ..control_core.models.runtime_bridge_types import DispatchResult, SchedulerDispatchEnvelope
from .ros2_interfaces import GUI_ROS2_COMMAND_TOPICS, GUI_ROS2_TOPIC_NAMESPACES
from .ros2_message_adapters import (
    build_gui_ros2_emergency_stop_commands,
    scheduler_envelope_to_gui_ros2_commands,
)
from .ros2_runtime_node import Ros2RuntimeNode
from .ros2_topic_config import Ros2TopicConfig, build_default_ros2_topic_config


class LiveRuntimeCommandDispatcher:
    """Dispatch scheduler envelopes onto GUI-compatible ROS2 command topics."""

    dispatcher_kind = "ros2"

    def __init__(
        self,
        *,
        source_name: str = "live_command_dispatcher",
        adapter_description: str = "ros2 live runtime command dispatcher",
        runtime_node: Ros2RuntimeNode | None = None,
        topic_config: Ros2TopicConfig | None = None,
    ) -> None:
        self.source_name = source_name
        self.adapter_description = adapter_description
        self.topic_config = topic_config or build_default_ros2_topic_config()
        self.runtime_node = runtime_node or Ros2RuntimeNode(topic_config=self.topic_config)
        self._started = False
        self._publisher_created = False
        self._published_primitive_count = 0
        self.last_reason = "live_dispatcher_not_started"

    def start(self) -> None:
        """Start the ROS2 dispatcher and create GUI command publishers when possible."""
        self.runtime_node.start()
        node_diagnostics = self.runtime_node.diagnostics()
        self._started = True
        if not bool(node_diagnostics.get("ros2_available")):
            self.last_reason = "ros2_unavailable"
            return
        self._publisher_created = self.runtime_node.create_motor_command_publishers(
            namespaces=GUI_ROS2_TOPIC_NAMESPACES
        )
        self.last_reason = (
            "ros2_gui_dispatcher_started"
            if self._publisher_created
            else "ros2_publisher_unavailable"
        )

    def stop(self) -> None:
        """Stop the ROS2 dispatcher."""
        self.runtime_node.stop()
        self._started = False
        self._publisher_created = False
        self.last_reason = "live_dispatcher_stopped"

    def reset(self) -> None:
        """Clear dispatcher-local counters without changing topic wiring."""
        self._published_primitive_count = 0
        self.last_reason = "live_dispatcher_reset"

    def dispatch(self, envelope: SchedulerDispatchEnvelope) -> DispatchResult:
        """Publish one scheduler envelope toward the GUI-compatible ROS2 runtime."""
        if not self._started:
            return self._rejected("dispatcher_not_started", envelope=envelope)
        if not bool(self.runtime_node.diagnostics().get("ros2_available")):
            return self._rejected("ros2_unavailable", envelope=envelope)
        if not self._publisher_created:
            return self._rejected("ros2_publisher_unavailable", envelope=envelope)
        commands = scheduler_envelope_to_gui_ros2_commands(envelope, source_name=self.source_name)
        if not commands:
            self.last_reason = "ros2_dispatch_noop"
            return DispatchResult(
                accepted=True,
                dispatched_commands=[],
                reason="ros2_dispatch_noop",
                diagnostics={
                    **self.runtime_diagnostics(),
                    "input_source": envelope.input_source,
                    "dispatch_target": envelope.dispatch_target or "ros2",
                    "published_command_count": 0.0,
                    "noop_requested": True,
                },
            )
        published_topics: list[str] = []
        dispatched_commands: list[str] = []
        for command in commands:
            if not self.runtime_node.publish_motor_command(command):
                return self._rejected("ros2_publish_failed", envelope=envelope)
            self._published_primitive_count += 1
            published_topics.append(command.command_topic)
            dispatched_commands.append(f"{command.namespace}:motor{command.motor_id}:0x{command.cmd_type:02X}")
        self.last_reason = "ros2_gui_dispatch_published"
        return DispatchResult(
            accepted=True,
            dispatched_commands=dispatched_commands,
            reason="ros2_gui_dispatch_published",
            diagnostics={
                **self.runtime_diagnostics(),
                "input_source": envelope.input_source,
                "dispatch_target": envelope.dispatch_target or "ros2",
                "graph_id": None
                if envelope.scheduled_command is None
                else envelope.scheduled_command.graph_id,
                "node_id": None
                if envelope.scheduled_command is None
                else envelope.scheduled_command.node_id,
                "skill_key": None
                if envelope.scheduled_command is None
                else envelope.scheduled_command.skill_key,
                "published_command_count": float(len(commands)),
                "published_primitive_count": self._published_primitive_count,
                "published_topic": published_topics[0],
                "published_topics": published_topics,
                "noop_requested": False,
                "stop_requested": False,
            },
        )

    def emergency_stop(self) -> DispatchResult:
        """Publish a structured emergency-stop command when ROS2 is available."""
        if (
            not self._started
            or not bool(self.runtime_node.diagnostics().get("ros2_available"))
            or not self._publisher_created
        ):
            return self._rejected("ros2_unavailable", emergency_stop_requested=True)
        commands = build_gui_ros2_emergency_stop_commands(
            GUI_ROS2_TOPIC_NAMESPACES,
            source_name=self.source_name,
        )
        published_topics: list[str] = []
        for command in commands:
            if not self.runtime_node.publish_motor_command(command):
                return self._rejected("ros2_publish_failed", emergency_stop_requested=True)
            published_topics.append(command.command_topic)
        self.last_reason = "ros2_emergency_stop_published"
        return DispatchResult(
            accepted=True,
            dispatched_commands=["emergency_stop"],
            reason="ros2_emergency_stop_published",
            diagnostics={
                **self.runtime_diagnostics(),
                "published_topics": published_topics,
                "published_topic": None if not published_topics else published_topics[0],
                "emergency_stop_requested": True,
            },
        )

    def flush(self) -> DispatchResult:
        """Return a structured flush result for the ROS2 dispatcher."""
        if not self._started or not bool(self.runtime_node.diagnostics().get("ros2_available")):
            return self._rejected("ros2_unavailable")
        return DispatchResult(
            accepted=True,
            dispatched_commands=[],
            reason="ros2_flush_noop",
            diagnostics=self.runtime_diagnostics(),
        )

    def runtime_diagnostics(self) -> dict[str, object]:
        """Return dispatcher diagnostics for runtime-session reporting."""
        node_diagnostics = self.runtime_node.diagnostics()
        return {
            **node_diagnostics,
            "dispatcher_kind": self.dispatcher_kind,
            "dispatcher": self.source_name,
            "source_name": self.source_name,
            "adapter_description": self.adapter_description,
            "started": self._started,
            "publisher_created": self._publisher_created,
            "ros2_available": bool(node_diagnostics.get("ros2_available")),
            "publish_target_topics": list(GUI_ROS2_COMMAND_TOPICS),
            "topic_namespaces": list(GUI_ROS2_TOPIC_NAMESPACES),
            "gui_ros2_compatible": True,
            "published_primitive_count": self._published_primitive_count,
            "reason": self.last_reason,
        }

    def _rejected(
        self,
        reason: str,
        *,
        envelope: SchedulerDispatchEnvelope | None = None,
        emergency_stop_requested: bool = False,
    ) -> DispatchResult:
        """Build a structured rejection result."""
        diagnostics = self.runtime_diagnostics()
        diagnostics["emergency_stop_requested"] = emergency_stop_requested
        if envelope is not None:
            diagnostics["input_source"] = envelope.input_source
            diagnostics["dispatch_target"] = envelope.dispatch_target or "ros2"
            diagnostics["scheduled_skill_key"] = (
                None if envelope.scheduled_command is None else envelope.scheduled_command.skill_key
            )
        return DispatchResult(
            accepted=False,
            dispatched_commands=[],
            reason=reason,
            diagnostics=diagnostics,
        )


__all__ = ["LiveRuntimeCommandDispatcher"]

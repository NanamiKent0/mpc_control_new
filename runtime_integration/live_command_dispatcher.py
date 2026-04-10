"""ROS2-backed live runtime dispatcher with graceful degradation."""

from __future__ import annotations

from ..control_core.models.runtime_bridge_types import DispatchResult, SchedulerDispatchEnvelope
from .ros2_message_adapters import envelope_to_ros2_command_payload
from .ros2_runtime_node import Ros2RuntimeNode


class LiveRuntimeCommandDispatcher:
    """Dispatch scheduler envelopes onto a ROS2-facing live runtime boundary."""

    dispatcher_kind = "ros2"

    def __init__(
        self,
        *,
        source_name: str = "live_command_dispatcher",
        adapter_description: str = "ros2 live runtime command dispatcher",
        runtime_node: Ros2RuntimeNode | None = None,
    ) -> None:
        self.source_name = source_name
        self.adapter_description = adapter_description
        self.runtime_node = runtime_node or Ros2RuntimeNode()
        self._started = False
        self._publisher_created = False
        self.last_reason = "live_dispatcher_not_started"

    def start(self) -> None:
        """Start the ROS2 dispatcher and create the command publisher when possible."""
        self.runtime_node.start()
        node_diagnostics = self.runtime_node.diagnostics()
        self._started = True
        if not bool(node_diagnostics.get("ros2_available")):
            self.last_reason = "ros2_unavailable"
            return
        self._publisher_created = self.runtime_node.create_command_publisher()
        self.last_reason = "ros2_dispatcher_started" if self._publisher_created else "ros2_publisher_unavailable"

    def stop(self) -> None:
        """Stop the ROS2 dispatcher."""
        self.runtime_node.stop()
        self._started = False
        self._publisher_created = False
        self.last_reason = "live_dispatcher_stopped"

    def dispatch(self, envelope: SchedulerDispatchEnvelope) -> DispatchResult:
        """Publish one scheduler envelope toward the ROS2 live runtime when possible."""
        if not self._started:
            return self._rejected("dispatcher_not_started", envelope=envelope)
        if not bool(self.runtime_node.diagnostics().get("ros2_available")):
            return self._rejected("ros2_unavailable", envelope=envelope)
        if not self._publisher_created:
            return self._rejected("ros2_publisher_unavailable", envelope=envelope)
        payload = envelope_to_ros2_command_payload(envelope, source_name=self.source_name)
        if not self.runtime_node.publish_command(payload):
            return self._rejected("ros2_publish_failed", envelope=envelope)
        self.last_reason = "ros2_dispatch_published"
        command_count = len(payload.commands)
        dispatched_commands = [command.primitive_name for command in payload.commands]
        if payload.noop_requested:
            dispatched_commands.append("noop")
        return DispatchResult(
            accepted=True,
            dispatched_commands=dispatched_commands,
            reason="ros2_dispatch_published",
            diagnostics={
                **self.runtime_diagnostics(),
                "input_source": envelope.input_source,
                "dispatch_target": envelope.dispatch_target or "ros2",
                "graph_id": payload.graph_id,
                "node_id": payload.node_id,
                "skill_key": payload.skill_key,
                "published_command_count": float(command_count),
                "noop_requested": payload.noop_requested,
            },
        )

    def emergency_stop(self) -> DispatchResult:
        """Publish a structured emergency-stop command when ROS2 is available."""
        if not self._started or not bool(self.runtime_node.diagnostics().get("ros2_available")) or not self._publisher_created:
            return self._rejected("ros2_unavailable", emergency_stop_requested=True)
        payload = {
            "graph_id": None,
            "node_id": None,
            "skill_key": None,
            "dispatch_target": "ros2",
            "stop_requested": True,
            "noop_requested": True,
            "commands": [],
            "diagnostics": {"source_name": self.source_name, "emergency_stop_requested": True},
        }
        self.runtime_node.publish_command(payload)
        self.last_reason = "ros2_emergency_stop_published"
        return DispatchResult(
            accepted=True,
            dispatched_commands=["emergency_stop"],
            reason="ros2_emergency_stop_published",
            diagnostics=self.runtime_diagnostics(),
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
            "dispatcher_kind": self.dispatcher_kind,
            "dispatcher": self.source_name,
            "source_name": self.source_name,
            "adapter_description": self.adapter_description,
            "started": self._started,
            "publisher_created": self._publisher_created,
            "reason": self.last_reason,
            **node_diagnostics,
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
            diagnostics["scheduled_skill_key"] = None if envelope.scheduled_command is None else envelope.scheduled_command.skill_key
        return DispatchResult(
            accepted=False,
            dispatched_commands=[],
            reason=reason,
            diagnostics=diagnostics,
        )


__all__ = ["LiveRuntimeCommandDispatcher"]

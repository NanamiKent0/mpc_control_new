"""ROS2-backed live runtime observation provider with graceful degradation."""

from __future__ import annotations

from .observation_types import RuntimeObservationFrame
from .ros2_message_adapters import runtime_frame_from_ros2_message
from .ros2_runtime_node import Ros2RuntimeNode


class LiveRuntimeObservationProvider:
    """Provide live runtime frames through a ROS2-facing integration boundary."""

    provider_kind = "ros2"

    def __init__(
        self,
        *,
        source_name: str = "live_runtime_provider",
        adapter_description: str = "ros2 live runtime observation provider",
        runtime_node: Ros2RuntimeNode | None = None,
        topology_hint: dict[str, object] | None = None,
    ) -> None:
        self.source_name = source_name
        self.adapter_description = adapter_description
        self.runtime_node = runtime_node or Ros2RuntimeNode()
        self.topology_hint = dict(topology_hint or {})
        self._started = False
        self._subscription_created = False
        self._latest_frame: RuntimeObservationFrame | None = None
        self.last_reason = "live_provider_not_started"

    def start(self) -> None:
        """Start the ROS2 provider and subscribe to the observation topic when possible."""
        self.runtime_node.start()
        node_diagnostics = self.runtime_node.diagnostics()
        self._started = True
        if not bool(node_diagnostics.get("ros2_available")):
            self.last_reason = "ros2_unavailable"
            return
        self._subscription_created = self.runtime_node.create_observation_subscription(self._handle_observation)
        self.last_reason = "ros2_provider_started" if self._subscription_created else "ros2_subscription_unavailable"

    def stop(self) -> None:
        """Stop the ROS2 provider and clear its local frame cache."""
        self.runtime_node.stop()
        self._started = False
        self._subscription_created = False
        self.last_reason = "live_provider_stopped"

    def warmup(self) -> RuntimeObservationFrame | None:
        """Spin once and return the latest cached frame when available."""
        return self.get_latest_frame()

    def is_ready(self) -> bool:
        """Return whether the ROS2 provider has a live subscription path."""
        return self._started and self._subscription_created and bool(self.runtime_node.diagnostics().get("ros2_available"))

    def get_latest_frame(self) -> RuntimeObservationFrame | None:
        """Return the latest cached ROS2-adapted frame."""
        if not self._started:
            self.last_reason = "live_provider_not_started"
            return None
        self.runtime_node.poll_once()
        if self._latest_frame is None and not self.runtime_node.diagnostics().get("ros2_available"):
            self.last_reason = "ros2_unavailable"
        return self._latest_frame

    def runtime_diagnostics(self) -> dict[str, object]:
        """Return provider diagnostics for runtime-session reporting."""
        node_diagnostics = self.runtime_node.diagnostics()
        return {
            "provider_kind": self.provider_kind,
            "provider": self.source_name,
            "source_name": self.source_name,
            "adapter_description": self.adapter_description,
            "started": self._started,
            "subscription_created": self._subscription_created,
            "reason": self.last_reason,
            "latest_frame_timestamp_ns": None if self._latest_frame is None else float(self._latest_frame.timestamp_ns),
            **node_diagnostics,
        }

    def _handle_observation(self, payload: object) -> None:
        """Adapt one ROS2-side observation payload into the runtime frame cache."""
        self._latest_frame = runtime_frame_from_ros2_message(
            payload,
            source_name=self.source_name,
            fallback_topology_hint=self.topology_hint,
        )
        self.last_reason = "ros2_frame_received"


__all__ = ["LiveRuntimeObservationProvider"]

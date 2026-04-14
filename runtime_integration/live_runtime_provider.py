"""ROS2-backed live runtime observation provider with GUI-topic compatibility."""

from __future__ import annotations

from .observation_types import RuntimeObservationFrame
from .ros2_interfaces import GUI_ROS2_FEEDBACK_TOPICS, GUI_ROS2_TOPIC_NAMESPACES
from .ros2_message_adapters import (
    normalize_gui_ros2_feedback_payload,
    runtime_frame_from_gui_ros2_feedbacks,
    runtime_frame_from_ros2_message,
)
from .ros2_runtime_node import Ros2RuntimeNode
from .ros2_topic_config import Ros2TopicConfig, build_default_ros2_topic_config


class LiveRuntimeObservationProvider:
    """Provide live runtime frames through GUI-compatible ROS2 feedback topics."""

    provider_kind = "ros2"

    def __init__(
        self,
        *,
        source_name: str = "live_runtime_provider",
        adapter_description: str = "ros2 live runtime observation provider",
        runtime_node: Ros2RuntimeNode | None = None,
        topic_config: Ros2TopicConfig | None = None,
        topology_hint: dict[str, object] | None = None,
    ) -> None:
        self.source_name = source_name
        self.adapter_description = adapter_description
        self.topic_config = topic_config or build_default_ros2_topic_config()
        self.runtime_node = runtime_node or Ros2RuntimeNode(topic_config=self.topic_config)
        self.topology_hint = dict(topology_hint or {})
        self._started = False
        self._feedback_subscription_created = False
        self._observation_subscription_created = False
        self._latest_frame: RuntimeObservationFrame | None = None
        self._latest_payload: object | None = None
        self._feedback_cache: dict[str, dict[int, object]] = {}
        self.last_reason = "live_provider_not_started"

    def start(self) -> None:
        """Start the ROS2 provider and subscribe to feedback topics when possible."""
        self.runtime_node.start()
        node_diagnostics = self.runtime_node.diagnostics()
        self._started = True
        if not bool(node_diagnostics.get("ros2_available")):
            self.last_reason = "ros2_unavailable"
            return
        self._feedback_subscription_created = self.runtime_node.create_feedback_subscriptions(
            self._handle_feedback,
            namespaces=GUI_ROS2_TOPIC_NAMESPACES,
        )
        self._observation_subscription_created = self.runtime_node.create_observation_subscription(
            self._handle_observation
        )
        self.last_reason = (
            "ros2_gui_feedback_subscribed"
            if self._feedback_subscription_created
            else (
                "ros2_observation_fallback_subscribed"
                if self._observation_subscription_created
                else "ros2_subscription_unavailable"
            )
        )

    def stop(self) -> None:
        """Stop the ROS2 provider and clear its local frame cache."""
        self.runtime_node.stop()
        self._started = False
        self._feedback_subscription_created = False
        self._observation_subscription_created = False
        self._latest_payload = None
        self._latest_frame = None
        self._feedback_cache = {}
        self.last_reason = "live_provider_stopped"

    def reset(self) -> None:
        """Clear local caches while preserving the runtime node wiring."""
        self._latest_payload = None
        self._latest_frame = None
        self._feedback_cache = {}
        self.last_reason = "live_provider_reset"

    def warmup(self) -> RuntimeObservationFrame | None:
        """Spin once and return the latest cached frame when available."""
        return self.get_latest_frame()

    def is_ready(self) -> bool:
        """Return whether the ROS2 provider has a live subscription path."""
        return (
            self._started
            and bool(self.runtime_node.diagnostics().get("ros2_available"))
            and (self._feedback_subscription_created or self._observation_subscription_created)
        )

    def get_latest_frame(self) -> RuntimeObservationFrame | None:
        """Return the latest cached ROS2-adapted frame."""
        if not self._started:
            self.last_reason = "live_provider_not_started"
            return None
        self.runtime_node.poll_once()
        latest_payload = self.runtime_node.get_latest_observation_payload()
        if latest_payload is not None and latest_payload is not self._latest_payload:
            self._handle_observation(latest_payload)
        if self._feedback_cache:
            self._latest_frame = runtime_frame_from_gui_ros2_feedbacks(
                self._feedback_cache,
                source_name=self.source_name,
                fallback_topology_hint=self.topology_hint,
            )
            self.last_reason = "ros2_gui_feedback_frame_ready"
        node_diagnostics = self.runtime_node.diagnostics()
        if self._latest_frame is None and not bool(node_diagnostics.get("ros2_available")):
            self.last_reason = "ros2_unavailable"
        elif self._latest_frame is None:
            self.last_reason = "frame_cache_empty"
        return self._latest_frame

    def runtime_diagnostics(self) -> dict[str, object]:
        """Return provider diagnostics for runtime-session reporting."""
        node_diagnostics = self.runtime_node.diagnostics()
        return {
            **node_diagnostics,
            "provider_kind": self.provider_kind,
            "provider": self.source_name,
            "source_name": self.source_name,
            "adapter_description": self.adapter_description,
            "started": self._started,
            "feedback_subscription_created": self._feedback_subscription_created,
            "observation_subscription_created": self._observation_subscription_created,
            "subscription_created": self._feedback_subscription_created
            or self._observation_subscription_created,
            "ros2_available": bool(node_diagnostics.get("ros2_available")),
            "frame_cache_ready": self._latest_frame is not None,
            "source_topics": list(GUI_ROS2_FEEDBACK_TOPICS),
            "topic_namespaces": list(GUI_ROS2_TOPIC_NAMESPACES),
            "gui_ros2_compatible": True,
            "reason": self.last_reason,
            "latest_frame_timestamp_ns": (
                None if self._latest_frame is None else float(self._latest_frame.timestamp_ns)
            ),
            "feedback_cache_size": float(sum(len(items) for items in self._feedback_cache.values())),
        }

    def _handle_observation(self, payload: object) -> None:
        """Adapt one generic ROS2-side observation payload into the runtime frame cache."""
        self._latest_payload = payload
        try:
            self._latest_frame = runtime_frame_from_ros2_message(
                payload,
                source_name=self.source_name,
                fallback_topology_hint=self.topology_hint,
            )
            self.last_reason = "ros2_generic_frame_received"
        except Exception:
            self.last_reason = "ros2_generic_frame_invalid"

    def _handle_feedback(self, namespace: str, payload: object) -> None:
        """Cache one namespace-scoped GUI feedback payload."""
        normalized = normalize_gui_ros2_feedback_payload(payload, namespace=namespace)
        namespace_cache = self._feedback_cache.setdefault(namespace, {})
        namespace_cache[int(normalized.motor_id)] = normalized
        self._latest_frame = runtime_frame_from_gui_ros2_feedbacks(
            self._feedback_cache,
            source_name=self.source_name,
            fallback_topology_hint=self.topology_hint,
        )
        self.last_reason = "ros2_gui_feedback_received"


__all__ = ["LiveRuntimeObservationProvider"]

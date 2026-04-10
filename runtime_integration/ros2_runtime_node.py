"""Minimal ROS2 runtime node wrapper with graceful degradation outside ROS2 environments."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable

from .ros2_interfaces import ROS2_RUNTIME_COMMAND_TOPIC, ROS2_RUNTIME_OBSERVATION_TOPIC


@dataclass(slots=True)
class Ros2Availability:
    """Describe whether a ROS2 runtime path is currently available."""

    ros2_available: bool
    backend: str
    reason: str | None = None
    import_error: str | None = None
    diagnostics: dict[str, object] = field(default_factory=dict)


class Ros2RuntimeNode:
    """Own the ROS2 node resources shared by the live provider and dispatcher."""

    def __init__(
        self,
        *,
        node_name: str = "mpc_control_new_runtime",
        observation_topic: str = ROS2_RUNTIME_OBSERVATION_TOPIC,
        command_topic: str = ROS2_RUNTIME_COMMAND_TOPIC,
        node_handle: object | None = None,
        enable_rclpy: bool = False,
    ) -> None:
        self.node_name = node_name
        self.observation_topic = observation_topic
        self.command_topic = command_topic
        self._external_node_handle = node_handle
        self._enable_rclpy = enable_rclpy
        self._node_handle: object | None = None
        self._owns_rclpy_context = False
        self._started = False
        self._publisher: object | None = None
        self._subscriptions: list[object] = []
        self._observation_callbacks: list[Callable[[object], None]] = []
        self._published_payloads: list[object] = []
        self._last_observation_payload: object | None = None
        self._resource_error: str | None = None
        self._availability = self._detect_availability()

    @property
    def availability(self) -> Ros2Availability:
        """Return the cached ROS2 availability snapshot."""
        return self._availability

    def start(self) -> None:
        """Start the ROS2 node when available, or degrade cleanly when unavailable."""
        if self._started:
            return
        if not self._availability.ros2_available:
            return
        if self._external_node_handle is not None:
            self._node_handle = self._external_node_handle
            self._started = True
            return

        rclpy_module = _safe_import_rclpy()
        if rclpy_module is None:
            self._availability = Ros2Availability(
                ros2_available=False,
                backend="unavailable",
                reason="rclpy_import_failed_at_start",
            )
            return
        node_module = _safe_import_rclpy_node()
        if node_module is None:
            self._availability = Ros2Availability(
                ros2_available=False,
                backend="unavailable",
                reason="rclpy_node_import_failed_at_start",
            )
            return
        if hasattr(rclpy_module, "init"):
            try:
                rclpy_module.init(args=None)
                self._owns_rclpy_context = True
            except Exception:
                self._owns_rclpy_context = False
        node_cls = getattr(node_module, "Node", None)
        if node_cls is None:
            self._availability = Ros2Availability(
                ros2_available=False,
                backend="unavailable",
                reason="rclpy_node_class_missing",
            )
            return
        self._node_handle = node_cls(self.node_name)
        self._started = True

    def stop(self) -> None:
        """Tear down publisher/subscription state and destroy the underlying node if owned."""
        if self._node_handle is not None and self._external_node_handle is None:
            destroy = getattr(self._node_handle, "destroy_node", None)
            if callable(destroy):
                destroy()
        if self._owns_rclpy_context:
            rclpy_module = _safe_import_rclpy()
            shutdown = None if rclpy_module is None else getattr(rclpy_module, "shutdown", None)
            if callable(shutdown):
                try:
                    shutdown()
                except Exception:
                    pass
        self._node_handle = None
        self._publisher = None
        self._subscriptions = []
        self._observation_callbacks = []
        self._started = False
        self._owns_rclpy_context = False

    def create_observation_subscription(self, callback: Callable[[object], None]) -> bool:
        """Create one ROS2 observation subscription or register a shim callback."""
        if not self._started or self._node_handle is None:
            return False
        self._observation_callbacks.append(callback)
        create_subscription = getattr(self._node_handle, "create_subscription", None)
        if callable(create_subscription):
            try:
                subscription = create_subscription(object, self.observation_topic, callback, 10)
            except Exception as exc:
                self._resource_error = f"create_subscription_failed:{exc.__class__.__name__}"
                return False
            self._subscriptions.append(subscription)
        return True

    def create_command_publisher(self) -> bool:
        """Create one ROS2 command publisher or reuse an existing shim publisher."""
        if not self._started or self._node_handle is None:
            return False
        if self._publisher is not None:
            return True
        create_publisher = getattr(self._node_handle, "create_publisher", None)
        if callable(create_publisher):
            try:
                self._publisher = create_publisher(object, self.command_topic, 10)
                return True
            except Exception as exc:
                self._resource_error = f"create_publisher_failed:{exc.__class__.__name__}"
                return False
        if self._external_node_handle is not None:
            self._publisher = _CallablePublisher(self._record_published_payload)
            return True
        return False

    def publish_command(self, payload: object) -> bool:
        """Publish one command payload through the ROS2 publisher abstraction."""
        if self._publisher is None:
            return False
        publish = getattr(self._publisher, "publish", None)
        if callable(publish):
            publish(payload)
        if not isinstance(self._publisher, _CallablePublisher):
            self._record_published_payload(payload)
        return True

    def poll_once(self) -> None:
        """Spin the ROS2 node once when backed by a real `rclpy` context."""
        if not self._started or self._external_node_handle is not None or self._node_handle is None:
            return
        rclpy_module = _safe_import_rclpy()
        spin_once = None if rclpy_module is None else getattr(rclpy_module, "spin_once", None)
        if callable(spin_once):
            try:
                spin_once(self._node_handle, timeout_sec=0.0)
            except Exception:
                return

    def push_observation(self, payload: object) -> None:
        """Inject one observation payload into the registered callbacks, mainly for tests."""
        self._last_observation_payload = payload
        for callback in list(self._observation_callbacks):
            callback(payload)

    def diagnostics(self) -> dict[str, object]:
        """Return stable diagnostics for provider/dispatcher status reporting."""
        return {
            "ros2_available": self._availability.ros2_available,
            "ros2_backend": self._availability.backend,
            "ros2_reason": self._availability.reason,
            "node_started": self._started,
            "subscriptions_created": len(self._subscriptions),
            "publishers_created": 0 if self._publisher is None else 1,
            "observation_topic": self.observation_topic,
            "command_topic": self.command_topic,
            "published_payload_count": len(self._published_payloads),
            "last_observation_received": self._last_observation_payload is not None,
            "resource_error": self._resource_error,
        }

    def published_payloads(self) -> list[object]:
        """Return a copy of the published ROS2 command payloads."""
        return list(self._published_payloads)

    def _record_published_payload(self, payload: object) -> None:
        """Record one published payload for diagnostics and tests."""
        self._published_payloads.append(payload)

    def _detect_availability(self) -> Ros2Availability:
        """Resolve whether a ROS2 runtime path is available in the current environment."""
        if self._external_node_handle is not None:
            return Ros2Availability(ros2_available=True, backend="shim", reason="external_node_handle")
        if not self._enable_rclpy:
            return Ros2Availability(
                ros2_available=False,
                backend="disabled",
                reason="rclpy_disabled_by_default",
            )
        rclpy_module = _safe_import_rclpy()
        if rclpy_module is None:
            return Ros2Availability(
                ros2_available=False,
                backend="unavailable",
                reason="rclpy_not_installed",
            )
        node_module = _safe_import_rclpy_node()
        if node_module is None:
            return Ros2Availability(
                ros2_available=False,
                backend="unavailable",
                reason="rclpy_node_not_available",
            )
        return Ros2Availability(ros2_available=True, backend="rclpy")


class _CallablePublisher:
    """Simple publisher shim used when the injected node handle has no publisher object."""

    def __init__(self, callback: Callable[[object], None]) -> None:
        self._callback = callback

    def publish(self, payload: object) -> None:
        """Forward one payload into the callback sink."""
        self._callback(payload)


def _safe_import_rclpy() -> object | None:
    """Import `rclpy` with an import guard."""
    try:
        import rclpy  # type: ignore

        return rclpy
    except Exception:
        return None


def _safe_import_rclpy_node() -> object | None:
    """Import `rclpy.node` with an import guard."""
    try:
        from rclpy import node as rclpy_node  # type: ignore

        return rclpy_node
    except Exception:
        return None

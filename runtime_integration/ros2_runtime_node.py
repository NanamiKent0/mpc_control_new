"""Minimal ROS2 runtime node wrapper with GUI-topic compatibility and graceful fallback."""

from __future__ import annotations

from dataclasses import dataclass, field
import json
from typing import Callable

from .ros2_interfaces import (
    GUI_ROS2_TOPIC_BINDINGS,
    GUI_ROS2_TOPIC_NAMESPACES,
    GuiRos2MotorCommandPayload,
    GuiRos2MotorFeedbackPayload,
    build_gui_ros2_topic_bindings,
)
from .ros2_message_adapters import (
    gui_ros2_command_payload_to_array,
    gui_ros2_feedback_payload_to_array,
    normalize_gui_ros2_command_payload,
    normalize_gui_ros2_feedback_payload,
    normalize_ros2_observation_payload,
    ros2_command_payload_to_mapping,
)
from .ros2_topic_config import Ros2TopicConfig, build_default_ros2_topic_config


@dataclass(slots=True)
class Ros2Availability:
    """Describe whether a ROS2 runtime path is currently available."""

    ros2_available: bool
    backend: str
    reason: str | None = None
    import_error: str | None = None
    diagnostics: dict[str, object] = field(default_factory=dict)


class LoopbackRclpyContext:
    """Tiny `rclpy`-like context used by GUI/sim fake wiring."""

    def __init__(self) -> None:
        self._ok = False

    def init(self, args: object | None = None) -> None:
        del args
        self._ok = True

    def ok(self) -> bool:
        return self._ok

    def shutdown(self) -> None:
        self._ok = False

    def spin_once(self, node: object, timeout_sec: float = 0.0) -> None:
        del node, timeout_sec
        return None


class LoopbackRos2Broker:
    """In-memory topic broker that behaves like a tiny ROS2 graph for tests/demos."""

    def __init__(self) -> None:
        self._subscriptions: dict[str, list[Callable[[object], None]]] = {}

    def create_node(self, node_name: str) -> "LoopbackRos2NodeHandle":
        return LoopbackRos2NodeHandle(self, node_name=node_name)

    def subscribe(self, topic: str, callback: Callable[[object], None]) -> None:
        self._subscriptions.setdefault(topic, []).append(callback)

    def unsubscribe(self, topic: str, callback: Callable[[object], None]) -> None:
        callbacks = self._subscriptions.get(topic)
        if not callbacks:
            return
        self._subscriptions[topic] = [item for item in callbacks if item is not callback]
        if not self._subscriptions[topic]:
            self._subscriptions.pop(topic, None)

    def publish(self, topic: str, payload: object) -> None:
        for callback in list(self._subscriptions.get(topic, [])):
            callback(payload)


class LoopbackRos2NodeHandle:
    """ROS2-like node handle backed by the in-memory loopback broker."""

    def __init__(self, broker: LoopbackRos2Broker, *, node_name: str) -> None:
        self.broker = broker
        self.node_name = node_name
        self._subscriptions: list[tuple[str, Callable[[object], None]]] = []

    def create_subscription(
        self,
        msg_type: object,
        topic: str,
        callback: Callable[[object], None],
        qos: int,
    ) -> object:
        del msg_type, qos
        self.broker.subscribe(topic, callback)
        self._subscriptions.append((topic, callback))
        return callback

    def create_publisher(self, msg_type: object, topic: str, qos: int) -> "_LoopbackPublisher":
        del msg_type, qos
        return _LoopbackPublisher(self.broker, topic=topic)

    def destroy_node(self) -> None:
        for topic, callback in self._subscriptions:
            self.broker.unsubscribe(topic, callback)
        self._subscriptions = []


class Ros2RuntimeNode:
    """Own the ROS2 node resources shared by the live provider and dispatcher."""

    def __init__(
        self,
        *,
        node_name: str = "mpc_control_new_runtime",
        topic_config: Ros2TopicConfig | None = None,
        observation_topic: str | None = None,
        command_topic: str | None = None,
        node_handle: object | None = None,
        enable_rclpy: bool = False,
    ) -> None:
        self.topic_config = topic_config or build_default_ros2_topic_config()
        self.node_name = node_name
        self.observation_topic = observation_topic or self.topic_config.observation.frame_topic
        self.command_topic = command_topic or self.topic_config.command.command_topic
        self._external_node_handle = node_handle
        self._enable_rclpy = enable_rclpy
        self._node_handle: object | None = None
        self._owns_rclpy_context = False
        self._started = False
        self._publisher: object | None = None
        self._subscriptions: list[object] = []
        self._command_publishers: dict[str, object] = {}
        self._feedback_subscriptions: dict[str, object] = {}
        self._observation_callbacks: list[Callable[[object], None]] = []
        self._feedback_callbacks: list[Callable[[str, object], None]] = []
        self._published_payloads: list[object] = []
        self._last_observation_payload: object | None = None
        self._latest_feedback_payloads: dict[str, GuiRos2MotorFeedbackPayload] = {}
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
        node_module = _safe_import_rclpy_node()
        node_cls = None if node_module is None else getattr(node_module, "Node", None)
        if rclpy_module is None or node_cls is None:
            self._availability = Ros2Availability(
                ros2_available=False,
                backend="unavailable",
                reason="rclpy_runtime_unavailable",
            )
            return
        try:
            ok = getattr(rclpy_module, "ok", None)
            already_ok = bool(ok()) if callable(ok) else False
        except Exception:
            already_ok = False
        if not already_ok and hasattr(rclpy_module, "init"):
            try:
                rclpy_module.init(args=None)
                self._owns_rclpy_context = True
            except Exception as exc:
                self._availability = Ros2Availability(
                    ros2_available=False,
                    backend="unavailable",
                    reason="rclpy_init_failed",
                    import_error=str(exc),
                )
                return
        try:
            self._node_handle = node_cls(self.node_name)
        except Exception as exc:
            self._availability = Ros2Availability(
                ros2_available=False,
                backend="unavailable",
                reason="node_creation_failed",
                import_error=str(exc),
            )
            return
        self._started = True

    def stop(self) -> None:
        """Tear down publisher/subscription state and destroy the underlying node if owned."""
        if self._node_handle is not None and self._external_node_handle is None:
            destroy = getattr(self._node_handle, "destroy_node", None)
            if callable(destroy):
                try:
                    destroy()
                except Exception:
                    pass
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
        self._command_publishers = {}
        self._feedback_subscriptions = {}
        self._observation_callbacks = []
        self._feedback_callbacks = []
        self._started = False
        self._owns_rclpy_context = False

    def create_observation_subscription(self, callback: Callable[[object], None]) -> bool:
        """Create one generic runtime-observation subscription or register a shim callback."""
        if not self._started or self._node_handle is None:
            return False
        self._observation_callbacks.append(callback)
        create_subscription = getattr(self._node_handle, "create_subscription", None)
        if not callable(create_subscription):
            if self._external_node_handle is not None:
                self._subscriptions.append(callback)
                return True
            return False
        try:
            message_type = _safe_import_ros2_string_message()
            subscription = create_subscription(
                object if message_type is None else message_type,
                self.observation_topic,
                self._receive_observation_payload,
                self.topic_config.observation.qos_depth,
            )
        except Exception as exc:
            self._resource_error = f"create_observation_subscription_failed:{exc.__class__.__name__}"
            return False
        self._subscriptions.append(subscription)
        return True

    def create_command_publisher(self) -> bool:
        """Create one generic runtime-command publisher or reuse a shim publisher."""
        if not self._started or self._node_handle is None:
            return False
        if self._publisher is not None:
            return True
        create_publisher = getattr(self._node_handle, "create_publisher", None)
        if callable(create_publisher):
            try:
                message_type = _safe_import_ros2_string_message()
                self._publisher = create_publisher(
                    object if message_type is None else message_type,
                    self.command_topic,
                    self.topic_config.command.qos_depth,
                )
                return True
            except Exception as exc:
                self._resource_error = f"create_command_publisher_failed:{exc.__class__.__name__}"
                return False
        if self._external_node_handle is not None:
            self._publisher = _CallablePublisher()
            return True
        return False

    def publish_command(self, payload: object) -> bool:
        """Publish one generic runtime command payload through the node abstraction."""
        if self._publisher is None:
            return False
        publish = getattr(self._publisher, "publish", None)
        publishable_payload = self._prepare_publish_payload(payload)
        try:
            if callable(publish):
                publish(publishable_payload)
        except Exception as exc:
            self._resource_error = f"publish_failed:{exc.__class__.__name__}"
            return False
        self._record_published_payload(payload)
        return True

    def create_feedback_subscriptions(
        self,
        callback: Callable[[str, object], None],
        *,
        namespaces: tuple[str, ...] | list[str] | None = None,
    ) -> bool:
        """Subscribe to the GUI-facing `motor_feedback` topics across namespaces."""
        if not self._started or self._node_handle is None:
            return False
        create_subscription = getattr(self._node_handle, "create_subscription", None)
        if not callable(create_subscription):
            return False
        message_type = _safe_import_ros2_int64_multi_array()
        self._feedback_callbacks.append(callback)
        created = False
        for binding in build_gui_ros2_topic_bindings(namespaces):
            try:
                subscription = create_subscription(
                    object if message_type is None else message_type,
                    binding.feedback_topic,
                    lambda msg, namespace=binding.namespace: self._receive_feedback_payload(
                        namespace,
                        msg,
                    ),
                    self.topic_config.observation.qos_depth,
                )
            except Exception as exc:
                self._resource_error = (
                    f"create_feedback_subscription_failed:{binding.namespace}:{exc.__class__.__name__}"
                )
                continue
            self._feedback_subscriptions[binding.namespace] = subscription
            created = True
        return created

    def create_motor_command_publishers(
        self,
        *,
        namespaces: tuple[str, ...] | list[str] | None = None,
    ) -> bool:
        """Create publishers for the GUI-facing `motor_command` topics."""
        if not self._started or self._node_handle is None:
            return False
        create_publisher = getattr(self._node_handle, "create_publisher", None)
        if not callable(create_publisher):
            return False
        message_type = _safe_import_ros2_int32_multi_array()
        created = False
        for binding in build_gui_ros2_topic_bindings(namespaces):
            if binding.namespace in self._command_publishers:
                created = True
                continue
            try:
                publisher = create_publisher(
                    object if message_type is None else message_type,
                    binding.command_topic,
                    self.topic_config.command.qos_depth,
                )
            except Exception as exc:
                self._resource_error = (
                    f"create_motor_command_publisher_failed:{binding.namespace}:{exc.__class__.__name__}"
                )
                continue
            self._command_publishers[binding.namespace] = publisher
            created = True
        return created

    def publish_motor_command(
        self,
        payload: GuiRos2MotorCommandPayload | dict[str, object] | object,
    ) -> bool:
        """Publish one GUI-facing compact motor command payload."""
        normalized = normalize_gui_ros2_command_payload(payload)
        publisher = self._command_publishers.get(normalized.namespace)
        if publisher is None:
            return False
        publish = getattr(publisher, "publish", None)
        message = _build_int32_multi_array_message(gui_ros2_command_payload_to_array(normalized))
        try:
            if callable(publish):
                publish(message)
        except Exception as exc:
            self._resource_error = f"publish_motor_command_failed:{exc.__class__.__name__}"
            return False
        self._record_published_payload(normalized)
        return True

    def push_observation(self, payload: object) -> None:
        """Inject one generic observation payload, mainly for tests."""
        self._receive_observation_payload(payload)

    def push_gui_feedback(
        self,
        namespace: str,
        payload: GuiRos2MotorFeedbackPayload | dict[str, object] | object,
    ) -> None:
        """Inject one GUI feedback payload, mainly for tests."""
        normalized = normalize_gui_ros2_feedback_payload(payload, namespace=namespace)
        message = _build_int64_multi_array_message(gui_ros2_feedback_payload_to_array(normalized))
        self._receive_feedback_payload(namespace, message)

    def poll_once(self) -> None:
        """Spin the ROS2 node once when backed by a real `rclpy` context."""
        if (
            not self._started
            or self._external_node_handle is not None
            or self._node_handle is None
            or not self._availability.backend.startswith("rclpy")
        ):
            return
        rclpy_module = _safe_import_rclpy()
        spin_once = None if rclpy_module is None else getattr(rclpy_module, "spin_once", None)
        if callable(spin_once):
            try:
                spin_once(self._node_handle, timeout_sec=0.0)
            except Exception:
                return

    def get_latest_observation_payload(self) -> object | None:
        """Return the latest generic observation payload cached by the node."""
        return self._last_observation_payload

    def latest_feedback_payloads(self) -> dict[str, GuiRos2MotorFeedbackPayload]:
        """Return the latest GUI feedback payload per namespace."""
        return dict(self._latest_feedback_payloads)

    def unavailable_diagnostics(self, *, component_kind: str) -> dict[str, object]:
        """Return a structured unavailable snapshot for provider/dispatcher callers."""
        return {
            "component_kind": component_kind,
            "ros2_available": self._availability.ros2_available,
            "ros2_backend": self._availability.backend,
            "ros2_reason": self._availability.reason,
            "topic_config": self.topic_config.to_dict(),
        }

    def diagnostics(self) -> dict[str, object]:
        """Return stable diagnostics for provider/dispatcher status reporting."""
        return {
            "ros2_available": self._availability.ros2_available,
            "ros2_backend": self._availability.backend,
            "ros2_reason": self._availability.reason,
            "node_started": self._started,
            "subscriptions_created": len(self._subscriptions) + len(self._feedback_subscriptions),
            "publishers_created": (0 if self._publisher is None else 1) + len(self._command_publishers),
            "observation_topic": self.observation_topic,
            "command_topic": self.command_topic,
            "source_topics": self.topic_config.source_topics(),
            "publish_target_topics": self.topic_config.target_topics(),
            "topic_namespaces": list(GUI_ROS2_TOPIC_NAMESPACES),
            "gui_feedback_topics": [binding.feedback_topic for binding in GUI_ROS2_TOPIC_BINDINGS],
            "gui_command_topics": [binding.command_topic for binding in GUI_ROS2_TOPIC_BINDINGS],
            "gui_ros2_compatible": True,
            "published_payload_count": len(self._published_payloads),
            "frame_cache_ready": self._last_observation_payload is not None
            or bool(self._latest_feedback_payloads),
            "last_observation_received": self._last_observation_payload is not None,
            "feedback_cache_namespaces": sorted(self._latest_feedback_payloads),
            "resource_error": self._resource_error,
        }

    def published_payloads(self) -> list[object]:
        """Return a copy of the published ROS2 command payloads."""
        return list(self._published_payloads)

    def _receive_observation_payload(self, payload: object) -> None:
        """Normalize and cache one incoming generic observation payload."""
        self._last_observation_payload = normalize_ros2_observation_payload(payload)
        for callback in list(self._observation_callbacks):
            callback(self._last_observation_payload)

    def _receive_feedback_payload(self, namespace: str, payload: object) -> None:
        """Normalize and cache one incoming GUI feedback payload."""
        normalized = normalize_gui_ros2_feedback_payload(payload, namespace=namespace)
        self._latest_feedback_payloads[namespace] = normalized
        for callback in list(self._feedback_callbacks):
            callback(namespace, normalized)

    def _prepare_publish_payload(self, payload: object) -> object:
        """Prepare one generic command payload for the selected ROS2 backend."""
        message_type = _safe_import_ros2_string_message()
        if message_type is None:
            return payload
        mapping = ros2_command_payload_to_mapping(payload)
        message = message_type()
        setattr(message, "data", json.dumps(mapping, sort_keys=True))
        return message

    def _record_published_payload(self, payload: object) -> None:
        """Record one published payload for diagnostics and tests."""
        self._published_payloads.append(payload)

    def _detect_availability(self) -> Ros2Availability:
        """Resolve whether a ROS2 runtime path is available in the current environment."""
        if self._external_node_handle is not None:
            return Ros2Availability(
                ros2_available=True,
                backend="shim",
                reason="external_node_handle",
                diagnostics={"topic_config": self.topic_config.to_dict()},
            )
        if not self._enable_rclpy:
            return Ros2Availability(
                ros2_available=False,
                backend="disabled",
                reason="rclpy_disabled_by_default",
                diagnostics={"topic_config": self.topic_config.to_dict()},
            )
        rclpy_module = _safe_import_rclpy()
        if rclpy_module is None:
            return Ros2Availability(
                ros2_available=False,
                backend="unavailable",
                reason="rclpy_not_installed",
                diagnostics={"topic_config": self.topic_config.to_dict()},
            )
        node_module = _safe_import_rclpy_node()
        if node_module is None:
            return Ros2Availability(
                ros2_available=False,
                backend="unavailable",
                reason="rclpy_node_not_available",
                diagnostics={"topic_config": self.topic_config.to_dict()},
            )
        int32_type = _safe_import_ros2_int32_multi_array()
        int64_type = _safe_import_ros2_int64_multi_array()
        if int32_type is None or int64_type is None:
            return Ros2Availability(
                ros2_available=False,
                backend="unavailable",
                reason="std_msgs_multi_array_not_available",
                diagnostics={"topic_config": self.topic_config.to_dict()},
            )
        return Ros2Availability(
            ros2_available=True,
            backend="rclpy_multiarray",
            diagnostics={"topic_config": self.topic_config.to_dict()},
        )


class _CallablePublisher:
    """Simple publisher shim used when the injected node handle has no publisher object."""

    def publish(self, payload: object) -> None:
        del payload
        return None


class _LoopbackPublisher:
    """Publish onto a loopback broker topic."""

    def __init__(self, broker: LoopbackRos2Broker, *, topic: str) -> None:
        self._broker = broker
        self._topic = topic

    def publish(self, payload: object) -> None:
        self._broker.publish(self._topic, payload)


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


def _safe_import_ros2_string_message() -> object | None:
    """Import `std_msgs.msg.String` with an import guard."""
    try:
        from std_msgs.msg import String  # type: ignore

        return String
    except Exception:
        return None


def _safe_import_ros2_int32_multi_array() -> object | None:
    """Import `std_msgs.msg.Int32MultiArray` with an import guard."""
    try:
        from std_msgs.msg import Int32MultiArray  # type: ignore

        return Int32MultiArray
    except Exception:
        return None


def _safe_import_ros2_int64_multi_array() -> object | None:
    """Import `std_msgs.msg.Int64MultiArray` with an import guard."""
    try:
        from std_msgs.msg import Int64MultiArray  # type: ignore

        return Int64MultiArray
    except Exception:
        return None


def _build_int32_multi_array_message(data: list[int]) -> object:
    """Build an `Int32MultiArray`-like object from a plain integer list."""
    message_type = _safe_import_ros2_int32_multi_array()
    if message_type is None:
        return _ArrayMessage(data)
    message = message_type()
    setattr(message, "data", list(data))
    return message


def _build_int64_multi_array_message(data: list[int]) -> object:
    """Build an `Int64MultiArray`-like object from a plain integer list."""
    message_type = _safe_import_ros2_int64_multi_array()
    if message_type is None:
        return _ArrayMessage(data)
    message = message_type()
    setattr(message, "data", list(data))
    return message


class _ArrayMessage:
    """Tiny fallback message object used when std_msgs is unavailable."""

    def __init__(self, data: list[int]) -> None:
        self.data = list(data)


__all__ = [
    "LoopbackRos2Broker",
    "LoopbackRos2NodeHandle",
    "LoopbackRclpyContext",
    "Ros2Availability",
    "Ros2RuntimeNode",
]

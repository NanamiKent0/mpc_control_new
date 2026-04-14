"""Backend discovery, fan-out dispatch, and feedback aggregation for `gui_ros2.py`."""

from __future__ import annotations

from dataclasses import dataclass, field
import time
from typing import Callable

from PyQt5.QtCore import QObject, QTimer, pyqtSignal

from ..common.encoder_protocol import (
    STATUS_STALE_SOURCE,
    ParsedFeedback,
    motor_ids_for_namespace_or_joint_name,
    parse_feedback_array,
)
from ..ros2_interfaces import GUI_ROS2_TOPIC_NAMESPACES
from ..sim_ros2_bridge import SimRos2Bridge
from ..ros2_runtime_node import LoopbackRos2Broker, LoopbackRclpyContext


BACKEND_SOURCE_SIM = "sim"
BACKEND_SOURCE_LIVE = "live"
BACKEND_SOURCES: tuple[str, str] = (BACKEND_SOURCE_SIM, BACKEND_SOURCE_LIVE)

BACKEND_STATUS_ONLINE = "online"
BACKEND_STATUS_OFFLINE = "offline"


@dataclass(slots=True)
class BackendStatusSnapshot:
    """Structured backend availability snapshot used by the GUI and tests."""

    source: str
    enabled: bool
    started: bool
    online: bool
    reason: str
    feedback_publishers: dict[str, int] = field(default_factory=dict)
    command_subscribers: dict[str, int] = field(default_factory=dict)
    feedback_cache_namespaces: tuple[str, ...] = ()
    diagnostics: dict[str, object] = field(default_factory=dict)

    def to_dict(self) -> dict[str, object]:
        """Return a stable dict representation."""
        return {
            "source": self.source,
            "enabled": self.enabled,
            "started": self.started,
            "online": self.online,
            "status": BACKEND_STATUS_ONLINE if self.online else BACKEND_STATUS_OFFLINE,
            "reason": self.reason,
            "feedback_publishers": dict(self.feedback_publishers),
            "command_subscribers": dict(self.command_subscribers),
            "feedback_cache_namespaces": list(self.feedback_cache_namespaces),
            "diagnostics": dict(self.diagnostics),
        }


class Ros2TopicBackend:
    """One backend endpoint that speaks the compact GUI ROS2 topic contract."""

    def __init__(
        self,
        *,
        source: str,
        ros_runtime: object,
        enabled: bool = True,
        node_name_prefix: str = "gui_backend",
        assume_online_if_started: bool = False,
        feedback_callback: Callable[[str, str, ParsedFeedback], None] | None = None,
        log_callback: Callable[[str, str], None] | None = None,
    ) -> None:
        self.source = source
        self.ros_runtime = ros_runtime
        self.enabled = bool(enabled)
        self.node_name_prefix = node_name_prefix
        self.assume_online_if_started = bool(assume_online_if_started)
        self.feedback_callback = feedback_callback
        self.log_callback = log_callback
        self.node = None
        self.started = False
        self.reason = "backend_disabled" if not self.enabled else "backend_not_started"
        self._owns_context = False
        self._publishers: dict[str, object] = {}
        self._subscriptions: dict[str, object] = {}
        self._latest_feedbacks: dict[str, dict[int, ParsedFeedback]] = {}
        self._last_feedback_log_ts: dict[tuple[str, str, int], float] = {}
        self._last_stale_warning_ts: dict[tuple[str, str, int], float] = {}

    def start(self) -> None:
        """Start the backend node and attach all namespace publishers/subscribers."""
        if not self.enabled or self.started:
            return
        try:
            if not self.ros_runtime.ok():
                self.ros_runtime.init()
                self._owns_context = True
            node_name = f"{self.node_name_prefix}_{self.source}_{int(time.time() * 1000)}"
            self.node = self.ros_runtime.create_node(node_name)
            self._create_topic_bindings()
            self.started = True
            self.reason = "topic_bindings_ready"
        except Exception as exc:
            self.reason = f"backend_start_failed:{exc.__class__.__name__}"
            self._emit_log(f"[{self.source}] backend 启动失败: {exc}", "error")

    def stop(self) -> None:
        """Stop the backend node and release owned runtime resources."""
        if self.node is not None:
            destroy = getattr(self.node, "destroy_node", None)
            if callable(destroy):
                try:
                    destroy()
                except Exception:
                    pass
        if self._owns_context:
            try:
                self.ros_runtime.shutdown()
            except Exception:
                pass
        self.node = None
        self.started = False
        self._owns_context = False
        self._publishers = {}
        self._subscriptions = {}
        self.reason = "backend_stopped"

    def poll(self, *, spin_burst: int = 8) -> None:
        """Spin the runtime node a few times when the backend uses a real ROS2 context."""
        if not self.started or self.node is None:
            return
        for _ in range(max(int(spin_burst), 1)):
            try:
                self.ros_runtime.spin_once(self.node, timeout_sec=0.0)
            except Exception:
                break

    def send_compact_command(
        self,
        *,
        namespace: str,
        motor_id: int,
        cmd_type: int,
        param1: int = 0,
        param2: int = 0,
        param3: int = 0,
    ) -> bool:
        """Publish one compact command onto one namespace-scoped topic."""
        if not self.started:
            self.reason = "backend_not_started"
            return False
        publisher = self._publishers.get(namespace)
        if publisher is None:
            self.reason = f"publisher_missing:{namespace}"
            return False
        try:
            message = self.ros_runtime.make_command_message()
            message.data = [int(motor_id), int(cmd_type), int(param1), int(param2), int(param3)]
            publisher.publish(message)
            self.reason = f"command_published:{namespace}"
            return True
        except Exception as exc:
            self.reason = f"command_publish_failed:{exc.__class__.__name__}"
            self._emit_log(
                f"[{self.source}] 发送 {namespace}/motor_command 失败: {exc}",
                "error",
            )
            return False

    def snapshot_status(self) -> BackendStatusSnapshot:
        """Return the latest backend snapshot used by the dispatch manager."""
        feedback_publishers = self._topic_counts("count_publishers", suffix="motor_feedback")
        command_subscribers = self._topic_counts("count_subscribers", suffix="motor_command")
        graph_has_endpoints = any(feedback_publishers.values()) or any(command_subscribers.values())
        if not self.enabled:
            online = False
            reason = "backend_disabled"
        elif not self.started:
            online = False
            reason = self.reason
        elif feedback_publishers or command_subscribers:
            online = bool(graph_has_endpoints)
            reason = "graph_endpoints_detected" if online else "graph_endpoints_missing"
        else:
            online = bool(self.assume_online_if_started or self.started)
            reason = self.reason
        diagnostics = {
            "node_created": self.node is not None,
            "publishers_created": len(self._publishers),
            "subscriptions_created": len(self._subscriptions),
        }
        return BackendStatusSnapshot(
            source=self.source,
            enabled=self.enabled,
            started=self.started,
            online=online,
            reason=reason,
            feedback_publishers=feedback_publishers,
            command_subscribers=command_subscribers,
            feedback_cache_namespaces=tuple(sorted(self._latest_feedbacks)),
            diagnostics=diagnostics,
        )

    def latest_feedbacks(self) -> dict[str, dict[int, ParsedFeedback]]:
        """Return a copy of the namespace->motor feedback cache."""
        return {
            namespace: dict(feedbacks)
            for namespace, feedbacks in self._latest_feedbacks.items()
        }

    def step_once(self) -> None:
        """Optional hook implemented by backends that can advance themselves."""
        return None

    def _create_topic_bindings(self) -> None:
        if self.node is None:
            raise RuntimeError("backend_node_missing")
        create_publisher = getattr(self.node, "create_publisher", None)
        create_subscription = getattr(self.node, "create_subscription", None)
        if not callable(create_publisher) or not callable(create_subscription):
            raise RuntimeError("backend_node_incompatible")
        message_cls_cmd = object if self.ros_runtime.int32_message_cls is None else self.ros_runtime.int32_message_cls
        message_cls_fb = object if self.ros_runtime.int64_message_cls is None else self.ros_runtime.int64_message_cls
        for namespace in GUI_ROS2_TOPIC_NAMESPACES:
            self._publishers[namespace] = create_publisher(
                message_cls_cmd,
                f"{namespace}/motor_command",
                10,
            )
            self._subscriptions[namespace] = create_subscription(
                message_cls_fb,
                f"{namespace}/motor_feedback",
                lambda msg, namespace_key=namespace: self._handle_feedback(namespace_key, msg),
                10,
            )

    def _handle_feedback(self, namespace: str, message: object) -> None:
        try:
            parsed = parse_feedback_array(getattr(message, "data", []))
        except Exception as exc:
            self._emit_log(
                f"[{self.source}] 解析 {namespace}/motor_feedback 失败: {exc}",
                "error",
            )
            return
        namespace_cache = self._latest_feedbacks.setdefault(namespace, {})
        namespace_cache[int(parsed.motor_id)] = parsed
        self._emit_feedback_log(namespace, parsed)
        if callable(self.feedback_callback):
            self.feedback_callback(self.source, namespace, parsed)

    def _emit_feedback_log(self, namespace: str, parsed: ParsedFeedback) -> None:
        now_ts = time.time()
        key = (self.source, namespace, int(parsed.motor_id))
        last_log_ts = self._last_feedback_log_ts.get(key, 0.0)
        if (last_log_ts == 0.0) or (parsed.is_extended and (now_ts - last_log_ts) >= 2.0):
            self._last_feedback_log_ts[key] = now_ts
            self._emit_log(
                f"[{self.source}] 收到 {namespace}/motor_feedback: motor={parsed.motor_id}, "
                f"raw={parsed.raw_pulses}, seq={parsed.seq}, flags=0x{int(parsed.status_flags or 0):X}",
                "recv",
            )
        if parsed.status_flags and (parsed.status_flags & STATUS_STALE_SOURCE):
            last_warn_ts = self._last_stale_warning_ts.get(key, 0.0)
            if (now_ts - last_warn_ts) >= 2.0:
                self._last_stale_warning_ts[key] = now_ts
                self._emit_log(
                    f"[{self.source}] {namespace} motor{parsed.motor_id} 反馈源陈旧(stale_source)",
                    "warning",
                )

    def _topic_counts(self, method_name: str, *, suffix: str) -> dict[str, int]:
        if self.node is None:
            return {}
        counter = getattr(self.node, method_name, None)
        if not callable(counter):
            return {}
        counts: dict[str, int] = {}
        for namespace in GUI_ROS2_TOPIC_NAMESPACES:
            topic_name = f"{namespace}/{suffix}"
            try:
                counts[namespace] = int(counter(topic_name))
            except Exception:
                counts[namespace] = 0
        return counts

    def _emit_log(self, message: str, level: str) -> None:
        if callable(self.log_callback):
            self.log_callback(message, level)


class EmbeddedSimBackend(Ros2TopicBackend):
    """Embedded self-contained sim backend exposed through the GUI topic contract."""

    def __init__(
        self,
        *,
        enabled: bool = False,
        auto_step_ms: int = 50,
        sim_backend: object | None = None,
        feedback_callback: Callable[[str, str, ParsedFeedback], None] | None = None,
        log_callback: Callable[[str, str], None] | None = None,
    ) -> None:
        self.broker = LoopbackRos2Broker()
        self.loopback_context = LoopbackRclpyContext()
        self.bridge = SimRos2Bridge(
            backend=sim_backend,
            node_handle=self.broker.create_node("sim_ros2_bridge"),
        )
        self.auto_step_ms = max(int(auto_step_ms), 0)
        self._timer: QTimer | None = None
        super().__init__(
            source=BACKEND_SOURCE_SIM,
            ros_runtime=_build_loopback_runtime(self.broker, self.loopback_context),
            enabled=enabled,
            node_name_prefix="gui_sim_backend",
            assume_online_if_started=True,
            feedback_callback=feedback_callback,
            log_callback=log_callback,
        )

    def start(self) -> None:
        super().start()
        if not self.started:
            return
        self.bridge.start()
        if self.auto_step_ms > 0 and self._timer is None:
            self._timer = QTimer()
            self._timer.timeout.connect(self.step_once)
            self._timer.start(self.auto_step_ms)
        self.reason = "embedded_sim_ready"

    def stop(self) -> None:
        if self._timer is not None:
            self._timer.stop()
            self._timer = None
        self.bridge.stop()
        super().stop()

    def step_once(self) -> None:
        self.bridge.step()

    def snapshot_status(self) -> BackendStatusSnapshot:
        snapshot = super().snapshot_status()
        diagnostics = dict(snapshot.diagnostics)
        if snapshot.started and self.enabled:
            diagnostics["bridge"] = self.bridge.snapshot()
        return BackendStatusSnapshot(
            source=snapshot.source,
            enabled=snapshot.enabled,
            started=snapshot.started,
            online=bool(snapshot.started and self.enabled),
            reason="embedded_sim_ready" if snapshot.started and self.enabled else snapshot.reason,
            feedback_publishers=snapshot.feedback_publishers,
            command_subscribers=snapshot.command_subscribers,
            feedback_cache_namespaces=snapshot.feedback_cache_namespaces,
            diagnostics=diagnostics,
        )


class NamespaceDispatchProxy(QObject):
    """Namespace-scoped proxy that lets one widget talk to the shared manager."""

    log_signal = pyqtSignal(str, str)
    connected_signal = pyqtSignal()
    disconnected_signal = pyqtSignal()
    feedback_signal = pyqtSignal(int, int)
    sourced_feedback_signal = pyqtSignal(str, int, int, "PyQt_PyObject")

    def __init__(self, namespace: str, manager: "GuiBackendManager") -> None:
        super().__init__()
        self.topic_ns = namespace
        self.manager = manager
        self.connected = False
        self.latest_feedback_by_motor: dict[int, ParsedFeedback] = {}
        self.latest_feedback_by_source: dict[str, dict[int, ParsedFeedback]] = {}
        self.manager.feedback_signal.connect(self._on_feedback)
        self.manager.status_signal.connect(self._on_status)
        self._on_status(self.manager.status_summary())

    def start_ros(self) -> object:
        """Compatibility shim kept for existing tests and widget code."""
        self.manager.start()
        return self.manager

    def stop_ros(self) -> None:
        """Namespace proxies do not own backend lifecycle."""
        return None

    def send_compact_command(
        self,
        motor_id: int,
        cmd_type: int,
        param1: int = 0,
        param2: int = 0,
        param3: int = 0,
    ) -> bool:
        return self.manager.send_compact_command(
            namespace=self.topic_ns,
            motor_id=motor_id,
            cmd_type=cmd_type,
            param1=param1,
            param2=param2,
            param3=param3,
        )

    def set_motor_velocity(self, motor_id: int, velocity: int | float) -> bool:
        timestamp_x100 = int((time.time() % 10000) * 100)
        return self.send_compact_command(
            motor_id,
            0x02,
            int(velocity),
            timestamp_x100,
            0,
        )

    def emergency_stop(self) -> bool:
        return self.manager.emergency_stop(namespace=self.topic_ns)

    def get_zero_target_motor_ids(self) -> list[int]:
        return motor_ids_for_namespace_or_joint_name(namespace=self.topic_ns)

    def set_zero_position(self) -> bool:
        success = True
        for motor_id in self.get_zero_target_motor_ids():
            if not self.send_compact_command(motor_id, 0x03):
                success = False
        return success

    def _on_feedback(self, source: str, namespace: str, parsed: ParsedFeedback) -> None:
        if namespace != self.topic_ns:
            return
        self.latest_feedback_by_motor[int(parsed.motor_id)] = parsed
        namespace_cache = self.latest_feedback_by_source.setdefault(source, {})
        namespace_cache[int(parsed.motor_id)] = parsed
        self.feedback_signal.emit(int(parsed.motor_id), int(parsed.raw_pulses))
        self.sourced_feedback_signal.emit(
            source,
            int(parsed.motor_id),
            int(parsed.raw_pulses),
            parsed,
        )

    def _on_status(self, summary: dict[str, object]) -> None:
        dispatch_mode = str(summary.get("dispatch_mode", "none"))
        is_connected = dispatch_mode != "none"
        if is_connected == self.connected:
            return
        self.connected = is_connected
        if self.connected:
            self.connected_signal.emit()
        else:
            self.disconnected_signal.emit()


class GuiBackendManager(QObject):
    """Own backend lifecycle, discovery, fan-out dispatch, and source-aware feedback."""

    log_signal = pyqtSignal(str, str)
    feedback_signal = pyqtSignal(str, str, "PyQt_PyObject")
    status_signal = pyqtSignal("PyQt_PyObject")

    def __init__(
        self,
        *,
        sim_backend: EmbeddedSimBackend | None = None,
        live_backend: Ros2TopicBackend | None = None,
        poll_interval_ms: int = 10,
    ) -> None:
        super().__init__()
        self.backends: dict[str, Ros2TopicBackend] = {}
        if sim_backend is not None:
            self.backends[BACKEND_SOURCE_SIM] = sim_backend
        if live_backend is not None:
            self.backends[BACKEND_SOURCE_LIVE] = live_backend
        for backend in self.backends.values():
            backend.feedback_callback = self._handle_feedback
            backend.log_callback = self._emit_log
        self.poll_interval_ms = max(int(poll_interval_ms), 1)
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._poll_backends)
        self._started = False
        self._feedback_cache: dict[str, dict[str, dict[int, ParsedFeedback]]] = {
            source: {} for source in BACKEND_SOURCES
        }
        self._status_summary: dict[str, object] | None = None

    def start(self) -> None:
        """Start all configured backends and begin periodic polling."""
        if self._started:
            return
        self._started = True
        for backend in self.backends.values():
            backend.start()
        self._timer.start(self.poll_interval_ms)
        self._refresh_status(force=True)
        if self.dispatch_mode() == "none":
            self._emit_log("当前无可用 backend，GUI 已进入只读/待机模式", "warning")

    def shutdown(self) -> None:
        """Stop all backends and clear the polling loop."""
        if self._timer.isActive():
            self._timer.stop()
        for backend in self.backends.values():
            backend.stop()
        self._started = False
        self._refresh_status(force=True)

    def create_namespace_proxy(self, namespace: str) -> NamespaceDispatchProxy:
        """Create one namespace-scoped widget proxy."""
        return NamespaceDispatchProxy(namespace, self)

    def backend_for_source(self, source: str) -> Ros2TopicBackend | None:
        """Return one backend object for tests and diagnostics."""
        return self.backends.get(source)

    def status_summary(self) -> dict[str, object]:
        """Return the latest status summary or compute one immediately."""
        if self._status_summary is None:
            self._status_summary = self._build_status_summary()
        return dict(self._status_summary)

    def dispatch_mode(self) -> str:
        """Return `none`, `sim`, `live`, or `both`."""
        summary = self.status_summary()
        return str(summary.get("dispatch_mode", "none"))

    def preferred_feedback_source(self) -> str | None:
        """Prefer live feedback when both sources are online, otherwise use any online source."""
        summary = self.status_summary()
        active_sources = list(summary.get("active_sources", []))
        if BACKEND_SOURCE_LIVE in active_sources:
            return BACKEND_SOURCE_LIVE
        if BACKEND_SOURCE_SIM in active_sources:
            return BACKEND_SOURCE_SIM
        return None

    def send_compact_command(
        self,
        *,
        namespace: str,
        motor_id: int,
        cmd_type: int,
        param1: int = 0,
        param2: int = 0,
        param3: int = 0,
    ) -> bool:
        """Fan out one compact command to every online backend."""
        online_backends = self._online_backends()
        if not online_backends:
            self._emit_log(
                f"当前无可用 backend，未发送 {namespace}/motor_command motor={motor_id} cmd=0x{cmd_type:02X}",
                "warning",
            )
            return False
        delivered: list[str] = []
        for backend in online_backends:
            if backend.send_compact_command(
                namespace=namespace,
                motor_id=motor_id,
                cmd_type=cmd_type,
                param1=param1,
                param2=param2,
                param3=param3,
            ):
                delivered.append(backend.source)
        if delivered:
            self._emit_log(
                f"{namespace} motor{motor_id} cmd=0x{cmd_type:02X} fan-out -> {', '.join(delivered)}",
                "send",
            )
        else:
            self._emit_log(
                f"{namespace} motor{motor_id} cmd=0x{cmd_type:02X} 发送失败",
                "error",
            )
        self._refresh_status()
        return bool(delivered)

    def emergency_stop(self, *, namespace: str | None = None) -> bool:
        """Broadcast emergency stop to one namespace or to every namespace."""
        online_backends = self._online_backends()
        if not online_backends:
            self._emit_log("当前无可用 backend，急停未发送", "warning")
            return False
        namespaces = [namespace] if namespace else list(GUI_ROS2_TOPIC_NAMESPACES)
        sent = False
        for namespace_key in namespaces:
            for motor_id in motor_ids_for_namespace_or_joint_name(namespace=namespace_key):
                if self.send_compact_command(
                    namespace=namespace_key,
                    motor_id=int(motor_id),
                    cmd_type=0x01,
                ):
                    sent = True
        if sent:
            fanout_scope = namespace if namespace is not None else "ALL"
            self._emit_log(
                f"急停广播完成: scope={fanout_scope}, dispatch={self.dispatch_mode()}",
                "warning",
            )
        return sent

    def step_backend(self, source: str, *, count: int = 1) -> None:
        """Advance one backend manually for deterministic tests."""
        backend = self.backends.get(source)
        if backend is None:
            return
        for _ in range(max(int(count), 1)):
            backend.step_once()
        self._refresh_status()

    def feedback_snapshot(self) -> dict[str, dict[str, dict[int, ParsedFeedback]]]:
        """Return a defensive copy of the aggregated feedback cache."""
        return {
            source: {
                namespace: dict(feedbacks)
                for namespace, feedbacks in namespace_map.items()
            }
            for source, namespace_map in self._feedback_cache.items()
        }

    def _handle_feedback(self, source: str, namespace: str, parsed: ParsedFeedback) -> None:
        source_cache = self._feedback_cache.setdefault(source, {})
        namespace_cache = source_cache.setdefault(namespace, {})
        namespace_cache[int(parsed.motor_id)] = parsed
        self.feedback_signal.emit(source, namespace, parsed)
        self._refresh_status()

    def _poll_backends(self) -> None:
        for backend in self.backends.values():
            backend.poll()
        self._refresh_status()

    def _online_backends(self) -> list[Ros2TopicBackend]:
        return [
            backend
            for source, backend in self.backends.items()
            if bool(self.status_summary().get("backends", {}).get(source, {}).get("online"))
        ]

    def _build_status_summary(self) -> dict[str, object]:
        backend_snapshots = {
            source: backend.snapshot_status().to_dict()
            for source, backend in self.backends.items()
        }
        active_sources = [
            source
            for source, snapshot in backend_snapshots.items()
            if bool(snapshot.get("online"))
        ]
        if not active_sources:
            dispatch_mode = "none"
        elif len(active_sources) == 2:
            dispatch_mode = "both"
        else:
            dispatch_mode = active_sources[0]
        return {
            "started": self._started,
            "dispatch_mode": dispatch_mode,
            "active_sources": active_sources,
            "backends": backend_snapshots,
        }

    def _refresh_status(self, *, force: bool = False) -> None:
        summary = self._build_status_summary()
        if not force and summary == self._status_summary:
            return
        self._status_summary = summary
        self.status_signal.emit(dict(summary))
        self._emit_status_transition(summary)

    def _emit_status_transition(self, summary: dict[str, object]) -> None:
        backends = dict(summary.get("backends", {}))
        sim_state = backends.get(BACKEND_SOURCE_SIM, {}).get("status", BACKEND_STATUS_OFFLINE)
        live_state = backends.get(BACKEND_SOURCE_LIVE, {}).get("status", BACKEND_STATUS_OFFLINE)
        dispatch_mode = summary.get("dispatch_mode", "none")
        self._emit_log(
            f"Backend 状态: SIM={sim_state}, LIVE={live_state}, dispatch={dispatch_mode}",
            "system",
        )

    def _emit_log(self, message: str, level: str) -> None:
        self.log_signal.emit(message, level)


def _build_loopback_runtime(broker: LoopbackRos2Broker, context: LoopbackRclpyContext) -> object:
    """Build one loopback runtime dependency bundle for the embedded sim path."""
    from .gui_ros2 import Ros2GuiRuntimeDependencies  # Local import avoids a circular dependency.

    return Ros2GuiRuntimeDependencies(
        rclpy_module=context,
        node_factory=broker.create_node,
    )


__all__ = [
    "BACKEND_SOURCE_LIVE",
    "BACKEND_SOURCE_SIM",
    "BACKEND_SOURCES",
    "BACKEND_STATUS_OFFLINE",
    "BACKEND_STATUS_ONLINE",
    "BackendStatusSnapshot",
    "EmbeddedSimBackend",
    "GuiBackendManager",
    "NamespaceDispatchProxy",
    "Ros2TopicBackend",
]

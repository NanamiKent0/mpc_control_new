"""Standalone ROS2-facing simulator backend owned by `mpc_control_new`."""

from __future__ import annotations

from dataclasses import dataclass
import json
import threading
import time

try:
    import rclpy
    from rclpy.node import Node
except Exception:  # pragma: no cover - dependency guard
    rclpy = None
    Node = None

try:
    from std_msgs.msg import Int32MultiArray, Int64MultiArray, String
except Exception:  # pragma: no cover - dependency guard
    Int32MultiArray = None
    Int64MultiArray = None
    String = None

from ..ros2_interfaces import build_gui_ros2_topic_bindings
from ..ros2_message_adapters import gui_ros2_feedback_payload_to_array
from ..sim_command_dispatcher import SimCommandDispatcher
from ..sim_observation_provider import SimObservationProvider
from .backend import SIM_BACKEND_VERSION, SimRuntimeBackend
from .types import SIM_MODULE_IDS, SimConfig, SimState


@dataclass(slots=True)
class SimRos2RuntimeDependencies:
    """Injectable ROS2 runtime hooks used by the standalone sim stack."""

    rclpy_module: object | None = rclpy
    node_factory: object | None = Node
    int32_message_cls: object | None = Int32MultiArray
    int64_message_cls: object | None = Int64MultiArray
    string_message_cls: object | None = String

    def ok(self) -> bool:
        module = self.rclpy_module
        if module is None:
            return False
        ok = getattr(module, "ok", None)
        if callable(ok):
            try:
                return bool(ok())
            except Exception:
                return False
        return False

    def init(self) -> None:
        module = self.rclpy_module
        if module is None:
            raise RuntimeError("rclpy_unavailable")
        init = getattr(module, "init", None)
        if callable(init):
            init(args=None)

    def shutdown(self) -> None:
        module = self.rclpy_module
        if module is None:
            return
        shutdown = getattr(module, "shutdown", None)
        if callable(shutdown):
            try:
                shutdown()
            except Exception:
                return

    def spin_once(self, node: object, timeout_sec: float) -> None:
        module = self.rclpy_module
        if module is None:
            return
        spin_once = getattr(module, "spin_once", None)
        if callable(spin_once):
            spin_once(node, timeout_sec=timeout_sec)

    def create_node(self, node_name: str) -> object:
        if self.node_factory is None:
            raise RuntimeError("ros2_node_factory_unavailable")
        return self.node_factory(node_name)

    def command_message_type(self) -> object:
        return object if self.int32_message_cls is None else self.int32_message_cls

    def feedback_message_type(self) -> object:
        return object if self.int64_message_cls is None else self.int64_message_cls

    def state_message_type(self) -> object:
        return object if self.string_message_cls is None else self.string_message_cls

    def make_feedback_message(self, data: list[int]) -> object:
        if self.int64_message_cls is None:
            return _ArrayMessage(data)
        message = self.int64_message_cls()
        message.data = list(data)
        return message

    def make_state_message(self, payload: str) -> object:
        if self.string_message_cls is None:
            return _StringMessage(payload)
        message = self.string_message_cls()
        message.data = str(payload)
        return message


class SimRos2BackendBridge:
    """Expose the internal sim backend through GUI-compatible ROS2 topics."""

    def __init__(
        self,
        *,
        node_handle: object,
        backend: SimRuntimeBackend | None = None,
        config: SimConfig | None = None,
        runtime: SimRos2RuntimeDependencies | None = None,
        namespaces: tuple[str, ...] = SIM_MODULE_IDS,
    ) -> None:
        self.node_handle = node_handle
        self.config = config or SimConfig()
        self.runtime = runtime or SimRos2RuntimeDependencies()
        self.backend = backend or SimRuntimeBackend(
            state=self.config.initial_state_copy(),
            dt=self.config.dt,
        )
        self.command_dispatcher = SimCommandDispatcher(backend=self.backend)
        self.observation_provider = SimObservationProvider(backend=self.backend)
        self.namespaces = tuple(namespaces)
        self._feedback_publishers: dict[str, object] = {}
        self._state_publisher: object | None = None
        self._subscriptions: list[object] = []
        self._started = False
        self._latest_state_payload: dict[str, object] | None = None
        self._latest_raw_commands: dict[str, list[int]] = {}

    @property
    def started(self) -> bool:
        return self._started

    def start(self) -> None:
        """Bind subscriptions/publishers and publish the initial state snapshot."""
        if self._started:
            return
        create_subscription = getattr(self.node_handle, "create_subscription", None)
        create_publisher = getattr(self.node_handle, "create_publisher", None)
        if not callable(create_subscription) or not callable(create_publisher):
            raise RuntimeError("sim_ros2_backend_node_handle_incompatible")
        self.command_dispatcher.start()
        self.observation_provider.start()
        for binding in build_gui_ros2_topic_bindings(self.namespaces):
            subscription = create_subscription(
                self.runtime.command_message_type(),
                binding.command_topic,
                lambda msg, namespace=binding.namespace: self._handle_command(namespace, msg),
                10,
            )
            publisher = create_publisher(
                self.runtime.feedback_message_type(),
                binding.feedback_topic,
                10,
            )
            self._subscriptions.append(subscription)
            self._feedback_publishers[binding.namespace] = publisher
        self._state_publisher = create_publisher(
            self.runtime.state_message_type(),
            self.config.state_topic,
            10,
        )
        self._started = True
        self.publish_snapshot()

    def stop(self) -> None:
        """Stop the backend bridge and clear local caches."""
        self.command_dispatcher.stop()
        self.observation_provider.stop()
        self._feedback_publishers = {}
        self._state_publisher = None
        self._subscriptions = []
        self._started = False

    def reset(self) -> SimState:
        """Reset the sim backend and publish the reset snapshot."""
        self.command_dispatcher.reset()
        self.observation_provider.reset()
        state = self.backend.reset()
        if self._started:
            self.publish_snapshot()
        return state

    def step(self) -> dict[str, object]:
        """Compatibility alias kept for the embedded GUI sim bridge."""
        return self.step_once()

    def step_once(self) -> dict[str, object]:
        """Advance the sim one tick using the currently latched GUI command."""
        self.backend.apply_command(self.backend.snapshot_last_command())
        self.publish_snapshot()
        return self.snapshot()

    def publish_feedback_snapshot(self) -> None:
        """Compatibility alias kept for the embedded GUI sim bridge."""
        self.publish_snapshot()

    def publish_snapshot(self) -> None:
        """Publish both compact motor feedback and the richer `sim/state` payload."""
        if not self._started:
            return
        for payload in self.observation_provider.gui_ros2_feedback_payloads():
            if payload.namespace not in self._feedback_publishers:
                continue
            publisher = self._feedback_publishers[payload.namespace]
            publish = getattr(publisher, "publish", None)
            if callable(publish):
                publish(
                    self.runtime.make_feedback_message(
                        gui_ros2_feedback_payload_to_array(payload)
                    )
                )
        state_payload = self.state_payload()
        self._latest_state_payload = state_payload
        if self._state_publisher is not None:
            publish = getattr(self._state_publisher, "publish", None)
            if callable(publish):
                publish(
                    self.runtime.make_state_message(
                        json.dumps(state_payload, ensure_ascii=True, sort_keys=True)
                    )
                )

    def state_payload(self) -> dict[str, object]:
        """Return the current `sim/state` payload as a JSON-friendly mapping."""
        return {
            "state": self.backend.snapshot_state().to_dict(),
            "command": self.backend.snapshot_last_command().to_dict(),
            "diagnostics": {
                "state_topic": self.config.state_topic,
                "namespaces": list(self.namespaces),
                "gui_ros2_compatible": True,
                "sim_backend_version": SIM_BACKEND_VERSION,
                "raw_command_namespaces": sorted(self._latest_raw_commands),
                **self.backend.diagnostics(),
            },
        }

    def latest_state_payload(self) -> dict[str, object] | None:
        """Return the last state payload published by the backend bridge."""
        return None if self._latest_state_payload is None else dict(self._latest_state_payload)

    def snapshot(self) -> dict[str, object]:
        """Return a compact diagnostics snapshot for tests and scripts."""
        return {
            "started": self._started,
            "namespaces": list(self.namespaces),
            "state_topic": self.config.state_topic,
            "provider": self.observation_provider.runtime_diagnostics(),
            "dispatcher": self.command_dispatcher.runtime_diagnostics(),
            "backend": self.backend.snapshot_state().to_dict(),
            "last_command": self.backend.snapshot_last_command().to_dict(),
            "latest_raw_commands": {
                namespace: list(payload)
                for namespace, payload in self._latest_raw_commands.items()
            },
        }

    def _handle_command(self, namespace: str, message: object) -> None:
        payload = list(getattr(message, "data", []) or [])
        self._latest_raw_commands[namespace] = [int(value) for value in payload]
        self.command_dispatcher.apply_gui_ros2_command(message, namespace=namespace)


class SimBackendRunner:
    """Background-thread wrapper that spins the standalone ROS2 simulator backend."""

    def __init__(
        self,
        *,
        config: SimConfig | None = None,
        backend: SimRuntimeBackend | None = None,
        runtime: SimRos2RuntimeDependencies | None = None,
        node_name: str | None = None,
        spin_timeout_sec: float = 0.02,
        spin_sleep_sec: float = 0.002,
        namespaces: tuple[str, ...] = SIM_MODULE_IDS,
    ) -> None:
        self.config = config or SimConfig()
        self.backend = backend or SimRuntimeBackend(
            state=self.config.initial_state_copy(),
            dt=self.config.dt,
        )
        self.runtime = runtime or SimRos2RuntimeDependencies()
        self.node_name = node_name or f"mpc_control_new_sim_backend_{int(time.time())}"
        self.spin_timeout_sec = max(float(spin_timeout_sec), 0.0)
        self.spin_sleep_sec = max(float(spin_sleep_sec), 0.0)
        self.namespaces = tuple(namespaces)
        self._bridge: SimRos2BackendBridge | None = None
        self._thread: threading.Thread | None = None
        self._state_lock = threading.RLock()
        self._started_event = threading.Event()
        self._stop_requested = threading.Event()
        self._start_exception: BaseException | None = None
        self._background_exception: BaseException | None = None
        self._running = False
        self._owns_context = False

    @property
    def is_running(self) -> bool:
        with self._state_lock:
            return bool(self._running and self._thread is not None and self._thread.is_alive())

    def start(self) -> None:
        """Start the background ROS2 backend thread."""
        with self._state_lock:
            if self.is_running:
                return
            if self.runtime.rclpy_module is None or self.runtime.node_factory is None:
                raise RuntimeError("ros2_runtime_unavailable")
            self._bridge = None
            self._thread = None
            self._started_event = threading.Event()
            self._stop_requested = threading.Event()
            self._start_exception = None
            self._background_exception = None
            self._running = False
            self._thread = threading.Thread(
                target=self._thread_main,
                name="SimBackendRunnerThread",
                daemon=True,
            )
            self._thread.start()
        if not self._started_event.wait(timeout=10.0):
            self.stop(timeout=1.0)
            raise TimeoutError("Timed out while starting SimBackendRunner")
        if self._start_exception is not None:
            exc = self._start_exception
            self.stop(timeout=1.0)
            raise RuntimeError(f"Failed to start SimBackendRunner: {exc}") from exc

    def stop(self, timeout: float = 5.0) -> None:
        """Stop the background thread and release owned ROS2 resources."""
        with self._state_lock:
            thread = self._thread
            self._stop_requested.set()
        if thread is None:
            return
        thread.join(timeout=max(float(timeout), 0.0))
        if thread.is_alive():
            raise TimeoutError("Timed out while stopping SimBackendRunner")
        with self._state_lock:
            terminal_exception = self._background_exception
            self._background_exception = None
            self._thread = None
        if terminal_exception is not None:
            raise RuntimeError(
                "SimBackendRunner background thread stopped because of an exception"
            ) from terminal_exception

    def snapshot(self) -> dict[str, object]:
        """Return the latest backend snapshot."""
        bridge = self._require_bridge()
        return bridge.snapshot()

    def latest_state_payload(self) -> dict[str, object]:
        """Return the latest `sim/state` payload."""
        bridge = self._require_bridge()
        payload = bridge.latest_state_payload()
        if payload is None:
            return bridge.state_payload()
        return payload

    def _thread_main(self) -> None:
        node_handle = None
        bridge = None
        thread_exception: BaseException | None = None
        try:
            if not self.runtime.ok():
                self.runtime.init()
                self._owns_context = True
            node_handle = self.runtime.create_node(self.node_name)
            bridge = SimRos2BackendBridge(
                node_handle=node_handle,
                backend=self.backend,
                config=self.config,
                runtime=self.runtime,
                namespaces=self.namespaces,
            )
            bridge.start()
            with self._state_lock:
                self._bridge = bridge
                self._running = True
            self._started_event.set()

            next_step_ts = time.monotonic()
            while not self._stop_requested.is_set():
                now_ts = time.monotonic()
                if now_ts >= next_step_ts:
                    bridge.step_once()
                    next_step_ts = now_ts + max(float(self.config.dt), 1e-4)
                timeout_sec = min(
                    self.spin_timeout_sec,
                    max(0.0, next_step_ts - time.monotonic()),
                )
                self.runtime.spin_once(node_handle, timeout_sec=timeout_sec)
                if self.spin_sleep_sec > 0.0:
                    self._stop_requested.wait(self.spin_sleep_sec)
        except BaseException as exc:
            thread_exception = exc
            with self._state_lock:
                if not self._started_event.is_set():
                    self._start_exception = exc
                    self._started_event.set()
                elif self._background_exception is None:
                    self._background_exception = exc
        finally:
            if bridge is not None:
                try:
                    bridge.stop()
                except BaseException as bridge_exc:
                    if thread_exception is None:
                        thread_exception = bridge_exc
                    with self._state_lock:
                        if self._background_exception is None:
                            self._background_exception = bridge_exc
            if node_handle is not None:
                destroy = getattr(node_handle, "destroy_node", None)
                if callable(destroy):
                    try:
                        destroy()
                    except BaseException as destroy_exc:
                        if thread_exception is None:
                            thread_exception = destroy_exc
                        with self._state_lock:
                            if self._background_exception is None:
                                self._background_exception = destroy_exc
            if self._owns_context and self.runtime.ok():
                try:
                    self.runtime.shutdown()
                except BaseException as shutdown_exc:
                    if thread_exception is None:
                        thread_exception = shutdown_exc
                    with self._state_lock:
                        if self._background_exception is None:
                            self._background_exception = shutdown_exc
            with self._state_lock:
                self._running = False
                self._bridge = None

    def _require_bridge(self) -> SimRos2BackendBridge:
        with self._state_lock:
            bridge = self._bridge
            background_exception = self._background_exception
        if bridge is not None:
            return bridge
        if background_exception is not None:
            raise RuntimeError(
                "SimBackendRunner is not running because the background thread failed"
            ) from background_exception
        raise RuntimeError("SimBackendRunner is not running. Call start() first.")


class _ArrayMessage:
    """Fallback array-style message used when std_msgs is unavailable."""

    def __init__(self, data: list[int]) -> None:
        self.data = list(data)


class _StringMessage:
    """Fallback string-style message used when std_msgs is unavailable."""

    def __init__(self, data: str) -> None:
        self.data = str(data)


__all__ = [
    "SimBackendRunner",
    "SimRos2BackendBridge",
    "SimRos2RuntimeDependencies",
]

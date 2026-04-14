"""Independent simulator visualizer that subscribes to `sim/state`."""

from __future__ import annotations

from dataclasses import dataclass
import json
import os
from pathlib import Path
import threading
import time
from typing import Any

os.environ.setdefault("MPLCONFIGDIR", "/tmp/mpc_control_new_mplconfig")

import matplotlib

if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
    matplotlib.use("Agg")

import numpy as np
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation

from ...control_core.kinematics import ChainSnapshot, compute_chain_snapshot
from ...control_core.models.module_state import make_joint_module_state, make_tip_module_state
from .ros2_backend import SimRos2RuntimeDependencies
from .types import SIM_JOINT_IDS, SIM_MODULE_IDS, SimState


_POINT_ORDER = (
    "base_origin",
    "joint5_start",
    "joint5_mid1",
    "joint5_mid2",
    "joint5_end",
    "joint4_start",
    "joint4_mid1",
    "joint4_mid2",
    "joint4_end",
    "joint3_start",
    "joint3_mid1",
    "joint3_mid2",
    "joint3_end",
    "joint2_start",
    "joint2_mid1",
    "joint2_mid2",
    "joint2_end",
    "joint1_start",
    "joint1_mid1",
    "joint1_mid2",
    "joint1_end",
    "tip_start",
    "tip_end",
)

_HIGHLIGHT_POINTS = (
    "base_origin",
    "joint5_start",
    "joint5_mid1",
    "joint5_mid2",
    "joint5_end",
    "joint4_start",
    "joint4_mid1",
    "joint4_mid2",
    "joint4_end",
    "joint3_start",
    "joint3_mid1",
    "joint3_mid2",
    "joint3_end",
    "joint2_start",
    "joint2_mid1",
    "joint2_mid2",
    "joint2_end",
    "joint1_start",
    "joint1_mid1",
    "joint1_mid2",
    "joint1_end",
    "tip_start",
    "tip_end",
)

_FRAME_NAMES = (
    "base_frame",
    "joint5_end_frame",
    "joint4_end_frame",
    "joint3_end_frame",
    "joint2_end_frame",
    "joint1_end_frame",
    "tip_end_frame",
)

_MODULE_LINE_COLORS = {
    "joint5": "#3D405B",
    "joint4": "#264653",
    "joint3": "#287271",
    "joint2": "#2A9D8F",
    "joint1": "#F4A261",
    "tip": "#E76F51",
}

_FRAME_AXIS_COLORS = {
    "x": "#D62828",
    "y": "#2A9D8F",
    "z": "#1D4ED8",
}


@dataclass(slots=True)
class VisualizerConfig:
    """Render configuration for the standalone sim visualizer."""

    fps: float = 30.0
    frame_axis_length_mm: float = 40.0
    xlim: tuple[float, float] = (-1350.0, 250.0)
    ylim: tuple[float, float] = (-300.0, 300.0)
    zlim: tuple[float, float] = (-300.0, 300.0)
    show_frames: bool = True
    view_mode: str = "follow_with_hysteresis"
    follow_margin_ratio: float = 0.12
    follow_cooldown_s: float = 0.5


class SimStateListener:
    """Background or externally-bound subscriber used by the visualizer."""

    def __init__(
        self,
        *,
        topic_name: str = "sim/state",
        runtime: SimRos2RuntimeDependencies | None = None,
        node_handle: object | None = None,
        spin_timeout_sec: float = 0.02,
        spin_sleep_sec: float = 0.005,
    ) -> None:
        self.topic_name = str(topic_name).strip().strip("/")
        self.runtime = runtime or SimRos2RuntimeDependencies()
        self.node_handle = node_handle
        self.spin_timeout_sec = max(float(spin_timeout_sec), 0.0)
        self.spin_sleep_sec = max(float(spin_sleep_sec), 0.0)
        self._latest_payload: dict[str, object] | None = None
        self._latest_wall_time_s: float | None = None
        self._data_lock = threading.RLock()
        self._thread: threading.Thread | None = None
        self._started_event = threading.Event()
        self._stop_requested = threading.Event()
        self._state_lock = threading.RLock()
        self._start_exception: BaseException | None = None
        self._background_exception: BaseException | None = None
        self._running = False
        self._owns_context = False
        self._owns_node = False

    @property
    def is_running(self) -> bool:
        with self._state_lock:
            if self.node_handle is not None and self._thread is None:
                return self._running
            return bool(self._running and self._thread is not None and self._thread.is_alive())

    def start(self) -> None:
        """Start listening to the state topic."""
        with self._state_lock:
            if self.is_running:
                return
        if self.node_handle is not None:
            self._bind_subscription(self.node_handle)
            with self._state_lock:
                self._running = True
            return
        with self._state_lock:
            if self.runtime.rclpy_module is None or self.runtime.node_factory is None:
                raise RuntimeError("ros2_runtime_unavailable")
            self._thread = None
            self._started_event = threading.Event()
            self._stop_requested = threading.Event()
            self._start_exception = None
            self._background_exception = None
            self._running = False
            self._thread = threading.Thread(
                target=self._thread_main,
                name="SimStateListenerThread",
                daemon=True,
            )
            self._thread.start()
        if not self._started_event.wait(timeout=10.0):
            self.stop(timeout=1.0)
            raise TimeoutError("Timed out while starting SimStateListener")
        if self._start_exception is not None:
            exc = self._start_exception
            self.stop(timeout=1.0)
            raise RuntimeError(f"Failed to start SimStateListener: {exc}") from exc

    def stop(self, timeout: float = 5.0) -> None:
        """Stop the listener and release owned resources."""
        with self._state_lock:
            thread = self._thread
            self._stop_requested.set()
            owns_node = self._owns_node
            node_handle = self.node_handle
        if thread is not None:
            thread.join(timeout=max(float(timeout), 0.0))
            if thread.is_alive():
                raise TimeoutError("Timed out while stopping SimStateListener")
            with self._state_lock:
                terminal_exception = self._background_exception
                self._background_exception = None
                self._thread = None
            if terminal_exception is not None:
                raise RuntimeError(
                    "SimStateListener background thread stopped because of an exception"
                ) from terminal_exception
            return
        if owns_node and node_handle is not None:
            destroy = getattr(node_handle, "destroy_node", None)
            if callable(destroy):
                destroy()
        with self._state_lock:
            self._running = False
            self.node_handle = None
            self._owns_node = False

    def get_latest_state(self) -> SimState | None:
        """Return the latest decoded simulator state snapshot."""
        with self._data_lock:
            payload = None if self._latest_payload is None else dict(self._latest_payload)
        if payload is None:
            return None
        return normalize_state_snapshot(payload)

    def get_latest_payload(self) -> dict[str, object] | None:
        """Return the raw latest `sim/state` payload."""
        with self._data_lock:
            return None if self._latest_payload is None else dict(self._latest_payload)

    def _bind_subscription(self, node_handle: object) -> None:
        create_subscription = getattr(node_handle, "create_subscription", None)
        if not callable(create_subscription):
            raise RuntimeError("sim_state_listener_node_handle_incompatible")
        create_subscription(
            self.runtime.state_message_type(),
            self.topic_name,
            self._handle_message,
            10,
        )

    def _handle_message(self, message: object) -> None:
        payload = payload_from_state_message(message)
        with self._data_lock:
            self._latest_payload = payload
            self._latest_wall_time_s = time.time()

    def _thread_main(self) -> None:
        node_handle = None
        thread_exception: BaseException | None = None
        try:
            if not self.runtime.ok():
                self.runtime.init()
                self._owns_context = True
            node_handle = self.runtime.create_node(
                f"mpc_control_new_sim_visualizer_listener_{int(time.time())}"
            )
            self.node_handle = node_handle
            self._owns_node = True
            self._bind_subscription(node_handle)
            with self._state_lock:
                self._running = True
            self._started_event.set()
            while not self._stop_requested.is_set():
                self.runtime.spin_once(node_handle, timeout_sec=self.spin_timeout_sec)
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
                self.node_handle = None
                self._owns_node = False


class SimVisualizer:
    """Matplotlib 3D viewer for the standalone simulator state stream."""

    def __init__(
        self,
        *,
        topic_name: str = "sim/state",
        config: VisualizerConfig | None = None,
        state_listener: SimStateListener | None = None,
    ) -> None:
        self.listener = state_listener or SimStateListener(topic_name=topic_name)
        self.geometry_module_order = SIM_MODULE_IDS
        self.render_module_order = tuple(reversed(SIM_JOINT_IDS)) + ("tip",)
        self.config = config or VisualizerConfig()
        self.config.xlim = self._normalize_limits(self.config.xlim)
        self.config.ylim = self._normalize_limits(self.config.ylim)
        self.config.zlim = self._normalize_limits(self.config.zlim)
        self.update_interval_ms = max(1, int(round(1000.0 / max(float(self.config.fps), 1.0))))
        self._view_mode = self._normalize_view_mode(self.config.view_mode)
        self._follow_margin_ratio = min(max(float(self.config.follow_margin_ratio), 0.0), 0.45)
        self._follow_cooldown_s = max(float(self.config.follow_cooldown_s), 0.0)
        self._current_limits = {
            "x": self.config.xlim,
            "y": self.config.ylim,
            "z": self.config.zlim,
        }
        self._view_sizes = {
            axis_name: float(limit_pair[1] - limit_pair[0])
            for axis_name, limit_pair in self._current_limits.items()
        }
        self._last_view_shift_time_s = float("-inf")
        self._fit_once_applied = False
        self._figure = None
        self._axes = None
        self._animation: FuncAnimation | None = None
        self._module_lines: dict[str, Any] = {}
        self._frame_axis_lines: dict[str, dict[str, Any]] = {}
        self._scatter = None
        self._status_text = None
        self._frame_status_text = None
        self._last_rendered_seq: int | None = None
        self._latest_render_payload: dict[str, object] | None = None

    def consume_state_snapshot(self, snapshot: object) -> dict[str, object]:
        """Normalize one state snapshot and convert it into renderable geometry."""
        state = normalize_state_snapshot(snapshot)
        payload = self._build_render_payload(state)
        self._latest_render_payload = payload
        return payload

    def latest_render_payload(self) -> dict[str, object] | None:
        """Return the latest render payload produced by the visualizer."""
        return None if self._latest_render_payload is None else dict(self._latest_render_payload)

    def show(self) -> None:
        """Open the interactive 3D visualizer window."""
        self.listener.start()
        self._create_figure()
        self._animation = FuncAnimation(
            self._figure,
            self._update_frame,
            interval=self.update_interval_ms,
            blit=False,
            cache_frame_data=False,
        )
        try:
            plt.show()
        finally:
            self.listener.stop()

    def run_headless(self, duration_s: float = 3.0, save_path: str | Path | None = None) -> None:
        """Poll the state topic without opening a window and optionally save one image."""
        self.listener.start()
        start = time.time()
        self._create_figure()
        try:
            while time.time() - start <= float(duration_s):
                self._update_frame(0)
                time.sleep(max(self.update_interval_ms / 1000.0, 0.05))
            if save_path is not None:
                output_path = Path(save_path)
                output_path.parent.mkdir(parents=True, exist_ok=True)
                self._figure.savefig(output_path)
        finally:
            self.listener.stop()
            plt.close(self._figure)

    def _create_figure(self) -> None:
        self._figure = plt.figure(figsize=(9, 7))
        self._axes = self._figure.add_subplot(111, projection="3d")
        self._axes.set_title("MPC Control New Sim Visualizer")
        self._axes.set_xlabel("X (mm)")
        self._axes.set_ylabel("Y (mm)")
        self._axes.set_zlabel("Z (mm)")
        self._apply_current_limits()
        self._axes.set_box_aspect(
            (
                self._view_sizes["x"],
                self._view_sizes["y"],
                self._view_sizes["z"],
            )
        )
        self._axes.grid(True, alpha=0.25)
        for module_id in self.render_module_order:
            self._module_lines[module_id], = self._axes.plot(
                [],
                [],
                [],
                color=_MODULE_LINE_COLORS[module_id],
                linewidth=3.0,
                label=module_id,
            )
        self._module_lines["interfaces"], = self._axes.plot(
            [],
            [],
            [],
            color="#E9C46A",
            linewidth=1.5,
            linestyle="--",
            label="interfaces",
        )
        self._scatter = self._axes.scatter([], [], [], color="#111111", s=28)
        self._status_text = self._axes.text2D(0.02, 0.95, "", transform=self._axes.transAxes)
        self._frame_status_text = self._axes.text2D(
            0.02,
            0.90,
            "frames: base / joint5_end / joint4_end / joint3_end / joint2_end / joint1_end / tip_end",
            transform=self._axes.transAxes,
        )
        if self.config.show_frames:
            for frame_name in _FRAME_NAMES:
                self._frame_axis_lines[frame_name] = {}
                for axis_name in ("x", "y", "z"):
                    artist, = self._axes.plot(
                        [],
                        [],
                        [],
                        color=_FRAME_AXIS_COLORS[axis_name],
                        linewidth=2.0 if frame_name == "base_frame" else 1.5,
                        alpha=0.95 if frame_name == "base_frame" else 0.85,
                    )
                    self._frame_axis_lines[frame_name][axis_name] = artist
        self._axes.legend(loc="upper right")

    def _update_frame(self, _frame_index: int) -> None:
        state = self.listener.get_latest_state()
        if state is None:
            if self._status_text is not None:
                self._status_text.set_text("waiting for sim/state ...")
            return
        if self._last_rendered_seq == int(state.seq):
            return
        self._last_rendered_seq = int(state.seq)
        payload = self.consume_state_snapshot(state)
        named_points = dict(payload["named_points"])
        named_frames = dict(payload["named_frames"])
        self._maybe_adjust_view(named_points)
        module_paths = dict(payload["module_paths"])
        for key, artist in self._module_lines.items():
            xs, ys, zs = module_paths[key]
            artist.set_data(xs, ys)
            artist.set_3d_properties(zs)
        highlight_xyz = self._points_to_xyz(named_points, _HIGHLIGHT_POINTS)
        self._scatter._offsets3d = highlight_xyz
        if self.config.show_frames:
            self._update_frame_artists(named_frames)
        if self._status_text is not None:
            self._status_text.set_text(str(payload["status_text"]))
        if self._frame_status_text is not None:
            self._frame_status_text.set_text(
                f"fps={self.config.fps:.0f}  "
                f"bend_axis=joint_end.z  "
                f"view={self._view_status_label()}  "
                f"x={self._format_limits(self._current_limits['x'])}  "
                f"y={self._format_limits(self._current_limits['y'])}  "
                f"z={self._format_limits(self._current_limits['z'])}"
            )

    def _build_render_payload(self, state: SimState) -> dict[str, object]:
        snapshot = compute_chain_snapshot(
            {
                "tip": make_tip_module_state(
                    growth_mm=float(state.g),
                    metadata={"sim_source": "sim_backend.visualizer"},
                ),
                "joint1": make_joint_module_state(
                    "joint1",
                    crawl_mm=float(state.c1),
                    bend_deg=float(state.theta1),
                    rotate_deg=float(state.psi1),
                    metadata={"sim_source": "sim_backend.visualizer"},
                ),
                "joint2": make_joint_module_state(
                    "joint2",
                    crawl_mm=float(state.c2),
                    bend_deg=float(state.theta2),
                    rotate_deg=float(state.psi2),
                    metadata={"sim_source": "sim_backend.visualizer"},
                ),
                "joint3": make_joint_module_state(
                    "joint3",
                    crawl_mm=float(state.c3),
                    bend_deg=float(state.theta3),
                    rotate_deg=float(state.psi3),
                    metadata={"sim_source": "sim_backend.visualizer"},
                ),
                "joint4": make_joint_module_state(
                    "joint4",
                    crawl_mm=float(state.c4),
                    bend_deg=float(state.theta4),
                    rotate_deg=float(state.psi4),
                    metadata={"sim_source": "sim_backend.visualizer"},
                ),
                "joint5": make_joint_module_state(
                    "joint5",
                    crawl_mm=float(state.c5),
                    bend_deg=float(state.theta5),
                    rotate_deg=float(state.psi5),
                    metadata={"sim_source": "sim_backend.visualizer"},
                ),
            },
            ordered_modules=self.geometry_module_order,
        )
        named_points = snapshot.copied_named_points()
        named_frames = snapshot.copied_named_frames()
        module_paths = {
            module_id: tuple(snapshot.module_paths.get(module_id, ([], [], [])))
            for module_id in self.render_module_order
        }
        module_paths["interfaces"] = self._points_to_xyz(named_points, self._interface_point_keys())
        bend_summary = "  ".join(
            f"b{joint_index}={self._bend_angle_from_snapshot(snapshot, module_id):.1f}"
            for joint_index, module_id in enumerate(SIM_JOINT_IDS, start=1)
        )
        status_text = (
            f"seq={state.seq}  "
            f"g={state.g:.1f}  "
            f"c1={state.c1:.1f}  c2={state.c2:.1f}  c3={state.c3:.1f}  c4={state.c4:.1f}  c5={state.c5:.1f}  "
            f"{bend_summary}"
        )
        return {
            "state": state.to_dict(),
            "named_points": named_points,
            "named_frames": named_frames,
            "module_paths": module_paths,
            "status_text": status_text,
        }

    @staticmethod
    def _interface_point_keys() -> tuple[str, ...]:
        return (
            "joint5_end",
            "joint4_start",
            "joint4_end",
            "joint3_start",
            "joint3_end",
            "joint2_start",
            "joint2_end",
            "joint1_start",
            "joint1_end",
            "tip_start",
        )

    def _points_to_xyz(
        self,
        named_points: dict[str, np.ndarray],
        keys: tuple[str, ...],
    ) -> tuple[list[float], list[float], list[float]]:
        xs: list[float] = []
        ys: list[float] = []
        zs: list[float] = []
        for key in keys:
            point = named_points.get(key)
            if point is None:
                continue
            xs.append(float(point[0]))
            ys.append(float(point[1]))
            zs.append(float(point[2]))
        return xs, ys, zs

    def _update_frame_artists(self, named_frames: dict[str, dict[str, np.ndarray]]) -> None:
        axis_length = float(self.config.frame_axis_length_mm)
        for frame_name, axis_lines in self._frame_axis_lines.items():
            frame = named_frames.get(frame_name)
            if frame is None:
                for artist in axis_lines.values():
                    artist.set_data([], [])
                    artist.set_3d_properties([])
                continue
            origin = np.asarray(frame["origin"], dtype=float).reshape(3)
            rotation_matrix = np.asarray(frame["R"], dtype=float).reshape(3, 3)
            axis_vectors = {
                "x": rotation_matrix[:, 0],
                "y": rotation_matrix[:, 1],
                "z": rotation_matrix[:, 2],
            }
            endpoints = {
                axis_name: origin + axis_length * axis_vector
                for axis_name, axis_vector in axis_vectors.items()
            }
            for axis_name, artist in axis_lines.items():
                end = endpoints[axis_name]
                artist.set_data(
                    [float(origin[0]), float(end[0])],
                    [float(origin[1]), float(end[1])],
                )
                artist.set_3d_properties([float(origin[2]), float(end[2])])

    def _maybe_adjust_view(self, named_points: dict[str, np.ndarray]) -> None:
        if not named_points:
            return
        bounds = self._compute_bounds(named_points)
        if bounds is None:
            return
        if self._view_mode == "fixed":
            return
        if self._view_mode == "fit_once":
            if not self._fit_once_applied:
                self._recenter_view_on_bounds(bounds)
                self._fit_once_applied = True
            return
        if self._view_mode != "follow_with_hysteresis":
            return
        deltas = self._compute_follow_shifts(bounds)
        if not any(abs(delta) > 1e-6 for delta in deltas):
            return
        now_s = time.monotonic()
        if now_s - self._last_view_shift_time_s < self._follow_cooldown_s:
            return
        self._translate_view(deltas)
        self._last_view_shift_time_s = now_s

    def _compute_bounds(
        self,
        named_points: dict[str, np.ndarray],
    ) -> tuple[np.ndarray, np.ndarray] | None:
        points: list[np.ndarray] = []
        for key in _POINT_ORDER:
            point = named_points.get(key)
            if point is None:
                continue
            points.append(np.asarray(point, dtype=float).reshape(3))
        if not points:
            return None
        stacked = np.vstack(points)
        return stacked.min(axis=0), stacked.max(axis=0)

    def _compute_follow_shifts(self, bounds: tuple[np.ndarray, np.ndarray]) -> tuple[float, float, float]:
        bbox_min, bbox_max = bounds
        shifts: list[float] = []
        for axis_index, axis_name in enumerate(("x", "y", "z")):
            low, high = self._current_limits[axis_name]
            width = self._view_sizes[axis_name]
            margin = self._follow_margin_ratio * width
            safe_low = low + margin
            safe_high = high - margin
            axis_min = float(bbox_min[axis_index])
            axis_max = float(bbox_max[axis_index])
            if axis_min >= safe_low and axis_max <= safe_high:
                shifts.append(0.0)
                continue
            lower_required = axis_max - high + margin
            upper_allowed = axis_min - low - margin
            if lower_required <= upper_allowed:
                if 0.0 < lower_required:
                    shifts.append(float(lower_required))
                elif 0.0 > upper_allowed:
                    shifts.append(float(upper_allowed))
                else:
                    shifts.append(0.0)
                continue
            current_center = 0.5 * (low + high)
            target_center = 0.5 * (axis_min + axis_max)
            shifts.append(float(target_center - current_center))
        return float(shifts[0]), float(shifts[1]), float(shifts[2])

    def _recenter_view_on_bounds(self, bounds: tuple[np.ndarray, np.ndarray]) -> None:
        bbox_min, bbox_max = bounds
        deltas = []
        for axis_index, axis_name in enumerate(("x", "y", "z")):
            low, high = self._current_limits[axis_name]
            current_center = 0.5 * (low + high)
            target_center = 0.5 * (float(bbox_min[axis_index]) + float(bbox_max[axis_index]))
            deltas.append(float(target_center - current_center))
        self._translate_view((float(deltas[0]), float(deltas[1]), float(deltas[2])))

    def _translate_view(self, deltas: tuple[float, float, float]) -> None:
        for axis_name, delta in zip(("x", "y", "z"), deltas):
            low, high = self._current_limits[axis_name]
            self._current_limits[axis_name] = (float(low + delta), float(high + delta))
        self._apply_current_limits()

    def _apply_current_limits(self) -> None:
        if self._axes is None:
            return
        self._axes.set_xlim(*self._current_limits["x"])
        self._axes.set_ylim(*self._current_limits["y"])
        self._axes.set_zlim(*self._current_limits["z"])

    def _view_status_label(self) -> str:
        if self._view_mode == "follow_with_hysteresis":
            return "follow_hys"
        return self._view_mode

    def _format_limits(self, limits: tuple[float, float]) -> str:
        return f"({limits[0]:.0f},{limits[1]:.0f})"

    @staticmethod
    def _normalize_limits(limits: tuple[float, float]) -> tuple[float, float]:
        low = float(limits[0])
        high = float(limits[1])
        if high < low:
            return high, low
        return low, high

    @staticmethod
    def _normalize_view_mode(view_mode: str) -> str:
        normalized = str(view_mode).strip().lower()
        if normalized in {"fixed", "follow_with_hysteresis", "fit_once"}:
            return normalized
        return "follow_with_hysteresis"

    @staticmethod
    def _bend_angle_from_snapshot(snapshot: ChainSnapshot, joint_name: str) -> float:
        pose = snapshot.module_poses.get(joint_name)
        if pose is None:
            return 0.0
        return float(pose.metadata.get("bend_angle_deg", 0.0))


def normalize_state_snapshot(snapshot: object) -> SimState:
    """Normalize one state snapshot or whole `sim/state` payload into `SimState`."""
    if isinstance(snapshot, SimState):
        return snapshot.copy()
    if isinstance(snapshot, str):
        return normalize_state_snapshot(json.loads(snapshot))
    if isinstance(snapshot, (bytes, bytearray)):
        return normalize_state_snapshot(snapshot.decode("utf-8"))
    if isinstance(snapshot, dict):
        payload = dict(snapshot)
    else:
        payload = payload_from_state_message(snapshot)
    state_payload = payload.get("state", payload)
    return SimState.from_dict(dict(state_payload))


def payload_from_state_message(message: object) -> dict[str, object]:
    """Decode one ROS2-style state message into a mapping."""
    if isinstance(message, dict):
        return dict(message)
    if isinstance(message, str):
        return dict(json.loads(message))
    data = getattr(message, "data", message)
    if isinstance(data, str):
        return dict(json.loads(data))
    if isinstance(data, bytes):
        return dict(json.loads(data.decode("utf-8")))
    if isinstance(data, dict):
        return dict(data)
    raise TypeError(f"unsupported_sim_state_message:{type(message)!r}")


__all__ = [
    "SimStateListener",
    "SimVisualizer",
    "VisualizerConfig",
    "normalize_state_snapshot",
    "payload_from_state_message",
]

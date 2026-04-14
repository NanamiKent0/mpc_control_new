"""Simulation-backed observation provider for the self-contained runtime path."""

from __future__ import annotations

from ..control_core.kinematics import compute_chain_snapshot
from ..control_core.models.module_state import make_joint_module_state, make_tip_module_state
from .common.encoder_protocol import physical_to_counts
from .observation_types import ModuleObservation, PairObservation, RuntimeObservationFrame
from .ros2_interfaces import GUI_ROS2_FEEDBACK_TOPICS, GUI_ROS2_TOPIC_NAMESPACES, GuiRos2MotorFeedbackPayload
from .sim_backend.backend import SIM_BACKEND_VERSION, SimRuntimeBackend
from .sim_backend.types import (
    SIM_ADJACENT_PAIRS,
    SIM_JOINT_IDS,
    SIM_MODULE_IDS,
    SimCommand,
    SimState,
    module_bend_attr,
    module_orientation_attr,
    module_position_attr,
    pair_coupling_attr,
    pair_distance_attr,
    pair_key,
    pair_orientation_attr,
)


class SimObservationProvider:
    """Expose self-contained sim-backend state as runtime-facing observation frames."""

    provider_kind = "sim"

    def __init__(
        self,
        backend: SimRuntimeBackend | None = None,
        *,
        source_name: str = "sim_observation_provider",
        topology_hint: dict[str, object] | None = None,
    ) -> None:
        self.backend = backend or SimRuntimeBackend()
        self.source_name = source_name
        self.topology_hint = dict(topology_hint or {})
        self._started = False
        self._latest_frame: RuntimeObservationFrame | None = None
        self.last_reason = "sim_provider_not_started"

    def start(self) -> None:
        """Mark the provider as ready for frame reads."""
        self._started = True
        self.last_reason = "sim_provider_started"

    def stop(self) -> None:
        """Mark the provider as stopped."""
        self._started = False
        self.last_reason = "sim_provider_stopped"

    def reset(self) -> None:
        """Reset the shared sim backend and clear the cached frame."""
        self.backend.reset()
        self._latest_frame = None
        self.last_reason = "sim_provider_reset"

    def warmup(self) -> RuntimeObservationFrame | None:
        """Fetch one frame immediately after start-up."""
        return self.get_latest_frame()

    def is_ready(self) -> bool:
        """Return whether the provider is ready to serve frames."""
        return self._started

    def runtime_diagnostics(self) -> dict[str, object]:
        """Return provider diagnostics for runtime-session reporting."""
        return {
            "provider_kind": self.provider_kind,
            "provider": self.source_name,
            "source_name": self.source_name,
            "started": self._started,
            "frame_cache_ready": self._latest_frame is not None,
            "source_topics": list(GUI_ROS2_FEEDBACK_TOPICS),
            "topic_namespaces": list(GUI_ROS2_TOPIC_NAMESPACES),
            "frame_origin": "sim",
            "ros2_gui_compatible": True,
            "gui_ros2_compatible": True,
            "reason": self.last_reason,
            "backend_state": self.backend.snapshot_state().to_dict(),
            "last_command": self.backend.snapshot_last_command().to_dict(),
            **self.backend.diagnostics(),
        }

    def get_latest_frame(self) -> RuntimeObservationFrame | None:
        """Read the latest backend state and convert it into a runtime frame."""
        if not self._started:
            self.last_reason = "sim_provider_not_started"
            return None

        state = self.backend.snapshot_state()
        command = self.backend.snapshot_last_command()

        module_states = _module_states_from_sim_state(state)
        snapshot = compute_chain_snapshot(
            module_states,
            ordered_modules=SIM_MODULE_IDS,
        )
        timestamp_ns = int(round(float(state.sim_time_s) * 1_000_000_000.0))

        tip_extension_mm = float(state.g)
        tip_heading_deg = _resolved_tip_heading_deg(state)

        pairs = _pair_observations_from_state(
            state,
            snapshot=snapshot,
            source_name=self.source_name,
        )
        modules = _module_observations_from_state(
            state,
            command,
            pair_observations=pairs,
            source_name=self.source_name,
            tip_heading_deg=tip_heading_deg,
        )

        resolved_topology_hint = self.backend.build_topology_hint()
        resolved_topology_hint.update(self.topology_hint)

        self._latest_frame = RuntimeObservationFrame(
            timestamp_ns=timestamp_ns,
            module_observations=modules,
            pair_observations=pairs,
            topology_hint=resolved_topology_hint,
            metadata={
                "source_name": self.source_name,
                "provider_source": "self_contained_sim_backend",
                "frame_origin": "sim",
                "sim_backend_version": SIM_BACKEND_VERSION,
                "sim_time_s": float(state.sim_time_s),
                "sim_seq": int(state.seq),
                "ros2_gui_compatible": True,
                "topic_namespaces": list(GUI_ROS2_TOPIC_NAMESPACES),

                # 新增：给 runtime_session / GUI 直接读取
                "tip_extension_mm": tip_extension_mm,
                "current_tip_growth_mm": tip_extension_mm,
                "tip_heading_deg": tip_heading_deg,
                "current_tip_heading_deg": tip_heading_deg,
            },
        )
        self.last_reason = "sim_frame_served"
        return self._latest_frame

    def snapshot_backend_state(self) -> dict[str, object]:
        """Return the latest backend snapshot for GUI or test consumers."""
        snapshot = self.backend.visualization_snapshot()
        state = self.backend.snapshot_state()

        tip_extension_mm = float(state.g)
        tip_heading_deg = _resolved_tip_heading_deg(state)

        snapshot["tip_extension_mm"] = tip_extension_mm
        snapshot["current_tip_growth_mm"] = tip_extension_mm
        snapshot["tip_heading_deg"] = tip_heading_deg
        snapshot["current_tip_heading_deg"] = tip_heading_deg

        snapshot["gui_feedback_payloads"] = [payload.data for payload in self.gui_ros2_feedback_payloads()]
        return snapshot

    def gui_ros2_feedback_payloads(self) -> list[GuiRos2MotorFeedbackPayload]:
        """Render the current sim state as GUI-compatible `motor_feedback` payloads."""
        state = self.backend.snapshot_state()
        feedbacks: list[GuiRos2MotorFeedbackPayload] = [
            _feedback_payload("tip", 4, state.g, state.seq, state.sim_time_s),
        ]
        for module_id in SIM_JOINT_IDS:
            orientation_attr = module_orientation_attr(module_id)
            bend_attr = module_bend_attr(module_id)
            assert orientation_attr is not None
            assert bend_attr is not None
            feedbacks.extend(
                [
                    _feedback_payload(
                        module_id,
                        1,
                        float(getattr(state, module_position_attr(module_id))),
                        state.seq,
                        state.sim_time_s,
                    ),
                    _feedback_payload(
                        module_id,
                        2,
                        float(getattr(state, orientation_attr)),
                        state.seq,
                        state.sim_time_s,
                    ),
                    _feedback_payload(
                        module_id,
                        3,
                        float(getattr(state, bend_attr)),
                        state.seq,
                        state.sim_time_s,
                    ),
                ]
            )
        return feedbacks

def _module_states_from_sim_state(state: SimState) -> dict[str, object]:
    """Build `ModuleState` inputs used by the unified kinematics layer."""
    module_states = {
        "tip": make_tip_module_state(
            growth_mm=float(state.g),
            metadata={"sim_source": "sim_observation_provider"},
        )
    }
    for module_id in SIM_JOINT_IDS:
        bend_attr = module_bend_attr(module_id)
        orientation_attr = module_orientation_attr(module_id)
        assert bend_attr is not None
        assert orientation_attr is not None
        module_states[module_id] = make_joint_module_state(
            module_id,
            crawl_mm=float(getattr(state, module_position_attr(module_id))),
            bend_deg=float(getattr(state, bend_attr)),
            rotate_deg=float(getattr(state, orientation_attr)),
            metadata={"sim_source": "sim_observation_provider"},
        )
    return module_states

def _resolved_tip_heading_deg(state: SimState) -> float:
    """
    Return one simplified planar tip heading proxy.

    Current sim convention:
        tip heading ~= joint1.rotate_deg + joint1.bend_deg

    This is not a full 3D heading representation. It is only the current
    planar heading proxy used by the sim/runtime GUI path.
    """
    joint1_orientation_attr = module_orientation_attr("joint1")
    joint1_bend_attr = module_bend_attr("joint1")
    assert joint1_orientation_attr is not None
    assert joint1_bend_attr is not None

    rotate_deg = float(getattr(state, joint1_orientation_attr))
    bend_deg = float(getattr(state, joint1_bend_attr))
    return rotate_deg + bend_deg

def _module_observations_from_state(
    state: SimState,
    command: SimCommand,
    *,
    pair_observations: dict[str, PairObservation],
    source_name: str,
    tip_heading_deg: float,
) -> dict[str, ModuleObservation]:
    """Build module observations for every simulated namespace."""
    modules: dict[str, ModuleObservation] = {
        "tip": ModuleObservation(
            module_id="tip",
            module_type="tip",
            dofs={
                "growth_mm": float(state.g),
                "heading_deg": float(tip_heading_deg),
            },
            velocities={"growth_mm_s": float(command.tip_growth_mm_s)},
            attach_state={"joint1": _pair_coupled(pair_observations, "joint1", "tip")},
            diagnostics={
                **_module_diagnostics(state.seq),
                "tip_extension_mm": float(state.g),
                "tip_heading_deg": float(tip_heading_deg),
            },
            source_name=source_name,
        )
    }
    for index, module_id in enumerate(SIM_JOINT_IDS, start=1):
        bend_attr = module_bend_attr(module_id)
        orientation_attr = module_orientation_attr(module_id)
        assert bend_attr is not None
        assert orientation_attr is not None
        attach_state: dict[str, bool | None] = {}
        distal_neighbor = "tip" if index == 1 else f"joint{index - 1}"
        attach_state[distal_neighbor] = _pair_coupled(pair_observations, module_id, distal_neighbor)
        if index < len(SIM_JOINT_IDS):
            proximal_neighbor = f"joint{index + 1}"
            attach_state[proximal_neighbor] = _pair_coupled(pair_observations, proximal_neighbor, module_id)
        modules[module_id] = ModuleObservation(
            module_id=module_id,
            module_type="joint",
            dofs={
                "crawl_mm": float(getattr(state, module_position_attr(module_id))),
                "bend_deg": float(getattr(state, bend_attr)),
                "rotate_deg": float(getattr(state, orientation_attr)),
            },
            velocities={
                "crawl_mm_s": float(getattr(command, f"{module_id}_crawl_mm_s")),
                "bend_deg_s": float(getattr(command, f"{module_id}_bend_deg_s")),
                "rotate_deg_s": float(getattr(command, f"{module_id}_rotate_deg_s")),
            },
            attach_state=attach_state,
            diagnostics=_module_diagnostics(state.seq),
            source_name=source_name,
        )
    return modules


def _pair_observations_from_state(
    state: SimState,
    *,
    snapshot: object,
    source_name: str,
) -> dict[str, PairObservation]:
    """Build pair observations for every adjacent pair in the sim chain."""
    pairs: dict[str, PairObservation] = {}
    for active_module, passive_module in SIM_ADJACENT_PAIRS:
        pair_geometry = snapshot.get_pair_geometry(active_module, passive_module)
        if pair_geometry is None:
            continue
        distance_mm = _resolved_pair_distance(
            state,
            active_module,
            passive_module,
            fallback=float(pair_geometry.distance_mm),
        )
        orientation_error_deg = _resolved_pair_orientation(
            state,
            active_module,
            passive_module,
            fallback=float(pair_geometry.orientation_error_deg),
        )
        coupled = _resolved_pair_coupled(
            state,
            active_module,
            passive_module,
            distance_mm=distance_mm,
            orientation_error_deg=orientation_error_deg,
        )
        pairs[pair_key(active_module, passive_module)] = PairObservation(
            active_module=active_module,
            passive_module=passive_module,
            relation_type="tip_joint" if passive_module == "tip" else "joint_joint",
            distance_mm=distance_mm,
            orientation_error_deg=orientation_error_deg,
            coupled=coupled,
            observation_valid=True,
            diagnostics={
                "sim_seq": state.seq,
                "provider_source": "self_contained_sim_backend",
                "gui_ros2_compatible": True,
                "geometry_source": "chain_snapshot",
            },
            source_name=source_name,
        )
    return pairs


def _module_diagnostics(sim_seq: int) -> dict[str, object]:
    return {
        "sim_seq": sim_seq,
        "provider_source": "self_contained_sim_backend",
        "gui_ros2_compatible": True,
    }


def _pair_coupled(
    pair_observations: dict[str, PairObservation],
    active_module: str,
    passive_module: str,
) -> bool | None:
    """Return one resolved pair coupling flag from the current pair observations."""
    observation = pair_observations.get(pair_key(active_module, passive_module))
    if observation is None:
        return None
    return observation.coupled


def _resolved_pair_distance(
    state: SimState,
    active_module: str,
    passive_module: str,
    *,
    fallback: float,
) -> float:
    """Return one pair distance reading with a deterministic fallback."""
    value = getattr(state, pair_distance_attr(active_module, passive_module))
    if value is None:
        return float(fallback)
    return float(value)


def _resolved_pair_orientation(
    state: SimState,
    active_module: str,
    passive_module: str,
    *,
    fallback: float,
) -> float:
    """Return one pair orientation reading with a deterministic fallback."""
    value = getattr(state, pair_orientation_attr(active_module, passive_module))
    if value is None:
        return float(fallback)
    return float(value)


def _resolved_pair_coupled(
    state: SimState,
    active_module: str,
    passive_module: str,
    *,
    distance_mm: float,
    orientation_error_deg: float,
) -> bool:
    """Return one pair coupling flag while respecting explicit sim-state overrides."""
    distance_override = getattr(state, pair_distance_attr(active_module, passive_module))
    orientation_override = getattr(state, pair_orientation_attr(active_module, passive_module))
    stored_value = getattr(state, pair_coupling_attr(active_module, passive_module))
    if distance_override is not None or orientation_override is not None:
        return bool(stored_value)
    if active_module == "joint1" and passive_module == "tip":
        return bool(float(distance_mm) <= 1.0)
    return bool(float(distance_mm) <= 1.0 and abs(float(orientation_error_deg)) <= 2.0)


def _feedback_payload(
    namespace: str,
    motor_id: int,
    physical_value: float,
    seq: int,
    sim_time_s: float,
) -> GuiRos2MotorFeedbackPayload:
    """Convert one sim scalar into a GUI-compatible feedback payload."""
    return GuiRos2MotorFeedbackPayload(
        namespace=namespace,
        motor_id=motor_id,
        raw_pulses=int(round(physical_to_counts(motor_id, physical_value))),
        device_time_ms=int(round(float(sim_time_s) * 1000.0)),
        seq=int(seq),
        status_flags=1,
        source_id=0,
        is_extended=True,
        feedback_topic=f"{namespace}/motor_feedback",
        diagnostics={
            "frame_origin": "sim",
            "simulated_namespace": True,
            "gui_ros2_compatible": True,
        },
    )


__all__ = ["SimObservationProvider"]

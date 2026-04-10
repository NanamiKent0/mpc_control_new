"""Self-contained lightweight simulator backend for runtime smoke coverage."""

from __future__ import annotations

from dataclasses import dataclass, field

from ...control_core.models.runtime_bridge_types import SchedulerDispatchEnvelope
from .types import SIM_MODULE_IDS, SimCommand, SimState

SIM_BACKEND_VERSION = "phase8.self_contained.v1"


@dataclass(slots=True)
class SimRuntimeBackend:
    """Minimal in-process sim backend owned entirely by `mpc_control_new`."""

    state: SimState
    dt: float = 0.1
    last_command: SimCommand = field(default_factory=SimCommand)
    last_envelope: SchedulerDispatchEnvelope | None = None
    backend_version: str = SIM_BACKEND_VERSION

    def __init__(self, state: SimState | None = None, *, dt: float = 0.1) -> None:
        self.state = (state or SimState()).copy()
        self.dt = float(dt)
        self.last_command = SimCommand()
        self.last_envelope = None
        self.backend_version = SIM_BACKEND_VERSION

    def snapshot_state(self) -> SimState:
        """Return a defensive copy of the latest backend state."""
        return self.state.copy()

    def snapshot_last_command(self) -> SimCommand:
        """Return a defensive copy of the most recent command."""
        return self.last_command.copy()

    def snapshot_last_envelope(self) -> SchedulerDispatchEnvelope | None:
        """Return the last accepted scheduler envelope."""
        return self.last_envelope

    def diagnostics(self) -> dict[str, object]:
        """Return stable backend diagnostics for providers, dispatchers, and tests."""
        return {
            "provider_source": "self_contained_sim_backend",
            "dispatch_target": "sim",
            "frame_origin": "sim",
            "sim_backend_version": self.backend_version,
            "sim_dt": float(self.dt),
            "sim_seq": float(self.state.seq),
            "sim_time_s": float(self.state.sim_time_s),
        }

    def build_topology_hint(self) -> dict[str, object]:
        """Build a small topology hint sufficient for runtime session smoke coverage."""
        return {
            "ordered_modules": list(SIM_MODULE_IDS),
            "active_frontier": ["joint1", "tip"],
            "support_modules": ["joint1", "joint2"],
            "grounded_modules": ["joint2"],
            "coupled_pairs": {
                "joint1->tip": bool(self.state.tip_joint1_coupled),
                "joint2->joint1": bool(self.state.joint1_joint2_coupled),
            },
        }

    def apply_command(
        self,
        command: SimCommand,
        *,
        envelope: SchedulerDispatchEnvelope | None = None,
    ) -> SimState:
        """Advance the lightweight sim backend one step using the latest command."""
        self.last_command = command.copy()
        self.last_envelope = envelope
        next_state = self.state.copy()
        next_state.seq = int(next_state.seq) + 1
        next_state.sim_time_s = float(next_state.sim_time_s) + float(self.dt)
        next_state.g += float(command.tip_growth_mm_s) * float(self.dt)
        next_state.c1 += float(command.joint1_crawl_mm_s) * float(self.dt)
        next_state.c2 += float(command.joint2_crawl_mm_s) * float(self.dt)
        next_state.theta1 += float(command.joint1_bend_deg_s) * float(self.dt)
        next_state.theta2 += float(command.joint2_bend_deg_s) * float(self.dt)
        next_state.psi1 += float(command.joint1_rotate_deg_s) * float(self.dt)
        next_state.psi2 += float(command.joint2_rotate_deg_s) * float(self.dt)
        self._advance_pair_geometry(next_state, envelope=envelope, command=command)
        self.state = next_state
        return next_state.copy()

    def stop_all(self) -> None:
        """Zero all command outputs without advancing the simulation."""
        self.last_command = SimCommand()

    def _advance_pair_geometry(
        self,
        state: SimState,
        *,
        envelope: SchedulerDispatchEnvelope | None,
        command: SimCommand,
    ) -> None:
        scheduled = None if envelope is None else envelope.scheduled_command
        if scheduled is None:
            state.tip_joint1_distance_mm = _pair_distance_after_drive(
                current=state.tip_joint1_distance_mm,
                drive_mm_s=max(float(command.tip_growth_mm_s), 0.0) + max(float(command.joint1_crawl_mm_s), 0.0),
                dt=self.dt,
            )
            state.joint1_joint2_distance_mm = _pair_distance_after_drive(
                current=state.joint1_joint2_distance_mm,
                drive_mm_s=max(float(command.joint1_crawl_mm_s), 0.0) + max(float(command.joint2_crawl_mm_s), 0.0),
                dt=self.dt,
            )
            return

        pair_name = f"{scheduled.active_module}->{scheduled.passive_module}"
        if pair_name == "joint1->tip":
            state.tip_joint1_distance_mm = _pair_distance_after_drive(
                current=state.tip_joint1_distance_mm,
                drive_mm_s=_module_linear_drive(command, scheduled.active_module),
                dt=self.dt,
            )
            state.tip_joint1_orientation_error_deg = _move_toward_zero(
                state.tip_joint1_orientation_error_deg,
                drive_deg_s=_module_rotate_drive(command, scheduled.active_module),
                dt=self.dt,
            )
            self._update_coupled_state_for_tip_joint1(state, skill_key=scheduled.skill_key)
            return
        if pair_name == "joint2->joint1":
            state.joint1_joint2_distance_mm = _pair_distance_after_drive(
                current=state.joint1_joint2_distance_mm,
                drive_mm_s=_module_linear_drive(command, scheduled.active_module),
                dt=self.dt,
            )
            state.joint1_joint2_orientation_error_deg = _move_toward_zero(
                state.joint1_joint2_orientation_error_deg,
                drive_deg_s=_module_rotate_drive(command, scheduled.active_module),
                dt=self.dt,
            )
            self._update_coupled_state_for_joint_joint(state, skill_key=scheduled.skill_key)

    @staticmethod
    def _update_coupled_state_for_tip_joint1(state: SimState, *, skill_key: str) -> None:
        """Update the tip-joint coupling flag from the current geometry approximation."""
        distance_ready = (state.tip_joint1_distance_mm or 0.0) <= 1.0
        orientation_ready = abs(float(state.tip_joint1_orientation_error_deg or 0.0)) <= 2.0
        if skill_key == "coarse_approach":
            state.tip_joint1_coupled = False
            return
        state.tip_joint1_coupled = bool(distance_ready and orientation_ready)

    @staticmethod
    def _update_coupled_state_for_joint_joint(state: SimState, *, skill_key: str) -> None:
        """Update the joint-joint coupling flag from the current geometry approximation."""
        distance_ready = (state.joint1_joint2_distance_mm or 0.0) <= 1.0
        orientation_ready = abs(float(state.joint1_joint2_orientation_error_deg or 0.0)) <= 2.0
        if skill_key == "coarse_approach":
            state.joint1_joint2_coupled = False
            return
        state.joint1_joint2_coupled = bool(distance_ready and orientation_ready)


def _module_linear_drive(command: SimCommand, module_id: str) -> float:
    """Return the positive linear drive applied by one module."""
    if module_id == "tip":
        return max(float(command.tip_growth_mm_s), 0.0)
    return max(float(getattr(command, f"{module_id}_crawl_mm_s", 0.0)), 0.0)


def _module_rotate_drive(command: SimCommand, module_id: str) -> float:
    """Return the absolute rotational drive applied by one module."""
    return abs(float(getattr(command, f"{module_id}_rotate_deg_s", 0.0)))


def _pair_distance_after_drive(current: float | None, *, drive_mm_s: float, dt: float) -> float:
    """Reduce a non-negative distance using a simple command-driven heuristic."""
    base_distance = 0.0 if current is None else float(current)
    return max(0.0, base_distance - max(float(drive_mm_s), 0.0) * float(dt))


def _move_toward_zero(current: float | None, *, drive_deg_s: float, dt: float) -> float | None:
    """Move an angular error toward zero without overshooting sign."""
    if current is None:
        return None
    current_value = float(current)
    delta = abs(float(drive_deg_s)) * float(dt)
    if current_value > 0.0:
        return max(0.0, current_value - delta)
    if current_value < 0.0:
        return min(0.0, current_value + delta)
    return 0.0

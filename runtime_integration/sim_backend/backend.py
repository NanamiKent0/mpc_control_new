"""Self-contained lightweight simulator backend for runtime smoke coverage."""

from __future__ import annotations

from dataclasses import dataclass, field

from ...control_core.models.runtime_bridge_types import SchedulerDispatchEnvelope
from .types import (
    SIM_ADJACENT_PAIRS,
    SIM_JOINT_IDS,
    SIM_MODULE_IDS,
    SimCommand,
    SimState,
    command_bend_attr,
    command_linear_attr,
    command_rotate_attr,
    module_bend_attr,
    module_orientation_attr,
    module_position_attr,
    pair_coupling_attr,
    pair_distance_attr,
    pair_key,
    pair_orientation_attr,
)

SIM_BACKEND_VERSION = "phase8.self_contained.v2"


@dataclass(slots=True)
class SimRuntimeBackend:
    """Minimal in-process sim backend owned entirely by `mpc_control_new`."""

    initial_state: SimState
    state: SimState
    dt: float = 0.1
    last_command: SimCommand = field(default_factory=SimCommand)
    last_envelope: SchedulerDispatchEnvelope | None = None
    backend_version: str = SIM_BACKEND_VERSION

    def __init__(self, state: SimState | None = None, *, dt: float = 0.1) -> None:
        self.initial_state = (state or SimState()).copy()
        self.state = self.initial_state.copy()
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
        last_skill_key = None
        last_pair = None
        if self.last_envelope is not None and self.last_envelope.scheduled_command is not None:
            last_skill_key = self.last_envelope.scheduled_command.skill_key
            last_pair = (
                f"{self.last_envelope.scheduled_command.active_module}"
                f"->{self.last_envelope.scheduled_command.passive_module}"
            )
        return {
            "provider_source": "self_contained_sim_backend",
            "dispatch_target": "sim",
            "frame_origin": "sim",
            "sim_backend_version": self.backend_version,
            "sim_dt": float(self.dt),
            "sim_seq": float(self.state.seq),
            "sim_time_s": float(self.state.sim_time_s),
            "coupled_pairs": {
                pair_key(active_module, passive_module): bool(
                    getattr(self.state, pair_coupling_attr(active_module, passive_module))
                )
                for active_module, passive_module in SIM_ADJACENT_PAIRS
            },
            "last_skill_key": last_skill_key,
            "last_pair": last_pair,
        }

    def build_topology_hint(self) -> dict[str, object]:
        """Build a topology hint aligned with the six-module serial chain."""
        return {
            "ordered_modules": list(SIM_MODULE_IDS),
            "active_frontier": ["joint1", "tip"],
            "support_modules": list(SIM_JOINT_IDS),
            "grounded_modules": [SIM_JOINT_IDS[-1]],
            "coupled_pairs": {
                pair_key(active_module, passive_module): bool(
                    getattr(self.state, pair_coupling_attr(active_module, passive_module))
                )
                for active_module, passive_module in SIM_ADJACENT_PAIRS
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
        for module_id in SIM_JOINT_IDS:
            position_attr = module_position_attr(module_id)
            bend_attr = module_bend_attr(module_id)
            orientation_attr = module_orientation_attr(module_id)
            command_bend = command_bend_attr(module_id)
            command_rotate = command_rotate_attr(module_id)
            assert bend_attr is not None
            assert orientation_attr is not None
            assert command_bend is not None
            assert command_rotate is not None
            setattr(
                next_state,
                position_attr,
                float(getattr(next_state, position_attr))
                + float(getattr(command, command_linear_attr(module_id))) * float(self.dt),
            )
            setattr(
                next_state,
                bend_attr,
                float(getattr(next_state, bend_attr))
                + float(getattr(command, command_bend)) * float(self.dt),
            )
            setattr(
                next_state,
                orientation_attr,
                float(getattr(next_state, orientation_attr))
                + float(getattr(command, command_rotate)) * float(self.dt),
            )
        self._advance_pair_geometry(next_state, envelope=envelope, command=command)
        self.state = next_state
        return next_state.copy()

    def stop_all(self) -> None:
        """Zero all command outputs without advancing the simulation."""
        self.last_command = SimCommand()

    def step(self) -> SimState:
        """Advance one simulator tick using the currently latched command."""
        return self.apply_command(self.snapshot_last_command())

    def update(self) -> SimState:
        """Compatibility alias for callers expecting an `update()` backend hook."""
        return self.step()

    def reset(self) -> SimState:
        """Reset the backend state back to the configured initial state."""
        self.state = self.initial_state.copy()
        self.last_command = SimCommand()
        self.last_envelope = None
        return self.snapshot_state()

    def visualization_snapshot(self) -> dict[str, object]:
        """Return a compact snapshot used by GUI and visualizer layers."""
        return {
            "state": self.snapshot_state().to_dict(),
            "last_command": self.snapshot_last_command().to_dict(),
            "diagnostics": self.diagnostics(),
        }

    def _advance_pair_geometry(
        self,
        state: SimState,
        *,
        envelope: SchedulerDispatchEnvelope | None,
        command: SimCommand,
    ) -> None:
        scheduled = None if envelope is None else envelope.scheduled_command
        if scheduled is None:
            for active_module, passive_module in SIM_ADJACENT_PAIRS:
                distance_attr = pair_distance_attr(active_module, passive_module)
                current_distance = getattr(state, distance_attr)
                if current_distance is not None:
                    setattr(
                        state,
                        distance_attr,
                        _pair_distance_after_drive(
                            current=current_distance,
                            drive_mm_s=_pair_linear_drive(command, active_module, passive_module),
                            dt=self.dt,
                        ),
                    )
                orientation_attr = pair_orientation_attr(active_module, passive_module)
                current_orientation = getattr(state, orientation_attr)
                if current_orientation is not None:
                    setattr(
                        state,
                        orientation_attr,
                        _move_toward_zero(
                            current_orientation,
                            drive_deg_s=_pair_rotate_drive(command, active_module, passive_module),
                            dt=self.dt,
                        ),
                    )
                self._update_pair_coupling_state(state, active_module, passive_module, skill_key=None)
            return

        active_module = str(scheduled.active_module)
        passive_module = str(scheduled.passive_module)
        if (active_module, passive_module) not in SIM_ADJACENT_PAIRS:
            return
        distance_attr = pair_distance_attr(active_module, passive_module)
        current_distance = getattr(state, distance_attr)
        if current_distance is not None:
            setattr(
                state,
                distance_attr,
                _pair_distance_after_drive(
                    current=current_distance,
                    drive_mm_s=_module_linear_drive(command, active_module),
                    dt=self.dt,
                ),
            )
        orientation_attr = pair_orientation_attr(active_module, passive_module)
        current_orientation = getattr(state, orientation_attr)
        if current_orientation is not None:
            setattr(
                state,
                orientation_attr,
                _move_toward_zero(
                    current_orientation,
                    drive_deg_s=_module_rotate_drive(command, active_module),
                    dt=self.dt,
                ),
            )
        self._update_pair_coupling_state(
            state,
            active_module,
            passive_module,
            skill_key=scheduled.skill_key,
        )

    @staticmethod
    def _update_pair_coupling_state(
        state: SimState,
        active_module: str,
        passive_module: str,
        *,
        skill_key: str | None,
    ) -> None:
        """Update one pair coupling flag from the stored pair metrics."""
        coupling_attr = pair_coupling_attr(active_module, passive_module)
        distance_value = getattr(state, pair_distance_attr(active_module, passive_module))
        orientation_value = getattr(state, pair_orientation_attr(active_module, passive_module))
        if skill_key == "coarse_approach":
            setattr(state, coupling_attr, False)
            return
        if distance_value is None:
            return
        distance_ready = float(distance_value) <= 1.0
        if _orientation_required_for_coupling(active_module, passive_module):
            if orientation_value is None:
                return
            orientation_ready = abs(float(orientation_value)) <= 2.0
            setattr(state, coupling_attr, bool(distance_ready and orientation_ready))
            return
        setattr(state, coupling_attr, bool(distance_ready))


def _module_linear_drive(command: SimCommand, module_id: str) -> float:
    """Return the positive linear drive applied by one module."""
    return max(float(getattr(command, command_linear_attr(module_id))), 0.0)


def _module_rotate_drive(command: SimCommand, module_id: str) -> float:
    """Return the absolute rotational drive applied by one module."""
    rotate_attr = command_rotate_attr(module_id)
    if rotate_attr is None:
        return 0.0
    return abs(float(getattr(command, rotate_attr)))


def _pair_linear_drive(command: SimCommand, active_module: str, passive_module: str) -> float:
    """Return the linear drive that heuristically closes one pair without a scheduled envelope."""
    return _module_linear_drive(command, active_module) + _module_linear_drive(command, passive_module)


def _pair_rotate_drive(command: SimCommand, active_module: str, passive_module: str) -> float:
    """Return the rotational drive that heuristically reduces one pair's orientation error."""
    return max(_module_rotate_drive(command, active_module), _module_rotate_drive(command, passive_module))


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


def _orientation_required_for_coupling(active_module: str, passive_module: str) -> bool:
    """Return whether the pair still requires orientation closure before capture."""
    return not (str(active_module) == "joint1" and str(passive_module) == "tip")


__all__ = ["SIM_BACKEND_VERSION", "SimRuntimeBackend"]

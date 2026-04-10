"""Simulation-backed observation provider for the self-contained runtime path."""

from __future__ import annotations

from .observation_types import ModuleObservation, PairObservation, RuntimeObservationFrame
from .sim_backend import SIM_BACKEND_VERSION, SimRuntimeBackend


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

    def start(self) -> None:
        """Mark the provider as ready for frame reads."""
        self._started = True

    def stop(self) -> None:
        """Mark the provider as stopped."""
        self._started = False

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
            **self.backend.diagnostics(),
        }

    def get_latest_frame(self) -> RuntimeObservationFrame | None:
        """Read the latest backend state and convert it into a runtime frame."""
        if not self._started:
            return None
        state = self.backend.snapshot_state()
        command = self.backend.snapshot_last_command()
        timestamp_ns = int(round(float(state.sim_time_s) * 1_000_000_000.0))
        modules = {
            "tip": ModuleObservation(
                module_id="tip",
                module_type="tip",
                dofs={"growth_mm": float(state.g)},
                velocities={"growth_mm_s": float(command.tip_growth_mm_s)},
                attach_state={"joint1": bool(state.tip_joint1_coupled)},
                diagnostics={"sim_seq": state.seq, "provider_source": "self_contained_sim_backend"},
                source_name=self.source_name,
            ),
            "joint1": ModuleObservation(
                module_id="joint1",
                module_type="joint",
                dofs={
                    "crawl_mm": float(state.c1),
                    "bend_deg": float(state.theta1),
                    "rotate_deg": float(state.psi1),
                },
                velocities={
                    "crawl_mm_s": float(command.joint1_crawl_mm_s),
                    "bend_deg_s": float(command.joint1_bend_deg_s),
                    "rotate_deg_s": float(command.joint1_rotate_deg_s),
                },
                attach_state={
                    "tip": bool(state.tip_joint1_coupled),
                    "joint2": bool(state.joint1_joint2_coupled),
                },
                diagnostics={"sim_seq": state.seq, "provider_source": "self_contained_sim_backend"},
                source_name=self.source_name,
            ),
            "joint2": ModuleObservation(
                module_id="joint2",
                module_type="joint",
                dofs={
                    "crawl_mm": float(state.c2),
                    "bend_deg": float(state.theta2),
                    "rotate_deg": float(state.psi2),
                },
                velocities={
                    "crawl_mm_s": float(command.joint2_crawl_mm_s),
                    "bend_deg_s": float(command.joint2_bend_deg_s),
                    "rotate_deg_s": float(command.joint2_rotate_deg_s),
                },
                attach_state={"joint1": bool(state.joint1_joint2_coupled)},
                diagnostics={"sim_seq": state.seq, "provider_source": "self_contained_sim_backend"},
                source_name=self.source_name,
            ),
        }
        pairs = {
            "joint1->tip": PairObservation(
                active_module="joint1",
                passive_module="tip",
                relation_type="tip_joint",
                distance_mm=_resolved_distance(
                    state.tip_joint1_distance_mm,
                    fallback=abs(float(state.c1) - float(state.g)),
                ),
                orientation_error_deg=_resolved_orientation(
                    state.tip_joint1_orientation_error_deg,
                    fallback=float(state.psi1),
                ),
                coupled=bool(state.tip_joint1_coupled),
                observation_valid=True,
                diagnostics={"sim_seq": state.seq, "provider_source": "self_contained_sim_backend"},
                source_name=self.source_name,
            ),
            "joint2->joint1": PairObservation(
                active_module="joint2",
                passive_module="joint1",
                relation_type="joint_joint",
                distance_mm=_resolved_distance(
                    state.joint1_joint2_distance_mm,
                    fallback=abs(float(state.c2) - float(state.c1)),
                ),
                orientation_error_deg=_resolved_orientation(
                    state.joint1_joint2_orientation_error_deg,
                    fallback=float(state.psi2 - state.psi1),
                ),
                coupled=bool(state.joint1_joint2_coupled),
                observation_valid=True,
                diagnostics={"sim_seq": state.seq, "provider_source": "self_contained_sim_backend"},
                source_name=self.source_name,
            ),
        }
        resolved_topology_hint = self.backend.build_topology_hint()
        resolved_topology_hint.update(self.topology_hint)
        return RuntimeObservationFrame(
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
            },
        )


def _resolved_distance(value: float | None, *, fallback: float) -> float:
    """Return a distance reading with a deterministic fallback."""
    if value is None:
        return float(fallback)
    return float(value)


def _resolved_orientation(value: float | None, *, fallback: float) -> float:
    """Return an orientation error reading with a deterministic fallback."""
    if value is None:
        return float(fallback)
    return float(value)


__all__ = ["SimObservationProvider"]

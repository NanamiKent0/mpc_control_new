"""Simulation-backed dispatcher for the self-contained runtime path."""

from __future__ import annotations

from ..control_core.models.runtime_bridge_types import DispatchResult, SchedulerDispatchEnvelope
from .sim_backend import SimCommand, SimRuntimeBackend


class SimCommandDispatcher:
    """Translate scheduler envelopes into self-contained sim backend commands."""

    dispatcher_kind = "sim"

    def __init__(
        self,
        backend: SimRuntimeBackend | None = None,
        *,
        source_name: str = "sim_command_dispatcher",
    ) -> None:
        self.backend = backend or SimRuntimeBackend()
        self.source_name = source_name
        self._started = False

    def start(self) -> None:
        """Mark the dispatcher as ready for command dispatch."""
        self._started = True

    def stop(self) -> None:
        """Stop the dispatcher and zero its cached command."""
        self._started = False
        self.backend.stop_all()

    def runtime_diagnostics(self) -> dict[str, object]:
        """Return dispatcher diagnostics for runtime-session reporting."""
        return {
            "dispatcher_kind": self.dispatcher_kind,
            "dispatcher": self.source_name,
            "source_name": self.source_name,
            "started": self._started,
            **self.backend.diagnostics(),
        }

    def dispatch(self, envelope: SchedulerDispatchEnvelope) -> DispatchResult:
        """Dispatch one scheduler envelope into the self-contained sim backend."""
        if not self._started:
            return DispatchResult(
                accepted=False,
                reason="dispatcher_not_started",
                diagnostics=self.runtime_diagnostics(),
            )
        scheduled = envelope.scheduled_command
        if scheduled is None:
            return DispatchResult(
                accepted=False,
                dispatched_commands=[],
                reason="dispatch_envelope_missing_command",
                diagnostics={
                    **self.runtime_diagnostics(),
                    "input_source": envelope.input_source,
                    "dispatch_target": envelope.dispatch_target or "sim",
                },
            )

        command = SimCommand()
        accepted_modules: list[str] = []
        accepted_primitives: list[str] = []
        dispatched_commands: list[str] = []
        for reference in scheduled.primitive_references:
            mapped = self._apply_reference(
                command,
                reference.module_id,
                reference.primitive_name,
                reference.reference_value,
            )
            if mapped is None:
                return DispatchResult(
                    accepted=False,
                    dispatched_commands=dispatched_commands,
                    reason=f"unsupported_primitive:{reference.primitive_name or reference.axis}",
                    diagnostics={
                        **self.runtime_diagnostics(),
                        "input_source": envelope.input_source,
                        "dispatch_target": envelope.dispatch_target or "sim",
                        "module_id": reference.module_id,
                        "primitive_name": reference.primitive_name,
                        "accepted_modules": accepted_modules,
                        "accepted_primitives": accepted_primitives,
                    },
                )
            dispatched_commands.append(mapped)
            if reference.module_id not in accepted_modules:
                accepted_modules.append(reference.module_id)
            accepted_primitives.append(reference.primitive_name or reference.axis)

        if not scheduled.primitive_references:
            dispatched_commands.append(f"{scheduled.skill_key}:noop")

        state = self.backend.apply_command(command, envelope=envelope)
        reason = "sim_dispatch_noop_applied" if not scheduled.primitive_references else "sim_dispatch_applied"
        return DispatchResult(
            accepted=True,
            dispatched_commands=dispatched_commands,
            reason=reason,
            diagnostics={
                **self.runtime_diagnostics(),
                "input_source": envelope.input_source,
                "dispatch_target": envelope.dispatch_target or "sim",
                "graph_id": scheduled.graph_id,
                "node_id": scheduled.node_id,
                "skill_key": scheduled.skill_key,
                "accepted_modules": accepted_modules,
                "accepted_primitives": accepted_primitives,
                "dispatch_ready": envelope.dispatch_ready,
                "sim_state_seq": float(state.seq),
                "sim_time_s": float(state.sim_time_s),
            },
        )

    def emergency_stop(self) -> DispatchResult:
        """Stop all sim outputs and return a structured result."""
        self.backend.stop_all()
        return DispatchResult(
            accepted=True,
            dispatched_commands=["emergency_stop"],
            reason="sim_emergency_stop",
            diagnostics=self.runtime_diagnostics(),
        )

    def flush(self) -> DispatchResult:
        """Return a structured no-op flush result for the in-memory sim backend."""
        return DispatchResult(
            accepted=True,
            dispatched_commands=[],
            reason="sim_flush_noop",
            diagnostics=self.runtime_diagnostics(),
        )

    @staticmethod
    def _apply_reference(
        command: SimCommand,
        module_id: str,
        primitive_name: str | None,
        reference_value: float,
    ) -> str | None:
        """Map one primitive reference into the internal sim command object."""
        primitive = primitive_name or ""
        value = float(reference_value)
        if primitive == "tip_growth":
            command.tip_growth_mm_s = value
            return f"{module_id}:tip_growth"
        if primitive == "joint_crawl":
            setattr(command, f"{module_id}_crawl_mm_s", value)
            return f"{module_id}:joint_crawl"
        if primitive == "joint_rotate":
            setattr(command, f"{module_id}_rotate_deg_s", value)
            return f"{module_id}:joint_rotate"
        if primitive == "joint_bend":
            setattr(command, f"{module_id}_bend_deg_s", value)
            return f"{module_id}:joint_bend"
        if primitive in {"joint_rotate_hold", "joint_bend_hold"}:
            return f"{module_id}:{primitive}"
        return None


__all__ = ["SimCommandDispatcher", "SimRuntimeBackend"]

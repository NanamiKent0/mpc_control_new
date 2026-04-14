"""Simulation-backed dispatcher for the self-contained runtime path."""

from __future__ import annotations

from ..control_core.models.runtime_bridge_types import DispatchResult, SchedulerDispatchEnvelope
from .common.encoder_protocol import counts_to_physical
from .ros2_interfaces import GUI_ROS2_COMMAND_TOPICS, GUI_ROS2_TOPIC_NAMESPACES
from .ros2_message_adapters import normalize_gui_ros2_command_payload
from .sim_backend.backend import SimRuntimeBackend
from .sim_backend.types import (
    SIM_JOINT_IDS,
    SimCommand,
    command_bend_attr,
    command_electromagnet_attr,
    command_linear_attr,
    command_rotate_attr,
    joint_index,
)


class SimCommandDispatcher:
    """Translate scheduler envelopes or GUI compact commands into sim backend state."""

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
        self._published_primitive_count = 0
        self._extra_namespace_commands: dict[str, dict[int, object]] = {}
        self.last_reason = "sim_dispatcher_not_started"

    def start(self) -> None:
        """Mark the dispatcher as ready for command dispatch."""
        self._started = True
        self.last_reason = "sim_dispatcher_started"

    def stop(self) -> None:
        """Stop the dispatcher and zero its cached command."""
        self._started = False
        self.backend.stop_all()
        self._extra_namespace_commands = {}
        self.last_reason = "sim_dispatcher_stopped"

    def reset(self) -> None:
        """Reset counters and the shared sim backend command cache."""
        self._published_primitive_count = 0
        self.backend.stop_all()
        self._extra_namespace_commands = {}
        self.last_reason = "sim_dispatcher_reset"

    def runtime_diagnostics(self) -> dict[str, object]:
        """Return dispatcher diagnostics for runtime-session reporting."""
        return {
            "dispatcher_kind": self.dispatcher_kind,
            "dispatcher": self.source_name,
            "source_name": self.source_name,
            "started": self._started,
            "published_primitive_count": self._published_primitive_count,
            "publish_target_topics": list(GUI_ROS2_COMMAND_TOPICS),
            "topic_namespaces": list(GUI_ROS2_TOPIC_NAMESPACES),
            "gui_ros2_compatible": True,
            "reason": self.last_reason,
            "backend_state": self.backend.snapshot_state().to_dict(),
            "last_command": self.backend.snapshot_last_command().to_dict(),
            "extra_namespace_commands": dict(self._extra_namespace_commands),
            **self.backend.diagnostics(),
        }

    def dispatch(self, envelope: SchedulerDispatchEnvelope) -> DispatchResult:
        """Dispatch one scheduler envelope into the self-contained sim backend."""
        if not self._started:
            self.last_reason = "dispatcher_not_started"
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
            self._published_primitive_count += 1

        if not scheduled.primitive_references:
            dispatched_commands.append(f"{scheduled.skill_key}:noop")

        state = self.backend.apply_command(command, envelope=envelope)
        reason = "sim_dispatch_noop_applied" if not scheduled.primitive_references else "sim_dispatch_applied"
        self.last_reason = reason
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
                "published_primitive_count": self._published_primitive_count,
            },
        )

    def apply_gui_ros2_command(
        self,
        payload: object,
        *,
        namespace: str | None = None,
    ) -> dict[str, object]:
        """Apply one `gui_ros2.py` compact command without advancing the simulation."""
        normalized = normalize_gui_ros2_command_payload(payload, namespace=namespace)
        command = self.backend.snapshot_last_command()
        diagnostics = {
            "namespace": normalized.namespace,
            "motor_id": normalized.motor_id,
            "cmd_type": normalized.cmd_type,
            "gui_ros2_compatible": True,
        }
        if normalized.cmd_type == 0x02:
            self._apply_gui_velocity_command(command, normalized)
            diagnostics["accepted"] = True
            diagnostics["semantic"] = "velocity"
        elif normalized.cmd_type == 0x01:
            self._apply_gui_stop_command(command, normalized)
            diagnostics["accepted"] = True
            diagnostics["semantic"] = "stop"
        elif normalized.cmd_type == 0x03:
            self._apply_gui_zero_command(normalized)
            diagnostics["accepted"] = True
            diagnostics["semantic"] = "zero"
        elif normalized.cmd_type == 0x05:
            self._apply_gui_electromagnet_command(command, normalized)
            diagnostics["accepted"] = True
            diagnostics["semantic"] = "electromagnet"
        else:
            diagnostics["accepted"] = False
            diagnostics["semantic"] = "unsupported"
            diagnostics["reason"] = "unsupported_gui_cmd_type"
            self.last_reason = "sim_gui_ros2_command_unsupported"
            return diagnostics
        self.backend.last_command = command
        self._published_primitive_count += 1
        self.last_reason = "sim_gui_ros2_command_applied"
        return diagnostics

    def emergency_stop(self) -> DispatchResult:
        """Stop all sim outputs and return a structured result."""
        self.backend.stop_all()
        self._extra_namespace_commands = {}
        self.last_reason = "sim_emergency_stop"
        return DispatchResult(
            accepted=True,
            dispatched_commands=["emergency_stop"],
            reason="sim_emergency_stop",
            diagnostics=self.runtime_diagnostics(),
        )

    def flush(self) -> DispatchResult:
        """Return a structured no-op flush result for the in-memory sim backend."""
        self.last_reason = "sim_flush_noop"
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

    def snapshot_backend_state(self) -> dict[str, object]:
        """Return the latest backend snapshot for GUI or test consumers."""
        snapshot = self.backend.visualization_snapshot()
        snapshot["extra_namespace_commands"] = dict(self._extra_namespace_commands)
        return snapshot

    def _apply_gui_velocity_command(
        self,
        command: SimCommand,
        payload: object,
    ) -> None:
        normalized = (
            payload
            if hasattr(payload, "namespace") and hasattr(payload, "motor_id")
            else normalize_gui_ros2_command_payload(payload)
        )
        velocity_value, _ = counts_to_physical(normalized.motor_id, normalized.param1)
        if normalized.namespace == "tip" and normalized.motor_id == 4:
            command.tip_growth_mm_s = float(velocity_value)
            return
        if normalized.namespace in SIM_JOINT_IDS and joint_index(normalized.namespace) is not None:
            rotate_attr = command_rotate_attr(normalized.namespace)
            bend_attr = command_bend_attr(normalized.namespace)
            assert rotate_attr is not None
            assert bend_attr is not None
            if normalized.motor_id == 1:
                setattr(command, command_linear_attr(normalized.namespace), float(velocity_value))
            elif normalized.motor_id == 2:
                setattr(command, rotate_attr, float(velocity_value))
            elif normalized.motor_id == 3:
                setattr(command, bend_attr, float(velocity_value))
            return
        namespace_cache = self._extra_namespace_commands.setdefault(normalized.namespace, {})
        namespace_cache[normalized.motor_id] = float(velocity_value)

    def _apply_gui_stop_command(self, command: SimCommand, payload: object) -> None:
        normalized = (
            payload
            if hasattr(payload, "namespace") and hasattr(payload, "motor_id")
            else normalize_gui_ros2_command_payload(payload)
        )
        if normalized.namespace == "tip" and normalized.motor_id == 4:
            command.tip_growth_mm_s = 0.0
            return
        if normalized.namespace in SIM_JOINT_IDS and joint_index(normalized.namespace) is not None:
            rotate_attr = command_rotate_attr(normalized.namespace)
            bend_attr = command_bend_attr(normalized.namespace)
            assert rotate_attr is not None
            assert bend_attr is not None
            if normalized.motor_id == 1:
                setattr(command, command_linear_attr(normalized.namespace), 0.0)
            elif normalized.motor_id == 2:
                setattr(command, rotate_attr, 0.0)
            elif normalized.motor_id == 3:
                setattr(command, bend_attr, 0.0)
            return
        namespace_cache = self._extra_namespace_commands.setdefault(normalized.namespace, {})
        namespace_cache[normalized.motor_id] = 0.0

    def _apply_gui_zero_command(self, payload: object) -> None:
        normalized = (
            payload
            if hasattr(payload, "namespace") and hasattr(payload, "motor_id")
            else normalize_gui_ros2_command_payload(payload)
        )
        state = self.backend.state
        if normalized.namespace == "tip" and normalized.motor_id == 4:
            state.g = 0.0
            return
        resolved_joint_index = joint_index(normalized.namespace)
        if resolved_joint_index is not None:
            self._zero_joint_state(state, joint_index=resolved_joint_index, motor_id=normalized.motor_id)
            return
        namespace_cache = self._extra_namespace_commands.setdefault(normalized.namespace, {})
        namespace_cache[normalized.motor_id] = 0.0

    def _apply_gui_electromagnet_command(self, command: SimCommand, payload: object) -> None:
        normalized = (
            payload
            if hasattr(payload, "namespace") and hasattr(payload, "motor_id")
            else normalize_gui_ros2_command_payload(payload)
        )
        magnet_on = bool(normalized.param1)
        if normalized.namespace == "tip":
            command.tip_electromagnet_on = magnet_on
        elif normalized.namespace in SIM_JOINT_IDS and joint_index(normalized.namespace) is not None:
            setattr(command, command_electromagnet_attr(normalized.namespace), magnet_on)
        else:
            namespace_cache = self._extra_namespace_commands.setdefault(normalized.namespace, {})
            namespace_cache[99] = magnet_on

    @staticmethod
    def _zero_joint_state(state: object, *, joint_index: int, motor_id: int) -> None:
        if motor_id == 1:
            setattr(state, f"c{joint_index}", 0.0)
        elif motor_id == 2:
            setattr(state, f"psi{joint_index}", 0.0)
        elif motor_id == 3:
            setattr(state, f"theta{joint_index}", 0.0)


__all__ = ["SimCommandDispatcher", "SimRuntimeBackend"]

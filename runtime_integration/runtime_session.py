"""Runtime-facing session loop for the refactored main execution path."""

from __future__ import annotations

from dataclasses import dataclass, field

from ..control_core.models.execution_context import ExecutionContext
from ..control_core.models.runtime_bridge_types import DispatchResult, SchedulerDispatchEnvelope
from ..control_core.models.task_types import SchedulerStepResult, TaskGraphSpec
from ..control_core.orchestration.graph_factories import build_runtime_demo_pair_graph
from ..control_core.orchestration.skill_registry import SkillRegistry, build_default_skill_registry
from ..control_core.orchestration.skill_scheduler import SkillScheduler
from ..control_core.orchestration.transition_policy_registry import (
    TransitionPolicyRegistry,
    build_default_transition_policy_registry,
)
from ..control_core.topology.chain_topology import ChainTopology
from ..control_core.topology.pair_registry import PairExtractorRegistry
from ..controllers.adapters.legacy_extractors import build_default_pair_extractor_registry
from ..controllers.adapters.skill_controller_adapter import SkillControllerAdapter
from .dispatcher_interfaces import CommandDispatcher
from .live_command_dispatcher import LiveRuntimeCommandDispatcher
from .live_runtime_provider import LiveRuntimeObservationProvider
from .observation_types import RuntimeObservationFrame
from .provider_interfaces import ObservationProvider
from .ros2_runtime_node import Ros2RuntimeNode
from .runtime_state_builder import build_topology_from_frame
from .sim_backend import SimRuntimeBackend, SimState
from .sim_command_dispatcher import SimCommandDispatcher
from .sim_observation_provider import SimObservationProvider


@dataclass(slots=True)
class RuntimeStepResult:
    """Structured result of one runtime-session step."""

    frame: RuntimeObservationFrame | None = None
    scheduler_step_result: SchedulerStepResult | None = None
    dispatch_envelope: SchedulerDispatchEnvelope | None = None
    dispatch_result: DispatchResult | None = None
    accepted: bool = False
    reason: str | None = None
    input_source: str | None = None
    dispatch_target: str | None = None
    provider_kind: str | None = None
    dispatcher_kind: str | None = None
    diagnostics: dict[str, object] = field(default_factory=dict)


class RuntimeSession:
    """Own the observation -> scheduler -> dispatch main path for the new architecture."""

    def __init__(
        self,
        *,
        observation_provider: ObservationProvider,
        command_dispatcher: CommandDispatcher,
        graph_spec: TaskGraphSpec | None = None,
        scheduler: SkillScheduler | None = None,
        skill_registry: SkillRegistry | None = None,
        transition_policy_registry: TransitionPolicyRegistry | None = None,
        pair_registry: PairExtractorRegistry | None = None,
        topology: ChainTopology | None = None,
        base_context: ExecutionContext | None = None,
    ) -> None:
        self.observation_provider = observation_provider
        self.command_dispatcher = command_dispatcher
        self.topology = topology or ChainTopology(ordered_modules=["tip", "joint1", "joint2"])
        self.skill_registry = skill_registry or build_default_skill_registry(topology=self.topology)
        self.transition_policy_registry = (
            transition_policy_registry or build_default_transition_policy_registry()
        )
        self.pair_registry = pair_registry or build_default_pair_extractor_registry()
        self.scheduler = scheduler or SkillScheduler(
            adapter=SkillControllerAdapter(
                topology=self.topology,
                skill_registry=self.skill_registry,
                pair_registry=self.pair_registry,
            ),
            skill_registry=self.skill_registry,
            pair_registry=self.pair_registry,
            transition_policy_registry=self.transition_policy_registry,
        )
        self.graph_spec = graph_spec
        self.base_context = base_context or ExecutionContext(
            topology=self.topology,
            metadata={"runtime_main_path": True},
            input_source="runtime_frame",
            topology_source="session_default",
        )
        self._started = False

    def start(self) -> None:
        """Start the provider/dispatcher pair and load the configured task graph."""
        if self.scheduler.task_graph is None:
            if self.graph_spec is None:
                raise ValueError("runtime_session_graph_not_loaded")
            self.scheduler.load_graph(self.graph_spec)
        self.observation_provider.start()
        self.command_dispatcher.start()
        self.observation_provider.warmup()
        self._started = True

    def stop(self) -> None:
        """Stop the provider/dispatcher pair."""
        self.command_dispatcher.stop()
        self.observation_provider.stop()
        self._started = False

    def run_once(self, *, context: ExecutionContext | None = None) -> RuntimeStepResult:
        """Run one runtime step."""
        return self.step(context=context)

    def step(self, *, context: ExecutionContext | None = None) -> RuntimeStepResult:
        """Execute one full observation -> scheduler -> dispatch cycle."""
        if not self._started:
            self.start()
        provider_kind = _provider_kind(self.observation_provider)
        dispatcher_kind = _dispatcher_kind(self.command_dispatcher)
        input_source = _runtime_input_source(provider_kind)
        provider_status = _component_runtime_diagnostics(self.observation_provider)
        dispatcher_status = _component_runtime_diagnostics(self.command_dispatcher)

        frame = self.observation_provider.get_latest_frame()
        if frame is None:
            return RuntimeStepResult(
                frame=None,
                scheduler_step_result=None,
                dispatch_envelope=None,
                dispatch_result=None,
                accepted=False,
                reason="observation_unavailable",
                input_source=input_source,
                dispatch_target=dispatcher_kind,
                provider_kind=provider_kind,
                dispatcher_kind=dispatcher_kind,
                diagnostics={
                    "input_source": input_source,
                    "dispatch_target": dispatcher_kind,
                    "provider_kind": provider_kind,
                    "dispatcher_kind": dispatcher_kind,
                    "scheduler_input_source": None,
                    "dispatch_ready": False,
                    "accepted": False,
                    "reason": "observation_unavailable",
                    "provider_status": provider_status,
                    "dispatcher_status": dispatcher_status,
                },
            )

        execution_context = self._build_context(
            frame,
            context=context,
            provider_kind=provider_kind,
            dispatcher_kind=dispatcher_kind,
        )
        scheduler_step_result = self.scheduler.step(frame, context=execution_context)
        dispatch_envelope = self.scheduler.to_dispatch_envelope(scheduler_step_result)
        dispatch_envelope.input_source = input_source
        dispatch_envelope.dispatch_target = dispatcher_kind
        dispatch_envelope.provider_kind = provider_kind
        dispatch_envelope.dispatcher_kind = dispatcher_kind
        dispatch_envelope.diagnostics.update(
            {
                "input_source": input_source,
                "dispatch_target": dispatcher_kind,
                "provider_kind": provider_kind,
                "dispatcher_kind": dispatcher_kind,
            }
        )
        if dispatch_envelope.dispatch_ready:
            dispatch_result = self.command_dispatcher.dispatch(dispatch_envelope)
        else:
            dispatch_result = DispatchResult(
                accepted=False,
                dispatched_commands=[],
                reason="dispatch_not_ready",
                diagnostics={
                    "dispatch_target": dispatcher_kind,
                    "dispatch_ready": False,
                    "dispatcher_kind": dispatcher_kind,
                },
            )
        dispatcher_status = _component_runtime_diagnostics(self.command_dispatcher)
        reason = dispatch_result.reason or scheduler_step_result.transition_reason
        return RuntimeStepResult(
            frame=frame,
            scheduler_step_result=scheduler_step_result,
            dispatch_envelope=dispatch_envelope,
            dispatch_result=dispatch_result,
            accepted=bool(dispatch_result.accepted),
            reason=reason,
            input_source=input_source,
            dispatch_target=dispatcher_kind,
            provider_kind=provider_kind,
            dispatcher_kind=dispatcher_kind,
            diagnostics={
                "input_source": input_source,
                "dispatch_target": dispatcher_kind,
                "provider_kind": provider_kind,
                "dispatcher_kind": dispatcher_kind,
                "scheduler_input_source": scheduler_step_result.diagnostics.get("scheduler_input_source"),
                "dispatch_ready": dispatch_envelope.dispatch_ready,
                "accepted": bool(dispatch_result.accepted),
                "reason": reason,
                "state_builder_source": scheduler_step_result.diagnostics.get("state_builder_source"),
                "legacy_path_used": scheduler_step_result.diagnostics.get("legacy_path_used"),
                "context_topology_source": scheduler_step_result.diagnostics.get("context_topology_source"),
                "scheduler_transition_reason": scheduler_step_result.transition_reason,
                "dispatch_reason": dispatch_result.reason,
                "provider_status": provider_status,
                "dispatcher_status": dispatcher_status,
            },
        )

    def emergency_stop(self) -> DispatchResult:
        """Issue a minimal emergency stop through the configured dispatcher."""
        return self.command_dispatcher.emergency_stop()

    def _build_context(
        self,
        frame: RuntimeObservationFrame,
        *,
        context: ExecutionContext | None,
        provider_kind: str,
        dispatcher_kind: str,
    ) -> ExecutionContext:
        """Build the execution context for one runtime-frame scheduler step."""
        resolved_context = self.base_context if context is None else self.base_context.merged_metadata(context.metadata).with_updates(
            topology=context.topology or self.base_context.topology,
            dt=context.dt,
            target_ns=context.target_ns,
        )
        topology_source = "runtime_frame_hint" if frame.topology_hint else "session_default"
        topology = build_topology_from_frame(
            frame,
            fallback_topology=resolved_context.topology or self.topology,
        )
        dt = resolved_context.dt
        if dt is None:
            dt = _frame_dt(frame)
        metadata = {
            **dict(resolved_context.metadata),
            "runtime_session": True,
            "runtime_frame_timestamp_ns": frame.timestamp_ns,
            "runtime_frame_metadata": dict(frame.metadata),
            "runtime_topology_source": topology_source,
            "runtime_provider_kind": provider_kind,
            "runtime_dispatcher_kind": dispatcher_kind,
            "runtime_input_source": _runtime_input_source(provider_kind),
        }
        return ExecutionContext(
            topology=topology,
            dt=dt,
            target_ns=frame.timestamp_ns,
            metadata=metadata,
            input_source="runtime_frame",
            frame_timestamp_ns=frame.timestamp_ns,
            topology_source=topology_source,
        )


def build_sim_runtime_session(
    *,
    graph_spec: TaskGraphSpec | None = None,
    backend: SimRuntimeBackend | None = None,
    initial_state: SimState | None = None,
    dt: float = 0.1,
) -> RuntimeSession:
    """Build the self-contained sim runtime session used by tests and local development."""
    resolved_backend = backend or SimRuntimeBackend(state=initial_state, dt=dt)
    resolved_graph = graph_spec or build_runtime_demo_pair_graph(
        graph_id="runtime_sim_demo",
        include_finalize_node=False,
    )
    return RuntimeSession(
        observation_provider=SimObservationProvider(backend=resolved_backend),
        command_dispatcher=SimCommandDispatcher(backend=resolved_backend),
        graph_spec=resolved_graph,
    )


def build_ros2_runtime_session(
    *,
    graph_spec: TaskGraphSpec | None = None,
    runtime_node: Ros2RuntimeNode | None = None,
    enable_rclpy: bool = False,
) -> RuntimeSession:
    """Build the ROS2 live runtime session entry point used by the new architecture."""
    shared_node = runtime_node or Ros2RuntimeNode(enable_rclpy=enable_rclpy)
    resolved_graph = graph_spec or build_runtime_demo_pair_graph(
        graph_id="runtime_ros2_demo",
        include_finalize_node=False,
    )
    return RuntimeSession(
        observation_provider=LiveRuntimeObservationProvider(runtime_node=shared_node),
        command_dispatcher=LiveRuntimeCommandDispatcher(runtime_node=shared_node),
        graph_spec=resolved_graph,
    )


def _provider_kind(provider: ObservationProvider) -> str:
    """Return the stable provider kind label used by runtime-session diagnostics."""
    value = getattr(provider, "provider_kind", None)
    if isinstance(value, str) and value:
        return value
    return "unknown"


def _dispatcher_kind(dispatcher: CommandDispatcher) -> str:
    """Return the stable dispatcher kind label used by runtime-session diagnostics."""
    value = getattr(dispatcher, "dispatcher_kind", None)
    if isinstance(value, str) and value:
        return value
    return "unknown"


def _component_runtime_diagnostics(component: object) -> dict[str, object]:
    """Read component diagnostics when the adapter exposes them."""
    method = getattr(component, "runtime_diagnostics", None)
    if callable(method):
        diagnostics = method()
        if isinstance(diagnostics, dict):
            return diagnostics
    return {}


def _runtime_input_source(provider_kind: str) -> str:
    """Return the runtime-step input-source label, preserving compatibility for generic doubles."""
    if provider_kind == "unknown":
        return "runtime_frame"
    return f"runtime_frame:{provider_kind}"


def _frame_dt(frame: RuntimeObservationFrame) -> float | None:
    """Resolve an optional dt hint from runtime-frame metadata."""
    value = frame.metadata.get("dt")
    if isinstance(value, (int, float)):
        return float(value)
    return None


__all__ = [
    "RuntimeSession",
    "RuntimeStepResult",
    "build_ros2_runtime_session",
    "build_sim_runtime_session",
]

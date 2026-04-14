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
from ..control_core.controllers.adapters.legacy_extractors import (
    build_default_pair_extractor_registry,
)
from ..control_core.controllers.adapters.skill_controller_adapter import (
    SkillControllerAdapter,
)
from .dispatcher_interfaces import CommandDispatcher
from .observation_types import RuntimeObservationFrame
from .provider_interfaces import ObservationProvider
from .ros2_runtime_node import Ros2RuntimeNode
from .ros2_topic_config import Ros2TopicConfig
from .ros2_topic_runtime import build_ros2_runtime_components
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
    graph_id: str | None = None
    current_node_id: str | None = None
    active_skill_key: str | None = None
    provider_status: dict[str, object] = field(default_factory=dict)
    dispatcher_status: dict[str, object] = field(default_factory=dict)
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
        session_name: str = "runtime_session",
    ) -> None:
        self.observation_provider = observation_provider
        self.command_dispatcher = command_dispatcher
        self.session_name = session_name
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
        self.last_step_result: RuntimeStepResult | None = None

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

    def load_graph(self, graph_spec: TaskGraphSpec) -> None:
        """Load a new task graph into the session scheduler."""
        self.graph_spec = graph_spec
        self.scheduler.load_graph(graph_spec)

    def reset(self, *, reset_backend: bool = True) -> dict[str, object]:
        """Reset the scheduler and any optional backend-owned caches."""
        if reset_backend:
            _call_optional(self.command_dispatcher, "reset")
            _call_optional(self.observation_provider, "reset")
        if self.scheduler.task_graph is None:
            if self.graph_spec is None:
                raise ValueError("runtime_session_graph_not_loaded")
            self.scheduler.load_graph(self.graph_spec)
        else:
            self.scheduler.reset()
        self.last_step_result = None
        return self.snapshot_status()

    def snapshot_status(self) -> dict[str, object]:
        """Return a structured snapshot for GUI, demos, and tests."""
        provider_kind = _provider_kind(self.observation_provider)
        dispatcher_kind = _dispatcher_kind(self.command_dispatcher)
        provider_status = _component_runtime_diagnostics(self.observation_provider)
        dispatcher_status = _component_runtime_diagnostics(self.command_dispatcher)
        scheduler_state = self.scheduler.state
        graph_metadata = {} if self.graph_spec is None else dict(self.graph_spec.metadata)
        topic_namespaces = _topic_namespaces(provider_status, dispatcher_status)
        gui_ros2_compatible = bool(
            provider_status.get("gui_ros2_compatible") or dispatcher_status.get("gui_ros2_compatible")
        )
        return {
            "session_name": self.session_name,
            "started": self._started,
            "provider_kind": provider_kind,
            "dispatcher_kind": dispatcher_kind,
            "scheduler_input_source": "runtime_frame",
            "graph_id": scheduler_state.graph_id or (None if self.graph_spec is None else self.graph_spec.graph_id),
            "graph_metadata": graph_metadata,
            "current_node_id": scheduler_state.current_node_id,
            "previous_node_id": scheduler_state.previous_node_id,
            "is_finished": scheduler_state.is_finished,
            "dispatch_target": dispatcher_kind,
            "dispatch_ready": bool(self.last_step_result and self.last_step_result.diagnostics.get("dispatch_ready")),
            "legacy_path_used": False,
            "self_contained": True,
            "gui_ros2_compatible": gui_ros2_compatible,
            "topic_namespaces": topic_namespaces,
            "provider_status": provider_status,
            "dispatcher_status": dispatcher_status,
            "last_reason": None if self.last_step_result is None else self.last_step_result.reason,
        }

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
        scheduler_state = self.scheduler.state

        frame = self.observation_provider.get_latest_frame()
        if frame is None:
            return self._store_result(
                RuntimeStepResult(
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
                    graph_id=scheduler_state.graph_id,
                    current_node_id=scheduler_state.current_node_id,
                    active_skill_key=scheduler_state.last_skill_key,
                    provider_status=provider_status,
                    dispatcher_status=dispatcher_status,
                    diagnostics={
                        "input_source": input_source,
                        "dispatch_target": dispatcher_kind,
                        "provider_kind": provider_kind,
                        "dispatcher_kind": dispatcher_kind,
                        "scheduler_input_source": "runtime_frame",
                        "dispatch_ready": False,
                        "accepted": False,
                        "reason": "observation_unavailable",
                        "graph_id": scheduler_state.graph_id,
                        "current_node_id": scheduler_state.current_node_id,
                        "active_skill_key": scheduler_state.last_skill_key,
                        "provider_status": provider_status,
                        "dispatcher_status": dispatcher_status,
                        "legacy_path_used": False,
                        "self_contained": True,
                        "gui_ros2_compatible": bool(
                            provider_status.get("gui_ros2_compatible")
                            or dispatcher_status.get("gui_ros2_compatible")
                        ),
                        "topic_namespaces": _topic_namespaces(provider_status, dispatcher_status),
                    },
                )
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
        dispatch_envelope.provider_hint = provider_kind
        dispatch_envelope.dispatcher_hint = dispatcher_kind
        dispatch_envelope.bridge_source = "runtime_session.scheduler_bridge"
        dispatch_envelope.diagnostics.update(
            {
                "input_source": input_source,
                "dispatch_target": dispatcher_kind,
                "provider_kind": provider_kind,
                "dispatcher_kind": dispatcher_kind,
                "bridge_source": "runtime_session.scheduler_bridge",
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
        provider_status = _component_runtime_diagnostics(self.observation_provider)
        dispatcher_status = _component_runtime_diagnostics(self.command_dispatcher)
        reason = dispatch_result.reason or scheduler_step_result.transition_reason
        active_skill_key = _active_skill_key(scheduler_step_result)
        result = RuntimeStepResult(
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
            graph_id=scheduler_step_result.scheduler_state.graph_id,
            current_node_id=scheduler_step_result.scheduler_state.current_node_id,
            active_skill_key=active_skill_key,
            provider_status=provider_status,
            dispatcher_status=dispatcher_status,
            diagnostics={
                "input_source": input_source,
                "dispatch_target": dispatcher_kind,
                "provider_kind": provider_kind,
                "dispatcher_kind": dispatcher_kind,
                "scheduler_input_source": scheduler_step_result.diagnostics.get(
                    "scheduler_input_source",
                    "runtime_frame",
                ),
                "dispatch_ready": dispatch_envelope.dispatch_ready,
                "accepted": bool(dispatch_result.accepted),
                "reason": reason,
                "graph_id": scheduler_step_result.scheduler_state.graph_id,
                "current_node_id": scheduler_step_result.scheduler_state.current_node_id,
                "active_skill_key": active_skill_key,
                "state_builder_source": scheduler_step_result.diagnostics.get("state_builder_source"),
                "legacy_path_used": False,
                "self_contained": True,
                "gui_ros2_compatible": bool(
                    provider_status.get("gui_ros2_compatible")
                    or dispatcher_status.get("gui_ros2_compatible")
                ),
                "topic_namespaces": _topic_namespaces(provider_status, dispatcher_status),
                "context_topology_source": scheduler_step_result.diagnostics.get(
                    "context_topology_source"
                ),
                "scheduler_transition_reason": scheduler_step_result.transition_reason,
                "dispatch_reason": dispatch_result.reason,
                "provider_status": provider_status,
                "dispatcher_status": dispatcher_status,
            },
        )
        return self._store_result(result)

    def emergency_stop(self) -> DispatchResult:
        """Issue a minimal emergency stop through the configured dispatcher."""
        return self.command_dispatcher.emergency_stop()

    def _store_result(self, result: RuntimeStepResult) -> RuntimeStepResult:
        """Cache the latest step result for GUI and demo consumers."""
        self.last_step_result = result
        return result

    def _build_context(
        self,
        frame: RuntimeObservationFrame,
        *,
        context: ExecutionContext | None,
        provider_kind: str,
        dispatcher_kind: str,
    ) -> ExecutionContext:
        """Build the execution context for one runtime-frame scheduler step."""
        if context is None:
            resolved_context = self.base_context
        else:
            resolved_context = self.base_context.merged_metadata(context.metadata).with_updates(
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
        session_name="sim_runtime_session",
    )


def build_ros2_runtime_session(
    *,
    graph_spec: TaskGraphSpec | None = None,
    runtime_node: Ros2RuntimeNode | None = None,
    topic_config: Ros2TopicConfig | None = None,
    enable_rclpy: bool = False,
) -> RuntimeSession:
    """Build the ROS2 live runtime session entry point used by the new architecture."""
    components = build_ros2_runtime_components(
        topic_config=topic_config,
        runtime_node=runtime_node,
        enable_rclpy=enable_rclpy,
    )
    resolved_graph = graph_spec or build_runtime_demo_pair_graph(
        graph_id="runtime_ros2_demo",
        include_finalize_node=False,
    )
    return RuntimeSession(
        observation_provider=components.observation_provider,
        command_dispatcher=components.command_dispatcher,
        graph_spec=resolved_graph,
        session_name="ros2_runtime_session",
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


def _topic_namespaces(
    provider_status: dict[str, object],
    dispatcher_status: dict[str, object],
) -> list[str]:
    """Resolve the stable GUI-topic namespace list visible to the runtime session."""
    provider_namespaces = provider_status.get("topic_namespaces")
    if isinstance(provider_namespaces, list):
        return [str(item) for item in provider_namespaces]
    dispatcher_namespaces = dispatcher_status.get("topic_namespaces")
    if isinstance(dispatcher_namespaces, list):
        return [str(item) for item in dispatcher_namespaces]
    return []


def _active_skill_key(step_result: SchedulerStepResult) -> str | None:
    """Resolve the active skill key from one scheduler step result."""
    if step_result.skill_result is not None and step_result.skill_result.skill_key:
        return step_result.skill_result.skill_key
    if step_result.node is not None:
        return step_result.node.skill_spec.skill_key
    return None


def _call_optional(target: object, method_name: str) -> None:
    """Invoke an optional component method when it exists."""
    method = getattr(target, method_name, None)
    if callable(method):
        method()


__all__ = [
    "RuntimeSession",
    "RuntimeStepResult",
    "build_ros2_runtime_session",
    "build_sim_runtime_session",
]

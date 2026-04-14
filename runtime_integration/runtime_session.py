"""Runtime-facing session loop for the refactored main execution path."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field

from ..control_core.models.execution_context import ExecutionContext
from ..control_core.models.operator_intent_types import OperatorIntent
from ..control_core.models.runtime_bridge_types import DispatchResult, SchedulerDispatchEnvelope
from ..control_core.models.task_types import (
    HighLevelTaskRequest,
    SchedulerStepResult,
    TIP_FREE_GROWTH,
    TIP_TURN_AUTONOMOUS,
    TaskGraphSpec,
)
from ..control_core.orchestration.graph_factories import (
    build_runtime_demo_pair_graph,
    build_tip_free_growth_graph,
)
from ..control_core.orchestration.skill_registry import SkillRegistry, build_default_skill_registry
from ..control_core.orchestration.skill_scheduler import SkillScheduler
from ..control_core.orchestration.transition_policy_registry import (
    TransitionPolicyRegistry,
    build_default_transition_policy_registry,
)
from ..control_core.supervisor.operator_intent import normalize_operator_intent
from ..control_core.supervisor.intent_router import route_operator_intent
from ..control_core.supervisor.front_cooperation_plan import FrontCooperationPlan
from ..control_core.supervisor.turn_task_supervisor import compile_turn_autonomous_request
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
    high_level_task_kind: str | None = None
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
        self.topology = topology or ChainTopology(
            ordered_modules=["tip", "joint1", "joint2", "joint3", "joint4", "joint5"]
        )
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
        self.last_turn_plan: FrontCooperationPlan | None = None
        self.last_high_level_task_request: HighLevelTaskRequest | None = None
        self.last_operator_intent: OperatorIntent | None = None

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
        if graph_spec.metadata.get("graph_family") != "turn_autonomous":
            self.last_turn_plan = None
        self.scheduler.load_graph(graph_spec)

    def load_high_level_task(
        self,
        task_request: HighLevelTaskRequest,
        *,
        frame: RuntimeObservationFrame | None = None,
    ) -> TaskGraphSpec:
        """Compile and load one high-level request into the session scheduler."""
        self.last_high_level_task_request = task_request
        if task_request.task_kind == TIP_FREE_GROWTH:
            self.last_turn_plan = None
            graph_spec = build_tip_free_growth_graph(
                graph_id=str(task_request.metadata.get("graph_id", "tip_free_growth")),
                metadata={
                    "runtime_main_path": True,
                    "runtime_dispatch_mode": "scheduler_envelope",
                    "runtime_input_mode": "runtime_frame",
                    "operator_intent": str(task_request.metadata.get("operator_intent", "TIP_FREE_GROWTH")),
                    "operator_intent_kind": str(
                        task_request.metadata.get("operator_intent_kind", "TIP_FREE_GROWTH")
                    ),
                    "target_heading_delta_deg": task_request.metadata.get("target_heading_delta_deg"),
                    **dict(task_request.metadata),
                },
            )
            graph_spec.metadata.update(
                {
                    "operator_intent": str(task_request.metadata.get("operator_intent", "TIP_FREE_GROWTH")),
                    "operator_intent_kind": str(
                        task_request.metadata.get("operator_intent_kind", "TIP_FREE_GROWTH")
                    ),
                    "target_heading_delta_deg": task_request.metadata.get("target_heading_delta_deg"),
                    **dict(task_request.metadata),
                }
            )
            self.load_graph(graph_spec)
            return graph_spec
        if task_request.task_kind != TIP_TURN_AUTONOMOUS:
            raise ValueError(f"runtime_session_high_level_task_unsupported:{task_request.task_kind}")

        target_heading_delta_deg = _required_target_heading_delta(task_request.metadata)
        resolved_frame = frame or self._frame_for_high_level_task()
        if resolved_frame is None:
            raise ValueError("runtime_session_turn_request_frame_unavailable")
        graph_id = task_request.metadata.get("graph_id")
        resolved_graph_id = None if graph_id is None else str(graph_id)
        plan, graph_spec = compile_turn_autonomous_request(
            resolved_frame,
            target_heading_delta_deg=target_heading_delta_deg,
            graph_id=resolved_graph_id,
        )
        graph_spec.metadata.update(
            {
                "operator_intent": str(task_request.metadata.get("operator_intent", "TIP_TURN")),
                "operator_intent_kind": str(task_request.metadata.get("operator_intent_kind", "TIP_TURN")),
                "target_heading_delta_deg": float(target_heading_delta_deg),
                **dict(task_request.metadata),
            }
        )
        self.last_turn_plan = plan
        self.load_graph(graph_spec)
        return graph_spec

    def _extract_tip_motion_status(
        self,
        latest_frame: RuntimeObservationFrame | None,
        provider_status: dict[str, object] | None,
    ) -> tuple[float, float]:
        """
        Return (tip_extension_mm, tip_heading_deg).
        Priority:
        1) latest_frame.metadata
        2) provider_status["latest_backend_state"]
        3) fallback zeros
        """
        tip_extension_mm = 0.0
        tip_heading_deg = 0.0

        if latest_frame is not None:
            metadata = dict(latest_frame.metadata or {})
            tip_extension_mm = float(metadata.get("tip_extension_mm", tip_extension_mm) or tip_extension_mm)
            tip_heading_deg = float(metadata.get("tip_heading_deg", tip_heading_deg) or tip_heading_deg)

        if provider_status:
            backend_state = provider_status.get("latest_backend_state")
            if isinstance(backend_state, dict):
                tip_extension_mm = float(backend_state.get("tip_extension_mm", tip_extension_mm) or tip_extension_mm)
                tip_heading_deg = float(backend_state.get("tip_heading_deg", tip_heading_deg) or tip_heading_deg)

        return tip_extension_mm, tip_heading_deg

    def submit_intent(
        self,
        intent: OperatorIntent | Mapping[str, object] | str,
        *,
        frame: RuntimeObservationFrame | None = None,
        metadata: Mapping[str, object] | None = None,
        target_heading_delta_deg: float | None = None,
    ) -> TaskGraphSpec:
        """Route one operator intent into a high-level task and load it."""
        task_request = route_operator_intent(
            intent,
            metadata=metadata,
            target_heading_delta_deg=target_heading_delta_deg,
        )
        self.last_operator_intent = normalize_operator_intent(
            intent,
            target_heading_delta_deg=target_heading_delta_deg,
        )
        return self.load_high_level_task(task_request, frame=frame)

    def set_operator_intent(
        self,
        intent: OperatorIntent | Mapping[str, object] | str,
        *,
        frame: RuntimeObservationFrame | None = None,
        metadata: Mapping[str, object] | None = None,
        target_heading_delta_deg: float | None = None,
    ) -> TaskGraphSpec:
        """Alias for `submit_intent(...)` used by higher-level callers."""
        return self.submit_intent(
            intent,
            frame=frame,
            metadata=metadata,
            target_heading_delta_deg=target_heading_delta_deg,
        )

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

        scheduler_state = self.scheduler.state
        graph_metadata = dict(self.graph_spec.metadata) if self.graph_spec is not None else {}

        provider_kind = _provider_kind(self.observation_provider)
        dispatcher_kind = _dispatcher_kind(self.command_dispatcher)
        provider_status = _component_runtime_diagnostics(self.observation_provider)
        dispatcher_status = _component_runtime_diagnostics(self.command_dispatcher)

        step = self.last_step_result
        latest_frame = step.frame if step is not None else None

        if latest_frame is None:
            candidate_frame = provider_status.get("latest_frame")
            if isinstance(candidate_frame, RuntimeObservationFrame):
                latest_frame = candidate_frame

        tip_extension_mm, tip_heading_deg = self._extract_tip_motion_status(
            latest_frame=latest_frame,
            provider_status=provider_status if isinstance(provider_status, dict) else None,
        )

        step_diagnostics: dict[str, object] = {}
        dispatch_ready = False
        input_source = _runtime_input_source(provider_kind)
        dispatch_target = dispatcher_kind

        if step is not None:
            step_diagnostics = dict(step.diagnostics or {})
            input_source = str(step.input_source or input_source)
            dispatch_target = str(step.dispatch_target or dispatch_target)
            dispatch_ready = bool(
                step.dispatch_envelope.dispatch_ready
                if step.dispatch_envelope is not None
                else step.accepted
            )

        selected_joint_id = getattr(self.last_turn_plan, "selected_joint_id", None)
        if selected_joint_id is None:
            selected_joint_id = graph_metadata.get("selected_joint_id")

        operator_intent_kind = (
            self.last_operator_intent.intent_kind
            if self.last_operator_intent is not None
            else graph_metadata.get("operator_intent_kind")
        )
        target_heading_delta_deg = (
            self.last_operator_intent.target_heading_delta_deg
            if self.last_operator_intent is not None
            else graph_metadata.get("target_heading_delta_deg")
        )

        latest_step_summary = None
        if step is not None:
            latest_step_summary = {
                "accepted": bool(step.accepted),
                "reason": step.reason,
                "provider_kind": step.provider_kind,
                "dispatcher_kind": step.dispatcher_kind,
                "input_source": step.input_source,
                "dispatch_target": step.dispatch_target,
                "has_dispatch_envelope": step.dispatch_envelope is not None,
                "diagnostics": dict(step.diagnostics or {}),
            }

        return {
            "mode": self.session_name,
            "started": bool(self._started),
            "graph_id": self.graph_spec.graph_id if self.graph_spec is not None else None,
            "graph_metadata": graph_metadata,
            "scheduler_state": scheduler_state.as_dict(),
            "current_node_id": scheduler_state.current_node_id,
            "last_skill_key": scheduler_state.last_skill_key,
            "is_finished": bool(scheduler_state.is_finished),
            "provider_kind": provider_kind,
            "dispatcher_kind": dispatcher_kind,
            "provider_status": provider_status,
            "dispatcher_status": dispatcher_status,
            "input_source": input_source,
            "dispatch_target": dispatch_target,
            "dispatch_ready": dispatch_ready,
            "operator_intent_kind": operator_intent_kind,
            "target_heading_delta_deg": target_heading_delta_deg,
            "selected_joint_id": selected_joint_id,
            "tip_extension_mm": tip_extension_mm,
            "tip_heading_deg": tip_heading_deg,
            "last_turn_plan_available": self.last_turn_plan is not None,
            "last_high_level_task_kind": (
                self.last_high_level_task_request.task_kind
                if self.last_high_level_task_request is not None
                else None
            ),
            "last_high_level_task_metadata": (
                dict(self.last_high_level_task_request.metadata)
                if self.last_high_level_task_request is not None
                else {}
            ),
            "latest_step": latest_step_summary,
            "step_diagnostics": step_diagnostics,
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
                    high_level_task_kind=(
                        None if self.graph_spec is None else self.graph_spec.metadata.get("high_level_task_kind")
                    ),
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
                        "operator_intent": (
                            None if self.graph_spec is None else self.graph_spec.metadata.get("operator_intent")
                        ),
                        "operator_intent_kind": (
                            None if self.graph_spec is None else self.graph_spec.metadata.get("operator_intent_kind")
                        ),
                        "high_level_task_kind": (
                            None
                            if self.graph_spec is None
                            else self.graph_spec.metadata.get("high_level_task_kind")
                        ),
                        "target_heading_delta_deg": (
                            None
                            if self.graph_spec is None
                            else self.graph_spec.metadata.get("target_heading_delta_deg")
                        ),
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
            high_level_task_kind=scheduler_step_result.diagnostics.get("high_level_task_kind"),
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
                "operator_intent": scheduler_step_result.diagnostics.get("operator_intent"),
                "operator_intent_kind": scheduler_step_result.diagnostics.get("operator_intent_kind"),
                "high_level_task_kind": scheduler_step_result.diagnostics.get("high_level_task_kind"),
                "target_heading_delta_deg": scheduler_step_result.diagnostics.get("target_heading_delta_deg"),
                "tip_heading_target_deg": scheduler_step_result.diagnostics.get("tip_heading_target_deg"),
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
                "selected_joint_id": scheduler_step_result.diagnostics.get("selected_joint_id"),
                "selected_joint_index": scheduler_step_result.diagnostics.get("selected_joint_index"),
                "direct_front_cooperation": scheduler_step_result.diagnostics.get(
                    "direct_front_cooperation"
                ),
                "requires_recursive_transfer": scheduler_step_result.diagnostics.get(
                    "requires_recursive_transfer"
                ),
                "planner_mode": scheduler_step_result.diagnostics.get("planner_mode"),
                "current_plan_node_kind": scheduler_step_result.diagnostics.get(
                    "current_plan_node_kind"
                ),
                "current_active_pair": scheduler_step_result.diagnostics.get("current_active_pair"),
                "returning_to_tip_free_growth": scheduler_step_result.diagnostics.get(
                    "returning_to_tip_free_growth"
                ),
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

    def _frame_for_high_level_task(self) -> RuntimeObservationFrame | None:
        """Resolve a runtime frame for high-level task compilation."""
        if self._started:
            return self.observation_provider.get_latest_frame()
        self.observation_provider.start()
        try:
            warmed = self.observation_provider.warmup()
            if warmed is not None:
                return warmed
            return self.observation_provider.get_latest_frame()
        finally:
            self.observation_provider.stop()


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


def _required_target_heading_delta(metadata: Mapping[str, object]) -> float:
    """Return the validated target-heading delta for one turn request."""
    if "target_heading_delta_deg" not in metadata:
        raise ValueError("runtime_session_turn_request_requires_target_heading_delta_deg")
    value = metadata.get("target_heading_delta_deg")
    try:
        return float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError("runtime_session_turn_request_requires_numeric_target_heading_delta_deg") from exc


__all__ = [
    "RuntimeSession",
    "RuntimeStepResult",
    "build_ros2_runtime_session",
    "build_sim_runtime_session",
]

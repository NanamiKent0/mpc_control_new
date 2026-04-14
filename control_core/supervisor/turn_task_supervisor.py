"""High-level supervisor entrypoints for compiling turn requests."""

from __future__ import annotations

from ..controllers.turn_reference_mapper import module_heading_deg, tip_turn_heading_target
from ..estimation import build_estimate_bundle
from ..models.task_types import TaskGraphSpec
from ..orchestration.graph_factories import build_turn_autonomous_graph_from_plan
from ..orchestration.turn_plan_graph_adapter import TURN_WORKFLOW_ORDERED_MODULES
from ...runtime_integration.observation_types import RuntimeObservationFrame
from .front_cooperation_plan import FrontCooperationPlan
from .turn_planner import plan_turn_workflow

TURN_TASK_SUPERVISOR_SOURCE = "control_core.supervisor.turn_task_supervisor"


def compile_turn_autonomous_request(
    frame: RuntimeObservationFrame,
    *,
    target_heading_delta_deg: float,
    graph_id: str | None = None,
) -> tuple[FrontCooperationPlan, TaskGraphSpec]:
    """Compile one runtime-frame turn request into plan plus executable graph."""
    estimate_bundle = build_estimate_bundle(
        frame,
        ordered_modules=TURN_WORKFLOW_ORDERED_MODULES,
    )
    heading_target = tip_turn_heading_target(
        estimate_bundle.chain_snapshot,
        target_heading_delta_deg=float(target_heading_delta_deg),
    )
    tip_heading_current_deg = heading_target.get("tip_heading_current_deg")
    tip_heading_target_deg = heading_target.get("tip_heading_target_deg")
    plan = plan_turn_workflow(estimate_bundle)
    plan.diagnostics.update(
        {
            "operator_intent": "TIP_TURN",
            "operator_intent_kind": "TIP_TURN",
            "target_heading_delta_deg": float(target_heading_delta_deg),
            "tip_heading_current_deg": tip_heading_current_deg,
            "tip_heading_target_deg": tip_heading_target_deg,
            "joint1_heading_current_deg": module_heading_deg(estimate_bundle.chain_snapshot, "joint1"),
        }
    )
    graph_spec = build_turn_autonomous_graph_from_plan(
        plan,
        graph_id=graph_id,
        metadata={
            "runtime_input_mode": "runtime_frame",
            "runtime_dispatch_mode": "scheduler_envelope",
            "runtime_main_path": True,
            "operator_intent": "TIP_TURN",
            "operator_intent_kind": "TIP_TURN",
            "target_heading_delta_deg": float(target_heading_delta_deg),
            "tip_heading_current_deg": tip_heading_current_deg,
            "tip_heading_target_deg": tip_heading_target_deg,
            "turn_task_supervisor_source": TURN_TASK_SUPERVISOR_SOURCE,
            "estimate_bundle_source": estimate_bundle.diagnostics.get("estimate_bundle_source"),
            "estimate_bundle_timestamp_ns": estimate_bundle.timestamp_ns,
            "estimate_bundle_ordered_modules": list(
                estimate_bundle.diagnostics.get("ordered_modules", TURN_WORKFLOW_ORDERED_MODULES)
            ),
        },
    )
    return (plan, graph_spec)


__all__ = [
    "TURN_TASK_SUPERVISOR_SOURCE",
    "compile_turn_autonomous_request",
]

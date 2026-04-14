"""Reusable task-graph factories for pair-generic skill chains."""

from __future__ import annotations

from ..models.task_types import TaskGraphSpec
from ..supervisor.front_cooperation_plan import FrontCooperationPlan
from ..topology.relation_state import RelationType
from .graph_builder import TaskGraphBuilder
from .graph_fragments import (
    build_failure_sink_fragment,
    build_pair_approach_dock_fragment,
)
from .turn_plan_graph_adapter import (
    build_tip_free_growth_graph as _build_tip_free_growth_graph,
    build_turn_autonomous_graph_from_plan as _build_turn_autonomous_graph_from_plan,
)


def _shared_graph_metadata(
    *,
    template_name: str,
    active_module: str,
    passive_module: str,
    relation_type: RelationType,
    metadata: dict[str, object] | None,
) -> dict[str, object]:
    """Return graph-level metadata shared across all pair templates."""
    pair_label = f"{active_module}->{passive_module}"
    return {
        "graph_template": template_name,
        "pair": pair_label,
        "relation_type": relation_type,
        "default_transition_policy_key": "default",
        "transition_policy_key": "default",
        **dict(metadata or {}),
    }


def build_pair_approach_then_dock_graph(
    *,
    graph_id: str,
    active_module: str,
    passive_module: str,
    relation_type: RelationType,
    coarse_distance_threshold_mm: float,
    dock_distance_done_mm: float,
    dock_orientation_done_deg: float | None,
    metadata: dict[str, object] | None = None,
) -> TaskGraphSpec:
    """Build a reusable linear pair graph: coarse approach, then fine dock."""
    template_name = "pair_approach_then_dock"
    builder = TaskGraphBuilder(
        graph_id,
        metadata=_shared_graph_metadata(
            template_name=template_name,
            active_module=active_module,
            passive_module=passive_module,
            relation_type=relation_type,
            metadata=metadata,
        ),
    )
    builder.add_fragment(
        build_pair_approach_dock_fragment(
            active_module=active_module,
            passive_module=passive_module,
            relation_type=relation_type,
            coarse_distance_threshold_mm=coarse_distance_threshold_mm,
            dock_distance_done_mm=dock_distance_done_mm,
            dock_orientation_done_deg=dock_orientation_done_deg,
            template_name=template_name,
            coarse_metadata={"allow_off_frontier": False, "requires_support_stability": False},
            fine_metadata={"allow_off_frontier": False, "requires_support_stability": True},
        )
    )
    return builder.build()


def build_pair_approach_dock_with_failure_sink_graph(
    *,
    graph_id: str,
    active_module: str,
    passive_module: str,
    relation_type: RelationType,
    coarse_distance_threshold_mm: float,
    dock_distance_done_mm: float,
    dock_orientation_done_deg: float | None,
    metadata: dict[str, object] | None = None,
) -> TaskGraphSpec:
    """Build a pair graph whose working nodes fall back into a terminal failure sink."""
    template_name = "pair_approach_dock_with_failure_sink"
    builder = TaskGraphBuilder(
        graph_id,
        metadata=_shared_graph_metadata(
            template_name=template_name,
            active_module=active_module,
            passive_module=passive_module,
            relation_type=relation_type,
            metadata=metadata,
        ),
    )
    pair_fragment = builder.add_fragment(
        build_pair_approach_dock_fragment(
            active_module=active_module,
            passive_module=passive_module,
            relation_type=relation_type,
            coarse_distance_threshold_mm=coarse_distance_threshold_mm,
            dock_distance_done_mm=dock_distance_done_mm,
            dock_orientation_done_deg=dock_orientation_done_deg,
            template_name=template_name,
            coarse_metadata={
                "recovery": "failure_sink",
                "allow_off_frontier": False,
                "requires_support_stability": False,
            },
            fine_metadata={
                "recovery": "failure_sink",
                "allow_off_frontier": False,
                "requires_support_stability": True,
            },
        )
    )
    failure_fragment = builder.add_fragment(
        build_failure_sink_fragment(
            active_module=active_module,
            passive_module=passive_module,
            relation_type=relation_type,
            reason="pair_failure_sink",
        )
    )
    builder.link_failure(pair_fragment.start_node_id, failure_fragment.start_node_id)
    builder.link_failure("fine_dock", failure_fragment.start_node_id)
    return builder.build()


def build_retryable_pair_approach_then_dock_graph(
    *,
    graph_id: str,
    active_module: str,
    passive_module: str,
    relation_type: RelationType,
    coarse_distance_threshold_mm: float,
    dock_distance_done_mm: float,
    dock_orientation_done_deg: float | None,
    coarse_max_attempts: int = 2,
    failure_after_attempts: int | None = None,
    metadata: dict[str, object] | None = None,
) -> TaskGraphSpec:
    """Build a retryable pair graph with bounded attempts and a fallback sink."""
    template_name = "retryable_pair_approach_then_dock"
    graph_metadata = _shared_graph_metadata(
        template_name=template_name,
        active_module=active_module,
        passive_module=passive_module,
        relation_type=relation_type,
        metadata=metadata,
    )
    graph_metadata["default_transition_policy_key"] = "default"
    builder = TaskGraphBuilder(graph_id, metadata=graph_metadata)
    coarse_metadata: dict[str, object] = {
        "recovery": "failure_sink",
        "allow_off_frontier": False,
        "requires_support_stability": False,
    }
    if failure_after_attempts is not None:
        coarse_metadata["force_failure_after_n_attempts"] = failure_after_attempts
    pair_fragment = builder.add_fragment(
        build_pair_approach_dock_fragment(
            active_module=active_module,
            passive_module=passive_module,
            relation_type=relation_type,
            coarse_distance_threshold_mm=coarse_distance_threshold_mm,
            dock_distance_done_mm=dock_distance_done_mm,
            dock_orientation_done_deg=dock_orientation_done_deg,
            template_name=template_name,
            coarse_transition_policy_key="default",
            coarse_max_attempts=coarse_max_attempts,
            fine_transition_policy_key="default",
            fine_max_attempts=1,
            coarse_metadata=coarse_metadata,
            fine_metadata={
                "recovery": "failure_sink",
                "allow_off_frontier": False,
                "requires_support_stability": True,
            },
        )
    )
    failure_fragment = builder.add_fragment(
        build_failure_sink_fragment(
            active_module=active_module,
            passive_module=passive_module,
            relation_type=relation_type,
            reason="retry_exhausted_or_forced_failure",
        )
    )
    builder.link_failure(pair_fragment.start_node_id, failure_fragment.start_node_id)
    builder.link_failure("fine_dock", failure_fragment.start_node_id)
    return builder.build()


def build_pair_approach_dock_finalize_graph(
    *,
    graph_id: str,
    active_module: str,
    passive_module: str,
    relation_type: RelationType,
    coarse_distance_threshold_mm: float,
    dock_distance_done_mm: float,
    dock_orientation_done_deg: float | None,
    metadata: dict[str, object] | None = None,
) -> TaskGraphSpec:
    """Build a three-node pair graph with an explicit terminal finalize node."""
    template_name = "pair_approach_dock_finalize"
    builder = TaskGraphBuilder(
        graph_id,
        metadata=_shared_graph_metadata(
            template_name=template_name,
            active_module=active_module,
            passive_module=passive_module,
            relation_type=relation_type,
            metadata=metadata,
        ),
    )
    builder.add_fragment(
        build_pair_approach_dock_fragment(
            active_module=active_module,
            passive_module=passive_module,
            relation_type=relation_type,
            coarse_distance_threshold_mm=coarse_distance_threshold_mm,
            dock_distance_done_mm=dock_distance_done_mm,
            dock_orientation_done_deg=dock_orientation_done_deg,
            template_name=template_name,
            coarse_metadata={
                "allow_off_frontier": False,
                "requires_support_stability": False,
            },
            fine_metadata={
                "allow_off_frontier": False,
                "requires_support_stability": True,
            },
        )
    )
    finalize_fragment = builder.add_fragment(
        build_failure_sink_fragment(
            active_module=active_module,
            passive_module=passive_module,
            relation_type=relation_type,
            node_id="finalize_sink",
            reason="pair_finalize_terminal",
            metadata={"sink": "finalize"},
        )
    )
    builder.link_success("fine_dock", finalize_fragment.start_node_id)
    return builder.build()


def build_runtime_demo_pair_graph(
    *,
    graph_id: str,
    active_module: str = "joint1",
    passive_module: str = "tip",
    relation_type: RelationType = "tip_joint",
    coarse_distance_threshold_mm: float = 12.0,
    dock_distance_done_mm: float = 2.0,
    dock_orientation_done_deg: float | None = 5.0,
    include_finalize_node: bool = True,
    metadata: dict[str, object] | None = None,
) -> TaskGraphSpec:
    """Build the minimal runtime-session demo graph used by sim/live smoke tests."""
    runtime_metadata = {
        "graph_family": "runtime_demo",
        "runtime_main_path": True,
        "runtime_input_mode": "runtime_frame",
        "runtime_dispatch_mode": "scheduler_envelope",
        "runtime_gui_entry": "runtime_integration/gui/gui_ros2.py",
        "runtime_gui_semantic": "ros2_motor_topics",
        "runtime_supported_pairs": [
            {"active_module": "joint1", "passive_module": "tip", "relation_type": "tip_joint"},
            {"active_module": "joint2", "passive_module": "joint1", "relation_type": "joint_joint"},
        ],
        "runtime_dispatch_boundary": "self_contained_runtime_integration",
        **dict(metadata or {}),
    }
    if include_finalize_node:
        graph = build_pair_approach_dock_finalize_graph(
            graph_id=graph_id,
            active_module=active_module,
            passive_module=passive_module,
            relation_type=relation_type,
            coarse_distance_threshold_mm=coarse_distance_threshold_mm,
            dock_distance_done_mm=dock_distance_done_mm,
            dock_orientation_done_deg=dock_orientation_done_deg,
            metadata=runtime_metadata,
        )
    else:
        graph = build_pair_approach_then_dock_graph(
            graph_id=graph_id,
            active_module=active_module,
            passive_module=passive_module,
            relation_type=relation_type,
            coarse_distance_threshold_mm=coarse_distance_threshold_mm,
            dock_distance_done_mm=dock_distance_done_mm,
            dock_orientation_done_deg=dock_orientation_done_deg,
            metadata=runtime_metadata,
        )
    pair_label = f"{active_module}->{passive_module}"
    graph.metadata.update(
        {
            "graph_label": f"Runtime Demo {pair_label}",
            "graph_name": f"Runtime Demo {pair_label}",
            "graph_display_name": f"Runtime Demo {pair_label}",
            "requested_runtime_pair": {
                "active_module": active_module,
                "passive_module": passive_module,
                "relation_type": relation_type,
            },
            "node_labels": {
                node_id: _node_label(node_id=node_id, phase=node.metadata.get("phase"))
                for node_id, node in graph.nodes.items()
            },
            "skill_labels": {
                node_id: _skill_label(node.skill_spec.skill_key)
                for node_id, node in graph.nodes.items()
            },
            "pair_labels": {
                node_id: str(node.metadata.get("pair", pair_label))
                for node_id, node in graph.nodes.items()
            },
            "pair_catalog": [
                {
                    "node_id": node_id,
                    "pair_label": str(node.metadata.get("pair", pair_label)),
                    "skill_key": node.skill_spec.skill_key,
                    "phase": node.metadata.get("phase"),
                }
                for node_id, node in graph.nodes.items()
            ],
        }
    )
    return graph


def build_tip_free_growth_graph(
    *,
    graph_id: str = "tip_free_growth",
    metadata: dict[str, object] | None = None,
) -> TaskGraphSpec:
    """Build the minimal steady-state tip free-growth graph."""
    return _build_tip_free_growth_graph(
        graph_id=graph_id,
        metadata=metadata,
    )


def build_turn_autonomous_graph_from_plan(
    plan: FrontCooperationPlan,
    *,
    graph_id: str | None = None,
    metadata: dict[str, object] | None = None,
) -> TaskGraphSpec:
    """Compile one structured planner result into a scheduler graph."""
    return _build_turn_autonomous_graph_from_plan(
        plan,
        graph_id=graph_id,
        metadata=metadata,
    )


def _node_label(*, node_id: str, phase: object) -> str:
    """Build a GUI-friendly node label."""
    if isinstance(phase, str) and phase:
        return f"{phase}:{node_id}"
    return node_id.replace("_", " ")


def _skill_label(skill_key: str) -> str:
    """Build a GUI-friendly skill label."""
    return skill_key.replace("_", " ")

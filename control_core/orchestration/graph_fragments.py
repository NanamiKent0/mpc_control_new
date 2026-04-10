"""Reusable graph fragments for Phase-6 task-graph composition."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass, field

from ..models.skill_types import SkillSpec
from ..models.task_types import TaskNode
from ..topology.relation_state import RelationType


def _module_type(module_id: str) -> str:
    """Return the coarse module family used by skill metadata."""
    if module_id == "tip":
        return "tip"
    if module_id.startswith("joint"):
        return "joint"
    return "unknown"


def _shared_skill_metadata(
    *,
    template_name: str,
    active_module: str,
    passive_module: str,
) -> dict[str, object]:
    """Return metadata shared by pair-template skill specs."""
    pair_label = f"{active_module}->{passive_module}"
    return {
        "active_module_type": _module_type(active_module),
        "pair": pair_label,
        "graph_template": template_name,
    }


def _build_coarse_spec(
    *,
    template_name: str,
    active_module: str,
    passive_module: str,
    relation_type: RelationType,
    coarse_distance_threshold_mm: float,
) -> SkillSpec:
    """Build the coarse approach spec used by pair graph fragments."""
    return SkillSpec(
        skill_key="coarse_approach",
        active_module=active_module,
        passive_module=passive_module,
        relation_type=relation_type,
        distance_done_mm=float(coarse_distance_threshold_mm),
        limits={"crawl_mm_s": 4.0},
        config={"gain": 0.2, "deadband_mm": 0.5},
        metadata=_shared_skill_metadata(
            template_name=template_name,
            active_module=active_module,
            passive_module=passive_module,
        ),
    )


def _build_fine_spec(
    *,
    template_name: str,
    active_module: str,
    passive_module: str,
    relation_type: RelationType,
    dock_distance_done_mm: float,
    dock_orientation_done_deg: float | None,
) -> SkillSpec:
    """Build the fine docking spec used by pair graph fragments."""
    fine_params: dict[str, object] = {"distance_ref_mm": 0.0}
    if dock_orientation_done_deg is not None:
        fine_params["orientation_ref_deg"] = 0.0
    return SkillSpec(
        skill_key="fine_dock",
        active_module=active_module,
        passive_module=passive_module,
        relation_type=relation_type,
        distance_done_mm=float(dock_distance_done_mm),
        orientation_done_deg=dock_orientation_done_deg,
        limits={"crawl_mm_s": 4.0, "rotate_deg_s": 6.0},
        config={
            "distance_gain": 0.2,
            "orientation_gain": 0.5,
            "distance_deadband_mm": 0.5,
            "orientation_deadband_deg": 0.5,
        },
        params=fine_params,
        metadata=_shared_skill_metadata(
            template_name=template_name,
            active_module=active_module,
            passive_module=passive_module,
        ),
    )


def _build_terminal_spec(
    *,
    active_module: str,
    passive_module: str,
    relation_type: RelationType,
    reason: str,
) -> SkillSpec:
    """Build a minimal terminal sink spec for finalize or failure paths."""
    return SkillSpec(
        skill_key="terminal_noop",
        active_module=active_module,
        passive_module=passive_module,
        relation_type=relation_type,
        metadata={
            "terminal_reason": reason,
            "active_module_type": _module_type(active_module),
        },
    )


@dataclass(slots=True)
class GraphFragment:
    """Reusable fragment of task nodes with one declared entry point."""

    nodes: dict[str, TaskNode]
    start_node_id: str
    terminal_node_ids: list[str] = field(default_factory=list)
    metadata: dict[str, object] = field(default_factory=dict)

    def renamed(self, mapping: dict[str, str]) -> "GraphFragment":
        """Return a copy of this fragment with node identifiers remapped."""
        return GraphFragment(
            nodes={
                mapping.get(node_id, node_id): _clone_node(
                    node,
                    node_id=mapping.get(node_id, node_id),
                    rename_map=mapping,
                )
                for node_id, node in self.nodes.items()
            },
            start_node_id=mapping.get(self.start_node_id, self.start_node_id),
            terminal_node_ids=[mapping.get(node_id, node_id) for node_id in self.terminal_node_ids],
            metadata=dict(self.metadata),
        )

    def prefixed(self, prefix: str) -> "GraphFragment":
        """Return a copy of this fragment with a stable node-id prefix applied."""
        mapping = {node_id: f"{prefix}{node_id}" for node_id in self.nodes}
        return self.renamed(mapping)


def _clone_node(
    node: TaskNode,
    *,
    node_id: str,
    rename_map: dict[str, str],
) -> TaskNode:
    """Return a deep-copied node with identifiers rewritten."""
    cloned = deepcopy(node)
    cloned.node_id = node_id
    if cloned.next_node_id is not None:
        cloned.next_node_id = rename_map.get(cloned.next_node_id, cloned.next_node_id)
    if cloned.on_success_node_id is not None:
        cloned.on_success_node_id = rename_map.get(cloned.on_success_node_id, cloned.on_success_node_id)
    if cloned.on_failure_node_id is not None:
        cloned.on_failure_node_id = rename_map.get(cloned.on_failure_node_id, cloned.on_failure_node_id)
    return cloned


def build_pair_approach_dock_fragment(
    *,
    active_module: str,
    passive_module: str,
    relation_type: RelationType,
    coarse_distance_threshold_mm: float,
    dock_distance_done_mm: float,
    dock_orientation_done_deg: float | None,
    template_name: str = "pair_approach_dock_fragment",
    coarse_node_id: str = "coarse_approach",
    fine_node_id: str = "fine_dock",
    coarse_transition_policy_key: str | None = None,
    coarse_max_attempts: int = 1,
    fine_transition_policy_key: str | None = None,
    fine_max_attempts: int = 1,
    coarse_metadata: dict[str, object] | None = None,
    fine_metadata: dict[str, object] | None = None,
) -> GraphFragment:
    """Build a reusable two-node coarse-approach then fine-dock fragment."""
    pair_label = f"{active_module}->{passive_module}"
    coarse_node = TaskNode(
        node_id=coarse_node_id,
        skill_spec=_build_coarse_spec(
            template_name=template_name,
            active_module=active_module,
            passive_module=passive_module,
            relation_type=relation_type,
            coarse_distance_threshold_mm=coarse_distance_threshold_mm,
        ),
        next_node_id=fine_node_id,
        transition_policy_key=coarse_transition_policy_key,
        retry_limit=coarse_max_attempts,
        metadata={
            "phase": "approach",
            "pair": pair_label,
            "allow_off_frontier": False,
            "requires_support_stability": False,
            **dict(coarse_metadata or {}),
        },
    )
    fine_node = TaskNode(
        node_id=fine_node_id,
        skill_spec=_build_fine_spec(
            template_name=template_name,
            active_module=active_module,
            passive_module=passive_module,
            relation_type=relation_type,
            dock_distance_done_mm=dock_distance_done_mm,
            dock_orientation_done_deg=dock_orientation_done_deg,
        ),
        transition_policy_key=fine_transition_policy_key,
        retry_limit=fine_max_attempts,
        metadata={
            "phase": "dock",
            "pair": pair_label,
            "allow_off_frontier": False,
            "requires_support_stability": True,
            **dict(fine_metadata or {}),
        },
    )
    return GraphFragment(
        nodes={
            coarse_node.node_id: coarse_node,
            fine_node.node_id: fine_node,
        },
        start_node_id=coarse_node.node_id,
        terminal_node_ids=[fine_node.node_id],
        metadata={"fragment_template": template_name, "pair": pair_label},
    )


def build_failure_sink_fragment(
    *,
    active_module: str,
    passive_module: str,
    relation_type: RelationType,
    node_id: str = "failure_sink",
    reason: str = "pair_failure_sink",
    metadata: dict[str, object] | None = None,
) -> GraphFragment:
    """Build a terminal sink fragment backed by `TerminalNoopSkill`."""
    pair_label = f"{active_module}->{passive_module}"
    terminal_node = TaskNode(
        node_id=node_id,
        skill_spec=_build_terminal_spec(
            active_module=active_module,
            passive_module=passive_module,
            relation_type=relation_type,
            reason=reason,
        ),
        metadata={
            "phase": "terminal",
            "pair": pair_label,
            "sink": reason,
            **dict(metadata or {}),
        },
    )
    return GraphFragment(
        nodes={terminal_node.node_id: terminal_node},
        start_node_id=terminal_node.node_id,
        terminal_node_ids=[terminal_node.node_id],
        metadata={"fragment_template": "failure_sink_fragment", "pair": pair_label},
    )

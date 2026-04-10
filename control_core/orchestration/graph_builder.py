"""Helpers for constructing task graphs and reusable graph fragments."""

from __future__ import annotations

from copy import deepcopy

from ..models.task_types import TaskGraphSpec, TaskNode
from .graph_fragments import GraphFragment


class TaskGraphBuilder:
    """Imperative builder for Phase-6 task graphs and reusable fragments."""

    def __init__(
        self,
        graph_id: str,
        *,
        metadata: dict[str, object] | None = None,
    ) -> None:
        self.graph_id = graph_id
        self.metadata = dict(metadata or {})
        self._nodes: dict[str, TaskNode] = {}
        self._default_start_node_id: str | None = None
        self._fragment_counter = 0

    def add_node(self, node: TaskNode) -> "TaskGraphBuilder":
        """Add or replace one node in the builder."""
        self._nodes[node.node_id] = deepcopy(node)
        if self._default_start_node_id is None:
            self._default_start_node_id = node.node_id
        return self

    def add_fragment(
        self,
        fragment: GraphFragment,
        *,
        prefix: str | None = None,
    ) -> GraphFragment:
        """Add one fragment, applying a prefix when required to avoid id collisions."""
        builder_was_empty = not self._nodes
        resolved_fragment = fragment if prefix is None else fragment.prefixed(prefix)
        if self._has_conflict(resolved_fragment):
            resolved_fragment = self._resolve_fragment_conflicts(fragment, prefix=prefix)
        for node in resolved_fragment.nodes.values():
            self.add_node(node)
        if builder_was_empty:
            self._default_start_node_id = resolved_fragment.start_node_id
        return resolved_fragment

    def prefix_node_ids(self, prefix: str) -> dict[str, str]:
        """Prefix all currently registered builder node ids and return the rename map."""
        mapping = {node_id: f"{prefix}{node_id}" for node_id in self._nodes}
        self._nodes = {
            new_node_id: self._clone_with_mapping(node, new_node_id, mapping)
            for new_node_id, node in (
                (mapping[node_id], self._nodes[node_id]) for node_id in list(self._nodes)
            )
        }
        if self._default_start_node_id is not None:
            self._default_start_node_id = mapping[self._default_start_node_id]
        return mapping

    def link_linear(self, node_a: str, node_b: str) -> "TaskGraphBuilder":
        """Set the linear `next_node_id` edge from one node to another."""
        self._require_node(node_a).next_node_id = node_b
        return self

    def link_success(self, node_a: str, node_b: str) -> "TaskGraphBuilder":
        """Set the success edge from one node to another."""
        self._require_node(node_a).on_success_node_id = node_b
        return self

    def link_failure(self, node_a: str, node_b: str) -> "TaskGraphBuilder":
        """Set the failure edge from one node to another."""
        self._require_node(node_a).on_failure_node_id = node_b
        return self

    def build(self, start_node_id: str | None = None) -> TaskGraphSpec:
        """Build the immutable graph specification."""
        resolved_start_node_id = start_node_id or self._default_start_node_id
        if resolved_start_node_id is None:
            raise ValueError(f"task_graph_builder_missing_start_node:{self.graph_id}")
        return TaskGraphSpec(
            graph_id=self.graph_id,
            start_node_id=resolved_start_node_id,
            nodes=dict(self._nodes),
            metadata=dict(self.metadata),
        )

    def _require_node(self, node_id: str) -> TaskNode:
        """Return one builder-owned node or raise a clear error."""
        try:
            return self._nodes[node_id]
        except KeyError as exc:
            raise KeyError(f"task_graph_builder_node_missing:{node_id}") from exc

    def _has_conflict(self, fragment: GraphFragment) -> bool:
        """Return whether any fragment node ids already exist in the builder."""
        return any(node_id in self._nodes for node_id in fragment.nodes)

    def _resolve_fragment_conflicts(
        self,
        fragment: GraphFragment,
        *,
        prefix: str | None,
    ) -> GraphFragment:
        """Return a prefixed fragment whose node ids no longer collide."""
        base_prefix = prefix or "fragment_"
        while True:
            self._fragment_counter += 1
            candidate_prefix = f"{base_prefix}{self._fragment_counter}_"
            candidate = fragment.prefixed(candidate_prefix)
            if not self._has_conflict(candidate):
                return candidate

    @staticmethod
    def _clone_with_mapping(
        node: TaskNode,
        new_node_id: str,
        mapping: dict[str, str],
    ) -> TaskNode:
        """Clone one node and rewrite any internal edge references."""
        cloned = deepcopy(node)
        cloned.node_id = new_node_id
        if cloned.next_node_id is not None:
            cloned.next_node_id = mapping.get(cloned.next_node_id, cloned.next_node_id)
        if cloned.on_success_node_id is not None:
            cloned.on_success_node_id = mapping.get(cloned.on_success_node_id, cloned.on_success_node_id)
        if cloned.on_failure_node_id is not None:
            cloned.on_failure_node_id = mapping.get(cloned.on_failure_node_id, cloned.on_failure_node_id)
        return cloned

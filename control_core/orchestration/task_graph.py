"""Task-graph wrapper for the Phase-6 orchestration scaffold."""

from __future__ import annotations

from collections import deque

from ..models.task_types import TaskGraphSpec, TaskNode


class TaskGraph:
    """Validated view over a `TaskGraphSpec`."""

    def __init__(self, spec: TaskGraphSpec) -> None:
        self.spec = spec
        self.validate()

    def get_node(self, node_id: str) -> TaskNode:
        """Return one node or raise a clear missing-node error."""
        try:
            return self.spec.nodes[node_id]
        except KeyError as exc:
            raise KeyError(f"task_graph_node_missing:{node_id}") from exc

    def start_node(self) -> TaskNode:
        """Return the graph start node."""
        return self.get_node(self.spec.start_node_id)

    def next_node(self, current_node_id: str, success: bool = True) -> TaskNode | None:
        """Return the next node for one transition outcome."""
        current_node = self.get_node(current_node_id)
        if success:
            target_node_id = current_node.on_success_node_id or current_node.next_node_id
        else:
            target_node_id = current_node.on_failure_node_id
        if target_node_id is None:
            return None
        return self.get_node(target_node_id)

    def reachable_nodes_from_start(self) -> list[str]:
        """Return node identifiers reachable from the graph start node."""
        visited: set[str] = set()
        ordered: list[str] = []
        pending: deque[str] = deque([self.spec.start_node_id])
        while pending:
            node_id = pending.popleft()
            if node_id in visited:
                continue
            visited.add(node_id)
            ordered.append(node_id)
            node = self.get_node(node_id)
            for target_node_id in self._edge_targets(node):
                if target_node_id not in visited:
                    pending.append(target_node_id)
        return ordered

    def validate_unreachable_nodes(self) -> list[str]:
        """Return node identifiers that are not reachable from the start node."""
        reachable = set(self.reachable_nodes_from_start())
        return [node_id for node_id in self.spec.nodes if node_id not in reachable]

    def validate(self) -> None:
        """Validate graph connectivity and node-attempt semantics."""
        if not self.spec.nodes:
            raise ValueError(f"task_graph_has_no_nodes:{self.spec.graph_id}")
        if self.spec.start_node_id not in self.spec.nodes:
            raise ValueError(
                f"task_graph_start_node_missing:{self.spec.graph_id}:{self.spec.start_node_id}"
            )
        for node_id, node in self.spec.nodes.items():
            if node.node_id != node_id:
                raise ValueError(f"task_graph_node_id_mismatch:{node_id}!={node.node_id}")
            if node.max_attempts < 1:
                raise ValueError(
                    f"task_graph_invalid_max_attempts:{self.spec.graph_id}:{node_id}:{node.max_attempts}"
                )
            for edge_name, target_node_id in (
                ("next_node_id", node.next_node_id),
                ("on_success_node_id", node.on_success_node_id),
                ("on_failure_node_id", node.on_failure_node_id),
            ):
                if target_node_id is None:
                    continue
                if target_node_id not in self.spec.nodes:
                    raise ValueError(
                        f"task_graph_invalid_edge:{self.spec.graph_id}:{node_id}.{edge_name}->{target_node_id}"
                    )

    @staticmethod
    def _edge_targets(node: TaskNode) -> tuple[str, ...]:
        """Return all configured edge targets for one node."""
        return tuple(
            target_node_id
            for target_node_id in (
                node.next_node_id,
                node.on_success_node_id,
                node.on_failure_node_id,
            )
            if target_node_id is not None
        )

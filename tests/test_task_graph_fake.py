"""Fake tests for the Phase-4 task-graph datamodel and wrapper."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mpc_control_new.control_core.models.skill_types import SkillSpec
from mpc_control_new.control_core.models.task_types import TaskGraphSpec, TaskNode
from mpc_control_new.control_core.orchestration.task_graph import TaskGraph


def _spec(skill_key: str = "coarse_approach") -> SkillSpec:
    """Build a minimal skill spec for graph tests."""
    return SkillSpec(
        skill_key=skill_key,
        active_module="joint1",
        passive_module="tip",
        relation_type="tip_joint",
        distance_done_mm=10.0,
    )


class TaskGraphFakeTest(unittest.TestCase):
    """Verify minimal graph construction, validation, and traversal."""

    def test_linear_graph_supports_start_and_next_resolution(self) -> None:
        """A linear graph should resolve its start node and next node."""
        spec = TaskGraphSpec(
            graph_id="linear_graph",
            start_node_id="node1",
            nodes={
                "node1": TaskNode(node_id="node1", skill_spec=_spec("coarse_approach"), next_node_id="node2"),
                "node2": TaskNode(node_id="node2", skill_spec=_spec("fine_dock")),
            },
        )
        graph = TaskGraph(spec)
        self.assertEqual(graph.start_node().node_id, "node1")
        self.assertEqual(graph.next_node("node1").node_id, "node2")
        self.assertIsNone(graph.next_node("node2"))

    def test_success_and_failure_edges_are_both_supported(self) -> None:
        """The graph wrapper should expose success and failure branches."""
        spec = TaskGraphSpec(
            graph_id="branch_graph",
            start_node_id="node1",
            nodes={
                "node1": TaskNode(
                    node_id="node1",
                    skill_spec=_spec("coarse_approach"),
                    on_success_node_id="node2",
                    on_failure_node_id="node_fail",
                ),
                "node2": TaskNode(node_id="node2", skill_spec=_spec("fine_dock")),
                "node_fail": TaskNode(node_id="node_fail", skill_spec=_spec("coarse_approach")),
            },
        )
        graph = TaskGraph(spec)
        self.assertEqual(graph.next_node("node1", success=True).node_id, "node2")
        self.assertEqual(graph.next_node("node1", success=False).node_id, "node_fail")

    def test_missing_start_node_is_rejected(self) -> None:
        """Validation should fail if the start node is absent."""
        spec = TaskGraphSpec(
            graph_id="bad_start",
            start_node_id="missing",
            nodes={"node1": TaskNode(node_id="node1", skill_spec=_spec())},
        )
        with self.assertRaisesRegex(ValueError, "task_graph_start_node_missing"):
            TaskGraph(spec)

    def test_missing_edge_target_is_rejected(self) -> None:
        """Validation should fail if an edge points at a missing node."""
        spec = TaskGraphSpec(
            graph_id="bad_edge",
            start_node_id="node1",
            nodes={
                "node1": TaskNode(node_id="node1", skill_spec=_spec(), next_node_id="missing"),
            },
        )
        with self.assertRaisesRegex(ValueError, "task_graph_invalid_edge"):
            TaskGraph(spec)

    def test_unreachable_nodes_are_reported_without_failing_validation(self) -> None:
        """Graphs may keep disconnected nodes, and the wrapper should report them."""
        spec = TaskGraphSpec(
            graph_id="with_unreachable",
            start_node_id="node1",
            nodes={
                "node1": TaskNode(node_id="node1", skill_spec=_spec(), next_node_id="node2"),
                "node2": TaskNode(node_id="node2", skill_spec=_spec("fine_dock")),
                "orphan": TaskNode(node_id="orphan", skill_spec=_spec("terminal_noop")),
            },
        )
        graph = TaskGraph(spec)

        self.assertEqual(graph.reachable_nodes_from_start(), ["node1", "node2"])
        self.assertEqual(graph.validate_unreachable_nodes(), ["orphan"])


if __name__ == "__main__":
    unittest.main()

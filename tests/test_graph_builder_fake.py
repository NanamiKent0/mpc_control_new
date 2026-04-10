"""Fake tests for the lightweight task graph builder."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mpc_control_new.control_core.models.skill_types import SkillSpec
from mpc_control_new.control_core.models.task_types import TaskNode
from mpc_control_new.control_core.orchestration.graph_builder import TaskGraphBuilder
from mpc_control_new.control_core.orchestration.task_graph import TaskGraph


def _spec(skill_key: str) -> SkillSpec:
    """Build a minimal graph-builder skill spec."""
    return SkillSpec(
        skill_key=skill_key,
        active_module="joint1",
        passive_module="tip",
        relation_type="tip_joint",
        distance_done_mm=10.0,
    )


class GraphBuilderFakeTest(unittest.TestCase):
    """Verify the small task graph builder helper."""

    def test_builder_adds_nodes_and_links_edges(self) -> None:
        """Builder should support linear, success, and failure links."""
        builder = TaskGraphBuilder("builder_graph", metadata={"family": "fake"})
        builder.add_node(TaskNode(node_id="node1", skill_spec=_spec("coarse_approach")))
        builder.add_node(TaskNode(node_id="node2", skill_spec=_spec("fine_dock")))
        builder.add_node(TaskNode(node_id="node_fail", skill_spec=_spec("terminal_noop")))
        builder.link_linear("node1", "node2")
        builder.link_success("node1", "node2")
        builder.link_failure("node1", "node_fail")

        spec = builder.build("node1")
        graph = TaskGraph(spec)

        self.assertEqual(spec.metadata["family"], "fake")
        self.assertEqual(graph.get_node("node1").next_node_id, "node2")
        self.assertEqual(graph.get_node("node1").on_success_node_id, "node2")
        self.assertEqual(graph.get_node("node1").on_failure_node_id, "node_fail")
        self.assertEqual(graph.start_node().node_id, "node1")


if __name__ == "__main__":
    unittest.main()

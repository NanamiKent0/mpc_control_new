"""Fake tests for reusable graph fragments and fragment-aware graph building."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mpc_control_new.control_core.orchestration.graph_builder import TaskGraphBuilder
from mpc_control_new.control_core.orchestration.graph_fragments import (
    build_failure_sink_fragment,
    build_pair_approach_dock_fragment,
)
from mpc_control_new.control_core.orchestration.task_graph import TaskGraph


class GraphFragmentsFakeTest(unittest.TestCase):
    """Verify graph fragments can be reused and composed safely."""

    def test_fragment_can_be_added_twice_with_conflict_resolution(self) -> None:
        """Adding the same fragment twice should auto-prefix conflicting node ids."""
        fragment = build_pair_approach_dock_fragment(
            active_module="joint1",
            passive_module="tip",
            relation_type="tip_joint",
            coarse_distance_threshold_mm=12.0,
            dock_distance_done_mm=2.0,
            dock_orientation_done_deg=5.0,
        )
        failure_fragment = build_failure_sink_fragment(
            active_module="joint1",
            passive_module="tip",
            relation_type="tip_joint",
        )
        builder = TaskGraphBuilder("fragment_composition_graph")
        inserted_first = builder.add_fragment(fragment)
        inserted_second = builder.add_fragment(fragment)
        inserted_failure = builder.add_fragment(failure_fragment)
        builder.link_linear("fine_dock", inserted_second.start_node_id)
        builder.link_failure("fine_dock", inserted_failure.start_node_id)
        builder.link_failure(inserted_second.start_node_id, inserted_failure.start_node_id)

        spec = builder.build(inserted_first.start_node_id)
        graph = TaskGraph(spec)

        self.assertEqual(inserted_first.start_node_id, "coarse_approach")
        self.assertNotEqual(inserted_second.start_node_id, inserted_first.start_node_id)
        self.assertTrue(inserted_second.start_node_id.startswith("fragment_"))
        self.assertEqual(graph.get_node("fine_dock").next_node_id, inserted_second.start_node_id)
        self.assertIn(inserted_failure.start_node_id, graph.spec.nodes)

    def test_builder_can_prefix_existing_nodes(self) -> None:
        """Builder-level prefixing should rewrite node ids and internal edges together."""
        fragment = build_pair_approach_dock_fragment(
            active_module="joint2",
            passive_module="joint1",
            relation_type="joint_joint",
            coarse_distance_threshold_mm=20.0,
            dock_distance_done_mm=3.0,
            dock_orientation_done_deg=4.0,
        )
        builder = TaskGraphBuilder("prefixed_fragment_graph")
        builder.add_fragment(fragment)

        mapping = builder.prefix_node_ids("stack_")
        spec = builder.build()
        graph = TaskGraph(spec)

        self.assertEqual(mapping["coarse_approach"], "stack_coarse_approach")
        self.assertEqual(graph.start_node().node_id, "stack_coarse_approach")
        self.assertEqual(graph.get_node("stack_coarse_approach").next_node_id, "stack_fine_dock")


if __name__ == "__main__":
    unittest.main()

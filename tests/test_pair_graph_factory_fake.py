"""Fake tests for reusable pair task-graph factories."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mpc_control_new.control_core.orchestration.graph_factories import (
    build_pair_approach_dock_with_failure_sink_graph,
    build_pair_approach_then_dock_graph,
    build_retryable_pair_approach_then_dock_graph,
)


class PairGraphFactoryFakeTest(unittest.TestCase):
    """Verify one graph template can be reused across multiple relation pairs."""

    def test_factory_builds_joint1_tip_graph(self) -> None:
        """The factory should produce the expected two-node tip-joint chain."""
        graph = build_pair_approach_then_dock_graph(
            graph_id="joint1_tip_graph",
            active_module="joint1",
            passive_module="tip",
            relation_type="tip_joint",
            coarse_distance_threshold_mm=12.0,
            dock_distance_done_mm=2.0,
            dock_orientation_done_deg=5.0,
        )
        self.assertEqual(len(graph.nodes), 2)
        self.assertEqual(graph.nodes["coarse_approach"].skill_spec.skill_key, "coarse_approach")
        self.assertEqual(graph.nodes["fine_dock"].skill_spec.skill_key, "fine_dock")
        self.assertEqual(graph.nodes["coarse_approach"].skill_spec.distance_done_mm, 12.0)
        self.assertEqual(graph.nodes["fine_dock"].skill_spec.orientation_done_deg, 5.0)

    def test_factory_builds_joint2_joint1_graph(self) -> None:
        """The same template should work for a joint-joint pair without rewrites."""
        graph = build_pair_approach_then_dock_graph(
            graph_id="joint2_joint1_graph",
            active_module="joint2",
            passive_module="joint1",
            relation_type="joint_joint",
            coarse_distance_threshold_mm=20.0,
            dock_distance_done_mm=3.0,
            dock_orientation_done_deg=4.0,
            metadata={"family": "m5_m6"},
        )
        self.assertEqual(len(graph.nodes), 2)
        self.assertEqual(graph.nodes["coarse_approach"].skill_spec.active_module, "joint2")
        self.assertEqual(graph.nodes["coarse_approach"].skill_spec.passive_module, "joint1")
        self.assertEqual(graph.nodes["fine_dock"].skill_spec.relation_type, "joint_joint")
        self.assertEqual(graph.metadata["family"], "m5_m6")
        self.assertEqual(graph.nodes["fine_dock"].skill_spec.params["orientation_ref_deg"], 0.0)

    def test_factory_builds_failure_sink_template(self) -> None:
        """The failure-sink graph should route working nodes into a terminal sink."""
        graph = build_pair_approach_dock_with_failure_sink_graph(
            graph_id="joint1_tip_failure_sink_graph",
            active_module="joint1",
            passive_module="tip",
            relation_type="tip_joint",
            coarse_distance_threshold_mm=12.0,
            dock_distance_done_mm=2.0,
            dock_orientation_done_deg=5.0,
        )

        self.assertIn("failure_sink", graph.nodes)
        self.assertEqual(graph.nodes["failure_sink"].skill_spec.skill_key, "terminal_noop")
        self.assertEqual(graph.nodes["coarse_approach"].on_failure_node_id, "failure_sink")
        self.assertEqual(graph.nodes["fine_dock"].on_failure_node_id, "failure_sink")

    def test_factory_builds_retryable_template(self) -> None:
        """The retryable graph should carry max-attempt metadata and fallback edges."""
        graph = build_retryable_pair_approach_then_dock_graph(
            graph_id="joint1_tip_retryable_graph",
            active_module="joint1",
            passive_module="tip",
            relation_type="tip_joint",
            coarse_distance_threshold_mm=12.0,
            dock_distance_done_mm=2.0,
            dock_orientation_done_deg=5.0,
            coarse_max_attempts=3,
            failure_after_attempts=2,
        )

        self.assertEqual(graph.nodes["coarse_approach"].max_attempts, 3)
        self.assertEqual(graph.nodes["coarse_approach"].on_failure_node_id, "failure_sink")
        self.assertEqual(graph.nodes["coarse_approach"].metadata["force_failure_after_n_attempts"], 2)
        self.assertEqual(graph.nodes["failure_sink"].skill_spec.skill_key, "terminal_noop")


if __name__ == "__main__":
    unittest.main()

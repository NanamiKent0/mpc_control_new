"""Fake tests for compiling turn plans into executable task graphs."""

from __future__ import annotations

import unittest

from tests.turn_planner_fake_support import build_estimate_bundle

from mpc_control_new.control_core.models.task_types import TIP_TURN_AUTONOMOUS
from mpc_control_new.control_core.orchestration.graph_factories import (
    build_turn_autonomous_graph_from_plan,
)
from mpc_control_new.control_core.supervisor import plan_turn_workflow


class TurnPlanToGraphFakeTest(unittest.TestCase):
    """Verify planner outputs compile into scheduler-friendly graph specs."""

    def test_direct_plan_compiles_into_front_return_and_tip_free_growth(self) -> None:
        plan = plan_turn_workflow(build_estimate_bundle(idle_joint_ids={"joint1"}))

        graph = build_turn_autonomous_graph_from_plan(plan, graph_id="turn_plan_direct")

        self.assertEqual(graph.start_node_id, "front_cooperate_joint1_to_tip")
        self.assertEqual(
            list(graph.nodes),
            [
                "front_cooperate_joint1_to_tip",
                "return_to_tip_free_growth",
                "tip_free_growth",
            ],
        )
        self.assertEqual(
            [node.skill_spec.skill_key for node in graph.nodes.values()],
            ["front_cooperate", "return_to_free_growth", "tip_free_growth"],
        )
        self.assertEqual(graph.metadata["high_level_task_kind"], TIP_TURN_AUTONOMOUS)
        self.assertEqual(graph.metadata["planner_mode"], "direct")
        self.assertEqual(graph.metadata["selected_joint_id"], "joint1")
        self.assertEqual(graph.nodes["return_to_tip_free_growth"].metadata["current_active_pair"], None)
        self.assertTrue(graph.nodes["tip_free_growth"].metadata["returning_to_tip_free_growth"])

    def test_recursive_plan_compiles_full_transfer_chain_then_returns_to_free_growth(self) -> None:
        plan = plan_turn_workflow(build_estimate_bundle(idle_joint_ids={"joint5"}))

        graph = build_turn_autonomous_graph_from_plan(plan, graph_id="turn_plan_recursive")

        self.assertEqual(
            list(graph.nodes),
            [
                "local_transfer_joint5_to_joint4",
                "local_transfer_joint4_to_joint3",
                "local_transfer_joint3_to_joint2",
                "local_transfer_joint2_to_joint1",
                "front_cooperate_joint1_to_tip",
                "return_to_tip_free_growth",
                "tip_free_growth",
            ],
        )
        self.assertEqual(graph.metadata["planner_mode"], "recursive")
        self.assertEqual(graph.metadata["selected_joint_index"], 5)
        self.assertTrue(graph.metadata["requires_recursive_transfer"])
        self.assertEqual(
            graph.nodes["local_transfer_joint5_to_joint4"].metadata["current_active_pair"],
            "joint5->joint4",
        )
        self.assertEqual(
            graph.nodes["front_cooperate_joint1_to_tip"].metadata["current_plan_node_kind"],
            "front_cooperate",
        )
        self.assertEqual(
            graph.metadata["ordered_node_kinds"],
            [
                "local_transfer",
                "local_transfer",
                "local_transfer",
                "local_transfer",
                "front_cooperate",
                "return_to_free_growth",
                "tip_free_growth",
            ],
        )


if __name__ == "__main__":
    unittest.main()

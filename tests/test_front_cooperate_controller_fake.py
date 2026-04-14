"""Fake tests for the generic front cooperate controller family."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mpc_control_new.control_core.controllers.front_cooperate_controller import (
    FrontCooperateController,
)
from mpc_control_new.control_core.controllers.local_transfer_controller import (
    LocalTransferController,
)
from mpc_control_new.control_core.models.skill_types import SkillSpec
from mpc_control_new.control_core.topology.chain_topology import ChainTopology
from mpc_control_new.control_core.topology.relation_state import RelationState


def _build_topology() -> ChainTopology:
    """Build the topology used by the front cooperate fake tests."""
    return ChainTopology(ordered_modules=["tip", "joint1", "joint2", "joint3"])


def _build_spec(**params: object) -> SkillSpec:
    """Build a reusable front cooperate spec."""
    return SkillSpec(
        skill_key="front_cooperate",
        active_module="joint1",
        passive_module="tip",
        relation_type="tip_joint",
        distance_done_mm=2.0,
        orientation_done_deg=2.0,
        limits={
            "feed_mm_s": 4.0,
            "crawl_mm_s": 4.0,
            "rotate_deg_s": 6.0,
            "bend_deg_s": 5.0,
        },
        config={
            "gain": 0.2,
            "distance_gain": 0.2,
            "orientation_gain": 0.25,
            "bend_gain": 0.2,
        },
        params=dict(params),
    )


class FrontCooperateControllerFakeTest(unittest.TestCase):
    """Verify the generic front cooperation stage routing and reuse."""

    def test_joint1_release_adjust_uses_posture_adjust_template(self) -> None:
        """Front release should reuse the generic posture adjust template."""
        controller = FrontCooperateController(topology=_build_topology())
        relation_state = RelationState(
            active_module="joint1",
            passive_module="tip",
            relation_type="tip_joint",
            distance_mm=22.0,
            orientation_error_deg=12.0,
            coupled=False,
            observation_valid=True,
            diagnostics={"bend_error_deg": -6.0},
        )
        result = controller.control(
            relation_state,
            _build_spec(
                release_complete=False,
                release_orientation_ref_deg=0.0,
                release_bend_ref_deg=0.0,
            ),
        )
        self.assertEqual(result.stage, "joint1_release_adjust")
        self.assertEqual(result.selected_mpc, "posture_adjust_mpc")
        self.assertEqual(result.family, "front_cooperate")

    def test_joint1_adjust_is_separate_from_release(self) -> None:
        """Front adjustment should remain a stage, not a new joint1-specific mode."""
        controller = FrontCooperateController(topology=_build_topology())
        relation_state = RelationState(
            active_module="joint1",
            passive_module="tip",
            relation_type="tip_joint",
            distance_mm=18.0,
            orientation_error_deg=8.0,
            coupled=False,
            observation_valid=True,
            diagnostics={"bend_error_deg": 5.0},
        )
        result = controller.control(
            relation_state,
            _build_spec(
                release_complete=True,
                posture_complete=False,
                posture_orientation_ref_deg=0.0,
                posture_bend_ref_deg=0.0,
            ),
        )
        self.assertEqual(result.stage, "joint1_adjust")
        self.assertEqual(result.selected_mpc, "posture_adjust_mpc")

    def test_joint1_approach_tip_reuses_same_generic_coarse_controller_as_local_family(self) -> None:
        """Front cooperation and local transfer should share the same approach controller class."""
        topology = _build_topology()
        front_controller = FrontCooperateController(topology=topology)
        local_controller = LocalTransferController(topology=topology)
        relation_state = RelationState(
            active_module="joint1",
            passive_module="tip",
            relation_type="tip_joint",
            distance_mm=24.0,
            orientation_error_deg=3.0,
            coupled=False,
            observation_valid=True,
            diagnostics={"bend_error_deg": 1.0},
        )
        result = front_controller.control(
            relation_state,
            _build_spec(release_complete=True, posture_complete=True, dock_trigger_distance_mm=10.0),
        )
        primitive_names = {reference.primitive_name for reference in result.primitive_references}
        self.assertIs(front_controller.coarse_controller.__class__, local_controller.coarse_controller.__class__)
        self.assertEqual(result.stage, "joint1_approach_tip")
        self.assertEqual(result.selected_controller, "coarse_approach_controller")
        self.assertIn("joint_crawl", primitive_names)

    def test_joint1_dock_tip_reuses_generic_fine_dock_controller(self) -> None:
        """Front docking should be delegated to the generic pair docking controller."""
        controller = FrontCooperateController(topology=_build_topology())
        relation_state = RelationState(
            active_module="joint1",
            passive_module="tip",
            relation_type="tip_joint",
            distance_mm=4.0,
            orientation_error_deg=-7.0,
            coupled=False,
            observation_valid=True,
            diagnostics={"bend_error_deg": 2.0},
        )
        result = controller.control(
            relation_state,
            _build_spec(release_complete=True, posture_complete=True, dock_trigger_distance_mm=10.0),
        )
        self.assertEqual(result.stage, "joint1_dock_tip")
        self.assertEqual(result.selected_controller, "fine_dock_controller")
        self.assertEqual(result.selected_mpc, "pair_dock_mpc")

    def test_joint1_cooperate_then_becomes_returnable_to_free_growth(self) -> None:
        """Completion should mark the pair as returnable to free growth."""
        controller = FrontCooperateController(topology=_build_topology())
        active_relation = RelationState(
            active_module="joint1",
            passive_module="tip",
            relation_type="tip_joint",
            distance_mm=1.0,
            orientation_error_deg=2.0,
            coupled=True,
            observation_valid=True,
            diagnostics={"bend_error_deg": -1.0, "cooperate_progress": 0.3},
        )
        active_result = controller.control(
            active_relation,
            _build_spec(
                release_complete=True,
                posture_complete=True,
                cooperate_feed_velocity_mm_s=3.0,
                cooperate_target_progress=1.0,
            ),
        )
        done_relation = RelationState(
            active_module="joint1",
            passive_module="tip",
            relation_type="tip_joint",
            distance_mm=0.5,
            orientation_error_deg=0.5,
            coupled=True,
            observation_valid=True,
            diagnostics={"bend_error_deg": 0.2, "cooperate_progress": 1.0},
        )
        done_result = controller.control(
            done_relation,
            _build_spec(
                release_complete=True,
                posture_complete=True,
                cooperate_target_progress=1.0,
            ),
        )
        self.assertEqual(active_result.stage, "joint1_cooperate_with_tip")
        self.assertEqual(active_result.selected_mpc, "cooperate_mpc")
        self.assertEqual(done_result.stage, "returnable_to_free_growth")
        self.assertEqual(done_result.status, "done")
        self.assertTrue(done_result.returnable_to_free_growth)


if __name__ == "__main__":
    unittest.main()

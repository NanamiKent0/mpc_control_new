"""Fake tests for the generic local transfer controller family."""

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
    """Build the topology used by the local transfer fake tests."""
    return ChainTopology(ordered_modules=["tip", "joint1", "joint2", "joint3"])


def _build_spec(**params: object) -> SkillSpec:
    """Build a reusable local transfer spec."""
    return SkillSpec(
        skill_key="local_transfer",
        active_module="joint2",
        passive_module="joint1",
        relation_type="joint_joint",
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


class LocalTransferControllerFakeTest(unittest.TestCase):
    """Verify the generic local transfer stage routing and template reuse."""

    def test_active_joint_release_uses_posture_adjust_template(self) -> None:
        """Release should be routed through the generic posture adjust MPC."""
        controller = LocalTransferController(topology=_build_topology())
        relation_state = RelationState(
            active_module="joint2",
            passive_module="joint1",
            relation_type="joint_joint",
            distance_mm=20.0,
            orientation_error_deg=15.0,
            coupled=False,
            observation_valid=True,
            diagnostics={"bend_error_deg": -8.0},
        )
        result = controller.control(
            relation_state,
            _build_spec(
                release_complete=False,
                release_orientation_ref_deg=0.0,
                release_bend_ref_deg=0.0,
            ),
        )
        primitive_names = {reference.primitive_name for reference in result.primitive_references}
        self.assertEqual(result.stage, "active_joint_release")
        self.assertEqual(result.selected_mpc, "posture_adjust_mpc")
        self.assertIn("joint_rotate", primitive_names)
        self.assertIn("joint_bend", primitive_names)

    def test_local_posture_adjust_has_its_own_stage_without_new_joint_mode(self) -> None:
        """Posture adjustment should be stage-driven, not joint-index driven."""
        controller = LocalTransferController(topology=_build_topology())
        relation_state = RelationState(
            active_module="joint3",
            passive_module="joint2",
            relation_type="joint_joint",
            distance_mm=18.0,
            orientation_error_deg=9.0,
            coupled=False,
            observation_valid=True,
            diagnostics={"bend_error_deg": 6.0},
        )
        spec = SkillSpec(
            skill_key="local_transfer",
            active_module="joint3",
            passive_module="joint2",
            relation_type="joint_joint",
            distance_done_mm=2.0,
            orientation_done_deg=2.0,
            limits={"feed_mm_s": 4.0, "rotate_deg_s": 6.0, "bend_deg_s": 5.0},
            config={"orientation_gain": 0.25, "bend_gain": 0.2},
            params={
                "release_complete": True,
                "posture_complete": False,
                "posture_orientation_ref_deg": 0.0,
                "posture_bend_ref_deg": 0.0,
            },
        )
        result = controller.control(relation_state, spec)
        self.assertEqual(result.stage, "local_posture_adjust")
        self.assertEqual(result.selected_mpc, "posture_adjust_mpc")
        self.assertEqual(result.family, "local_transfer")

    def test_approach_stage_reuses_same_generic_coarse_controller_as_front_family(self) -> None:
        """Local transfer and front cooperation should share the same approach template."""
        topology = _build_topology()
        local_controller = LocalTransferController(topology=topology)
        front_controller = FrontCooperateController(topology=topology)
        relation_state = RelationState(
            active_module="joint2",
            passive_module="joint1",
            relation_type="joint_joint",
            distance_mm=30.0,
            orientation_error_deg=5.0,
            coupled=False,
            observation_valid=True,
            diagnostics={"bend_error_deg": 1.0},
        )
        result = local_controller.control(
            relation_state,
            _build_spec(release_complete=True, posture_complete=True, dock_trigger_distance_mm=12.0),
        )
        primitive_names = {reference.primitive_name for reference in result.primitive_references}
        self.assertIs(local_controller.coarse_controller.__class__, front_controller.coarse_controller.__class__)
        self.assertEqual(result.stage, "approach_passive_joint")
        self.assertEqual(result.selected_controller, "coarse_approach_controller")
        self.assertEqual(result.selected_mpc, "scalar_feed_mpc")
        self.assertIn("joint_crawl", primitive_names)

    def test_dock_stage_reuses_generic_fine_dock_controller(self) -> None:
        """Near-field docking should be delegated to the generic pair dock template."""
        controller = LocalTransferController(topology=_build_topology())
        relation_state = RelationState(
            active_module="joint2",
            passive_module="joint1",
            relation_type="joint_joint",
            distance_mm=5.0,
            orientation_error_deg=-10.0,
            coupled=False,
            observation_valid=True,
            diagnostics={"bend_error_deg": 2.0},
        )
        result = controller.control(
            relation_state,
            _build_spec(release_complete=True, posture_complete=True, dock_trigger_distance_mm=12.0),
        )
        primitive_names = {reference.primitive_name for reference in result.primitive_references}
        self.assertEqual(result.stage, "dock_with_passive_joint")
        self.assertEqual(result.selected_controller, "fine_dock_controller")
        self.assertEqual(result.selected_mpc, "pair_dock_mpc")
        self.assertIn("joint_rotate", primitive_names)

    def test_cooperate_stage_uses_generic_cooperate_template(self) -> None:
        """Coupled local transfer should reuse the generic cooperation MPC."""
        controller = LocalTransferController(topology=_build_topology())
        relation_state = RelationState(
            active_module="joint2",
            passive_module="joint1",
            relation_type="joint_joint",
            distance_mm=1.0,
            orientation_error_deg=4.0,
            coupled=True,
            observation_valid=True,
            diagnostics={"bend_error_deg": -3.0, "cooperate_progress": 0.4},
        )
        result = controller.control(
            relation_state,
            _build_spec(
                release_complete=True,
                posture_complete=True,
                cooperate_feed_velocity_mm_s=3.0,
                cooperate_target_progress=1.0,
            ),
        )
        primitive_names = {reference.primitive_name for reference in result.primitive_references}
        self.assertEqual(result.stage, "cooperate_with_passive_joint")
        self.assertEqual(result.selected_mpc, "cooperate_mpc")
        self.assertIn("joint_crawl", primitive_names)
        self.assertIn("joint_bend", primitive_names)


if __name__ == "__main__":
    unittest.main()

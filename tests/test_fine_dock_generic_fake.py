"""Fake tests for the Phase-3 generic fine docking skill surface."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mpc_control_new.control_core.models.skill_types import SkillSpec
from mpc_control_new.control_core.skills.fine_dock import FineDockSkill
from mpc_control_new.control_core.topology.chain_topology import ChainTopology
from mpc_control_new.control_core.topology.relation_state import RelationState


def _build_topology() -> ChainTopology:
    """Build the topology used by the generic fine docking tests."""
    return ChainTopology(ordered_modules=["tip", "joint1", "joint2"])


class FineDockGenericFakeTest(unittest.TestCase):
    """Verify the Phase-3 pair-generic fine docking behavior."""

    def test_joint_led_fine_dock_still_works(self) -> None:
        """Joint-led fine docking should still emit crawl and rotate closure."""
        skill = FineDockSkill(topology=_build_topology())
        spec = SkillSpec(
            skill_key="fine_dock",
            active_module="joint1",
            passive_module="tip",
            relation_type="tip_joint",
            distance_done_mm=3.0,
            orientation_done_deg=5.0,
            limits={"crawl_mm_s": 4.0, "rotate_deg_s": 6.0},
            config={"distance_gain": 0.2, "orientation_gain": 0.5},
        )
        relation_state = RelationState(
            active_module="joint1",
            passive_module="tip",
            relation_type="tip_joint",
            distance_mm=18.0,
            orientation_error_deg=-12.0,
            coupled=False,
            observation_valid=True,
        )
        result = skill.execute(relation_state, spec)
        primitive_names = {reference.primitive_name for reference in result.primitive_references}
        self.assertEqual(result.status, "active")
        self.assertIn("joint_crawl", primitive_names)
        self.assertIn("joint_rotate", primitive_names)
        self.assertEqual(result.diagnostics["active_module_type"], "joint")
        self.assertEqual(result.diagnostics["supported_axes"], "distance,orientation")

    def test_tip_led_orientation_request_blocks_cleanly(self) -> None:
        """Tip-led fine docking should explicitly block unsupported orientation closure."""
        skill = FineDockSkill(topology=_build_topology())
        spec = SkillSpec(
            skill_key="fine_dock",
            active_module="tip",
            passive_module="joint1",
            relation_type="tip_joint",
            distance_done_mm=3.0,
            orientation_done_deg=5.0,
            params={"required_axes": ["distance", "orientation"]},
            limits={"growth_mm_s": 4.0},
            config={"distance_gain": 0.2},
        )
        relation_state = RelationState(
            active_module="tip",
            passive_module="joint1",
            relation_type="tip_joint",
            distance_mm=12.0,
            orientation_error_deg=8.0,
            coupled=False,
            observation_valid=True,
        )
        result = skill.execute(relation_state, spec)
        self.assertEqual(result.status, "blocked")
        self.assertTrue(result.not_fully_supported)
        self.assertEqual(result.block_reason, "unsupported_required_axes:orientation")
        self.assertEqual(result.diagnostics["unsupported_axes"], "orientation")
        self.assertFalse(result.diagnostics["orientation_control_supported"])
        self.assertEqual(result.primitive_references, [])

    def test_parameterized_refs_replace_hardcoded_zero_targets(self) -> None:
        """Fine docking should use spec parameter targets instead of hardcoded zeros."""
        skill = FineDockSkill(topology=_build_topology())
        spec = SkillSpec(
            skill_key="fine_dock",
            active_module="joint1",
            passive_module="tip",
            relation_type="tip_joint",
            distance_done_mm=1.0,
            orientation_done_deg=2.0,
            params={
                "distance_ref_mm": 4.0,
                "orientation_ref_deg": 10.0,
                "required_axes": ["distance", "orientation"],
            },
            limits={"crawl_mm_s": 4.0, "rotate_deg_s": 6.0},
            config={"distance_gain": 0.2, "orientation_gain": 0.5},
        )
        relation_state = RelationState(
            active_module="joint1",
            passive_module="tip",
            relation_type="tip_joint",
            distance_mm=14.0,
            orientation_error_deg=20.0,
            coupled=False,
            observation_valid=True,
        )
        result = skill.execute(relation_state, spec)
        crawl_reference = next(reference for reference in result.primitive_references if reference.axis == "crawl")
        rotate_reference = next(reference for reference in result.primitive_references if reference.axis == "rotate")
        self.assertEqual(crawl_reference.target_value, 4.0)
        self.assertEqual(rotate_reference.target_value, 10.0)
        self.assertEqual(result.diagnostics["distance_ref_mm"], 4.0)
        self.assertEqual(result.diagnostics["orientation_ref_deg"], 10.0)

    def test_already_coupled_legal_pair_keeps_zero_hold_completion(self) -> None:
        """A legal already-coupled pair should complete with explicit zero holds."""
        skill = FineDockSkill(topology=_build_topology())
        spec = SkillSpec(
            skill_key="fine_dock",
            active_module="joint1",
            passive_module="tip",
            relation_type="tip_joint",
            distance_done_mm=3.0,
            orientation_done_deg=5.0,
            limits={"crawl_mm_s": 4.0, "rotate_deg_s": 6.0},
        )
        relation_state = RelationState(
            active_module="joint1",
            passive_module="tip",
            relation_type="tip_joint",
            distance_mm=40.0,
            orientation_error_deg=20.0,
            coupled=True,
            observation_valid=True,
        )
        result = skill.execute(relation_state, spec)
        primary_references = [reference for reference in result.primitive_references if reference.primary]
        self.assertEqual(result.status, "done")
        self.assertTrue(result.completion.done)
        self.assertTrue(all(reference.reference_value == 0.0 for reference in primary_references))
        self.assertTrue(all(reference.semantic == "zero_hold" for reference in primary_references))


if __name__ == "__main__":
    unittest.main()

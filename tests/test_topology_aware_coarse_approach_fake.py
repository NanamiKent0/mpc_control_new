"""Fake tests for topology-aware coarse approach behavior."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mpc_control_new.control_core.models.skill_types import SkillSpec
from mpc_control_new.control_core.skills.coarse_approach import CoarseApproachSkill
from mpc_control_new.control_core.topology.chain_topology import ChainTopology
from mpc_control_new.control_core.topology.relation_state import RelationState


def _build_topology() -> ChainTopology:
    """Build the topology used by the topology-aware fake tests."""
    return ChainTopology(ordered_modules=["tip", "joint1", "joint2"])


class TopologyAwareCoarseApproachFakeTest(unittest.TestCase):
    """Verify coarse approach topology gating and primitive family selection."""

    def test_legal_adjacent_pair_allows_coarse_approach(self) -> None:
        """An adjacent pair should remain executable under topology gating."""
        skill = CoarseApproachSkill(topology=_build_topology())
        spec = SkillSpec(
            skill_key="coarse_approach",
            active_module="joint2",
            passive_module="joint1",
            relation_type="joint_joint",
            distance_done_mm=180.0,
            limits={"crawl_mm_s": 5.0},
            config={"gain": 0.2},
        )
        relation_state = RelationState(
            active_module="joint2",
            passive_module="joint1",
            relation_type="joint_joint",
            distance_mm=260.0,
            orientation_error_deg=None,
            coupled=False,
            observation_valid=True,
        )
        result = skill.execute(relation_state, spec)
        self.assertEqual(result.status, "active")
        self.assertTrue(result.preconditions.passed)
        self.assertFalse(result.blocked_by_topology)

    def test_illegal_non_adjacent_pair_blocks_coarse_approach(self) -> None:
        """A non-adjacent pair should be blocked before any references are emitted."""
        skill = CoarseApproachSkill(topology=_build_topology())
        spec = SkillSpec(
            skill_key="coarse_approach",
            active_module="tip",
            passive_module="joint2",
            relation_type="tip_joint",
            distance_done_mm=180.0,
        )
        relation_state = RelationState(
            active_module="tip",
            passive_module="joint2",
            relation_type="tip_joint",
            distance_mm=260.0,
            orientation_error_deg=None,
            coupled=False,
            observation_valid=True,
        )
        result = skill.execute(relation_state, spec)
        self.assertEqual(result.status, "blocked")
        self.assertTrue(result.preconditions.blocked_by_topology)
        self.assertEqual(result.block_reason, "non_adjacent_pair:tip->joint2")
        self.assertEqual(result.primitive_references, [])

    def test_tip_led_coarse_approach_emits_tip_growth(self) -> None:
        """A tip-led coarse approach should emit the tip-growth primitive family."""
        skill = CoarseApproachSkill(topology=_build_topology())
        spec = SkillSpec(
            skill_key="coarse_approach",
            active_module="tip",
            passive_module="joint1",
            relation_type="tip_joint",
            distance_done_mm=180.0,
            limits={"crawl_mm_s": 5.0},
            config={"gain": 0.2},
        )
        relation_state = RelationState(
            active_module="tip",
            passive_module="joint1",
            relation_type="tip_joint",
            distance_mm=260.0,
            orientation_error_deg=None,
            coupled=False,
            observation_valid=True,
        )
        result = skill.execute(relation_state, spec)
        primary_reference = next(ref for ref in result.primitive_references if ref.primary)
        self.assertEqual(primary_reference.primitive_name, "tip_growth")
        self.assertEqual(primary_reference.axis, "growth")

    def test_joint_led_coarse_approach_emits_joint_crawl(self) -> None:
        """A joint-led coarse approach should emit the joint-crawl primitive family."""
        skill = CoarseApproachSkill(topology=_build_topology())
        spec = SkillSpec(
            skill_key="coarse_approach",
            active_module="joint1",
            passive_module="tip",
            relation_type="tip_joint",
            distance_done_mm=180.0,
            limits={"crawl_mm_s": 5.0},
            config={"gain": 0.2},
        )
        relation_state = RelationState(
            active_module="joint1",
            passive_module="tip",
            relation_type="tip_joint",
            distance_mm=260.0,
            orientation_error_deg=None,
            coupled=False,
            observation_valid=True,
        )
        result = skill.execute(relation_state, spec)
        primary_reference = next(ref for ref in result.primitive_references if ref.primary)
        self.assertEqual(primary_reference.primitive_name, "joint_crawl")
        self.assertEqual(primary_reference.axis, "crawl")


if __name__ == "__main__":
    unittest.main()

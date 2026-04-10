"""Fake tests for topology-aware fine docking behavior."""

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
    """Build the topology used by the topology-aware fake tests."""
    return ChainTopology(ordered_modules=["tip", "joint1", "joint2"])


def _build_spec(*, active_module: str, passive_module: str, relation_type: str) -> SkillSpec:
    """Build a reusable fine dock spec for topology-aware tests."""
    return SkillSpec(
        skill_key="fine_dock",
        active_module=active_module,
        passive_module=passive_module,
        relation_type=relation_type,
        distance_done_mm=3.0,
        orientation_done_deg=5.0,
        limits={"crawl_mm_s": 4.0, "rotate_deg_s": 6.0},
        config={
            "distance_gain": 0.2,
            "orientation_gain": 0.5,
            "distance_deadband_mm": 0.5,
            "orientation_deadband_deg": 0.5,
        },
    )


class TopologyAwareFineDockFakeTest(unittest.TestCase):
    """Verify fine docking topology gating and zero-hold completion semantics."""

    def test_legal_adjacent_pair_allows_fine_dock_references(self) -> None:
        """An adjacent pair should allow active fine docking references."""
        skill = FineDockSkill(topology=_build_topology())
        relation_state = RelationState(
            active_module="joint1",
            passive_module="tip",
            relation_type="tip_joint",
            distance_mm=18.0,
            orientation_error_deg=-12.0,
            coupled=False,
            observation_valid=True,
        )
        result = skill.execute(
            relation_state,
            _build_spec(active_module="joint1", passive_module="tip", relation_type="tip_joint"),
        )
        self.assertEqual(result.status, "active")
        self.assertGreaterEqual(len(result.primitive_references), 2)
        self.assertFalse(result.blocked_by_topology)

    def test_illegal_non_adjacent_pair_blocks_fine_dock(self) -> None:
        """A non-adjacent pair should be blocked before fine docking starts."""
        skill = FineDockSkill(topology=_build_topology())
        relation_state = RelationState(
            active_module="joint2",
            passive_module="tip",
            relation_type="tip_joint",
            distance_mm=18.0,
            orientation_error_deg=-12.0,
            coupled=False,
            observation_valid=True,
        )
        result = skill.execute(
            relation_state,
            _build_spec(active_module="joint2", passive_module="tip", relation_type="tip_joint"),
        )
        self.assertEqual(result.status, "blocked")
        self.assertTrue(result.preconditions.blocked_by_topology)
        self.assertEqual(result.block_reason, "non_adjacent_pair:joint2->tip")

    def test_already_docked_state_keeps_zero_hold_when_pair_is_legal(self) -> None:
        """A legal already-docked relation should emit explicit zero-hold commands."""
        skill = FineDockSkill(topology=_build_topology())
        relation_state = RelationState(
            active_module="joint1",
            passive_module="tip",
            relation_type="tip_joint",
            distance_mm=1.0,
            orientation_error_deg=1.0,
            coupled=True,
            observation_valid=True,
        )
        result = skill.execute(
            relation_state,
            _build_spec(active_module="joint1", passive_module="tip", relation_type="tip_joint"),
        )
        primary_references = [ref for ref in result.primitive_references if ref.primary]
        self.assertEqual(result.status, "done")
        self.assertTrue(result.completion.done)
        self.assertTrue(all(ref.reference_value == 0.0 for ref in primary_references))
        self.assertTrue(all(ref.semantic == "zero_hold" for ref in primary_references))

    def test_no_references_emitted_when_blocked_by_topology(self) -> None:
        """Topology-blocked docking should not emit any active references."""
        skill = FineDockSkill(topology=_build_topology())
        relation_state = RelationState(
            active_module="joint2",
            passive_module="tip",
            relation_type="tip_joint",
            distance_mm=1.0,
            orientation_error_deg=1.0,
            coupled=True,
            observation_valid=True,
        )
        result = skill.execute(
            relation_state,
            _build_spec(active_module="joint2", passive_module="tip", relation_type="tip_joint"),
        )
        self.assertEqual(result.status, "blocked")
        self.assertTrue(result.blocked_by_topology)
        self.assertEqual(result.primitive_references, [])


if __name__ == "__main__":
    unittest.main()

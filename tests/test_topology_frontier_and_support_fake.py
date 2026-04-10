"""Fake tests for Phase-3 topology frontier and support semantics."""

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


def _relation_state(active_module: str, passive_module: str, relation_type: str) -> RelationState:
    """Build a minimal valid coarse-approach relation state."""
    return RelationState(
        active_module=active_module,
        passive_module=passive_module,
        relation_type=relation_type,
        distance_mm=260.0,
        orientation_error_deg=None,
        coupled=False,
        observation_valid=True,
    )


def _spec(
    active_module: str,
    passive_module: str,
    relation_type: str,
    *,
    params: dict[str, object] | None = None,
) -> SkillSpec:
    """Build a coarse-approach spec used by the topology tests."""
    return SkillSpec(
        skill_key="coarse_approach",
        active_module=active_module,
        passive_module=passive_module,
        relation_type=relation_type,
        distance_done_mm=180.0,
        limits={"crawl_mm_s": 4.0},
        config={"gain": 0.2},
        params=dict(params or {}),
    )


class TopologyFrontierAndSupportFakeTest(unittest.TestCase):
    """Verify non-adjacency, blocked edges, frontier gating, and support protection."""

    def test_non_adjacent_pair_is_blocked(self) -> None:
        """Non-adjacent pairs should be blocked by topology."""
        topology = ChainTopology(ordered_modules=["tip", "joint1", "joint2", "joint3"])
        skill = CoarseApproachSkill(topology=topology)
        result = skill.execute(
            _relation_state("tip", "joint2", "tip_joint"),
            _spec("tip", "joint2", "tip_joint"),
        )
        self.assertEqual(result.status, "blocked")
        self.assertTrue(result.blocked_by_topology)
        self.assertEqual(result.block_reason, "non_adjacent_pair:tip->joint2")
        self.assertEqual(result.primitive_references, [])

    def test_blocked_edge_is_blocked(self) -> None:
        """Explicitly blocked edges should block skill execution."""
        topology = ChainTopology(ordered_modules=["tip", "joint1", "joint2", "joint3"])
        topology.set_blocked_edge("joint1", "joint2", True)
        skill = CoarseApproachSkill(topology=topology)
        result = skill.execute(
            _relation_state("joint2", "joint1", "joint_joint"),
            _spec("joint2", "joint1", "joint_joint"),
        )
        self.assertEqual(result.status, "blocked")
        self.assertEqual(result.block_reason, "blocked_edge:joint2->joint1")
        self.assertEqual(result.primitive_references, [])

    def test_off_frontier_pair_is_blocked_when_frontier_is_active(self) -> None:
        """Pairs away from the active frontier should be blocked unless allowed."""
        topology = ChainTopology(ordered_modules=["tip", "joint1", "joint2", "joint3"])
        topology.set_active_frontier("joint2", "joint3")
        skill = CoarseApproachSkill(topology=topology)
        result = skill.execute(
            _relation_state("joint1", "tip", "tip_joint"),
            _spec("joint1", "tip", "tip_joint"),
        )
        self.assertEqual(result.status, "blocked")
        self.assertEqual(result.block_reason, "off_frontier_pair:joint1->tip")
        self.assertTrue(result.blocked_by_topology)

    def test_support_protected_modules_block_when_required(self) -> None:
        """Support-protected modules should block actions when support stability is required."""
        topology = ChainTopology(ordered_modules=["tip", "joint1", "joint2", "joint3"])
        topology.set_support_module("joint1", True)
        skill = CoarseApproachSkill(topology=topology)
        result = skill.execute(
            _relation_state("joint2", "joint1", "joint_joint"),
            _spec("joint2", "joint1", "joint_joint", params={"requires_support_stability": True}),
        )
        self.assertEqual(result.status, "blocked")
        self.assertEqual(result.block_reason, "protected_support_module:joint1")
        self.assertTrue(result.blocked_by_topology)

    def test_legal_frontier_pair_passes(self) -> None:
        """A legal pair on the active frontier should still execute normally."""
        topology = ChainTopology(ordered_modules=["tip", "joint1", "joint2", "joint3"])
        topology.set_active_frontier("joint1", "joint2")
        skill = CoarseApproachSkill(topology=topology)
        result = skill.execute(
            _relation_state("joint2", "joint1", "joint_joint"),
            _spec("joint2", "joint1", "joint_joint"),
        )
        self.assertEqual(result.status, "active")
        self.assertTrue(result.preconditions.passed)
        self.assertFalse(result.blocked_by_topology)


if __name__ == "__main__":
    unittest.main()

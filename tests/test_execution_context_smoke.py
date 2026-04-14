"""Smoke tests for explicit execution context wiring."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mpc_control_new.control_core.models.execution_context import ExecutionContext
from mpc_control_new.control_core.models.skill_types import SkillSpec
from mpc_control_new.control_core.skills.coarse_approach import CoarseApproachSkill
from mpc_control_new.control_core.topology.chain_topology import ChainTopology
from mpc_control_new.control_core.topology.relation_state import RelationState
from mpc_control_new.control_core.controllers.adapters.skill_controller_adapter import (
    SkillControllerAdapter,
)


def _build_adapter_estimate() -> SimpleNamespace:
    """Build a fake estimate for joint-joint coarse approach tests."""
    geometry = SimpleNamespace(
        joint1_joint2_distance_mm=260.0,
        joint1_joint2_coupled=False,
        joint2_joint3_distance_mm=260.0,
        joint2_joint3_coupled=False,
    )
    control = SimpleNamespace(
        c1=0.0,
        psi1=0.0,
        theta1=0.0,
        c2=0.0,
        psi2=0.0,
        theta2=0.0,
        c3=0.0,
        psi3=0.0,
        theta3=0.0,
        d_12=False,
        d_23=False,
    )
    return SimpleNamespace(geometry=geometry, control=control)


class _RecordingCoarseApproachSkill(CoarseApproachSkill):
    """Small test double that records the execution context it receives."""

    def __init__(self, *args: object, **kwargs: object) -> None:
        super().__init__(*args, **kwargs)
        self.last_context: ExecutionContext | None = None

    def execute(
        self,
        source: object,
        spec: SkillSpec,
        *,
        module_states: dict[str, object] | None = None,
        context: ExecutionContext | None = None,
    ):
        self.last_context = context
        return super().execute(source, spec, module_states=module_states, context=context)


class ExecutionContextSmokeTest(unittest.TestCase):
    """Verify adapter/context wiring and explicit topology consumption."""

    def test_adapter_builds_and_passes_execution_context(self) -> None:
        """The adapter should construct and pass an explicit execution context."""
        recording_skill = _RecordingCoarseApproachSkill()
        adapter = SkillControllerAdapter(coarse_skill=recording_skill)
        spec = SkillSpec(
            skill_key="coarse_approach",
            active_module="joint2",
            passive_module="joint1",
            relation_type="joint_joint",
            distance_done_mm=220.0,
            solver_dt=0.25,
            limits={"crawl_mm_s": 4.0},
            config={"gain": 0.2},
        )
        result = adapter.execute(spec, _build_adapter_estimate())
        self.assertEqual(result.status, "active")
        self.assertIsNotNone(recording_skill.last_context)
        self.assertIs(recording_skill.last_context.topology, adapter.topology)
        self.assertEqual(recording_skill.last_context.dt, 0.25)
        self.assertTrue(result.used_context_topology)
        self.assertEqual(result.diagnostics["context_topology_source"], "adapter_default")

    def test_external_topology_overrides_default_adapter_topology(self) -> None:
        """Per-call topology overrides should replace the adapter default topology."""
        blocking_internal_topology = ChainTopology(ordered_modules=["tip", "joint1", "joint2"])
        recording_skill = _RecordingCoarseApproachSkill(topology=blocking_internal_topology)
        adapter = SkillControllerAdapter(coarse_skill=recording_skill)
        external_topology = ChainTopology(ordered_modules=["tip", "joint1", "joint2", "joint3"])
        spec = SkillSpec(
            skill_key="coarse_approach",
            active_module="joint3",
            passive_module="joint2",
            relation_type="joint_joint",
            distance_done_mm=220.0,
            limits={"crawl_mm_s": 4.0},
            config={"gain": 0.2},
        )
        result = adapter.execute(spec, _build_adapter_estimate(), topology=external_topology)
        self.assertEqual(result.status, "active")
        self.assertIs(recording_skill.last_context.topology, external_topology)
        self.assertEqual(result.diagnostics["context_topology_source"], "execute_override")
        self.assertFalse(result.diagnostics["context_topology_defaulted"])

    def test_skills_consume_context_topology_instead_of_internal_topology_only(self) -> None:
        """Direct skill execution should prefer explicit context topology over the skill member."""
        skill = CoarseApproachSkill(topology=ChainTopology(ordered_modules=["tip", "joint1"]))
        relation_state = RelationState(
            active_module="joint2",
            passive_module="joint1",
            relation_type="joint_joint",
            distance_mm=260.0,
            orientation_error_deg=None,
            coupled=False,
            observation_valid=True,
        )
        spec = SkillSpec(
            skill_key="coarse_approach",
            active_module="joint2",
            passive_module="joint1",
            relation_type="joint_joint",
            distance_done_mm=220.0,
            limits={"crawl_mm_s": 4.0},
            config={"gain": 0.2},
        )
        context = ExecutionContext(
            topology=ChainTopology(ordered_modules=["tip", "joint1", "joint2"]),
            dt=0.1,
        )
        result = skill.execute(relation_state, spec, context=context)
        self.assertEqual(result.status, "active")
        self.assertTrue(result.used_context_topology)
        self.assertFalse(result.blocked_by_topology)


if __name__ == "__main__":
    unittest.main()

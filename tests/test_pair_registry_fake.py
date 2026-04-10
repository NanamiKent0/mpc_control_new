"""Fake tests for the Phase-2 pair extractor registry."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mpc_control_new.control_core.models.skill_types import SkillSpec
from mpc_control_new.control_core.topology.pair_registry import PairDescriptor, PairExtractorRegistry
from mpc_control_new.control_core.topology.relation_state import RelationState
from mpc_control_new.controllers.adapters.legacy_extractors import (
    build_default_pair_extractor_registry,
    extract_relation_state_via_registry,
)
from mpc_control_new.controllers.adapters.skill_controller_adapter import SkillControllerAdapter


def _build_estimate() -> SimpleNamespace:
    """Build a minimal fake legacy-like estimate for registry tests."""
    geometry = SimpleNamespace(
        tip_joint1_distance_mm=22.0,
        tip_joint1_orientation_error_deg=7.0,
        tip_joint1_orientation_error_signed_deg=-7.0,
        tip_joint1_coupled=False,
        joint1_joint2_distance_mm=28.0,
        joint1_joint2_orientation_error_deg=4.0,
        joint1_joint2_orientation_error_signed_deg=4.0,
        joint1_joint2_coupled=False,
    )
    control = SimpleNamespace(
        c1=0.0,
        psi1=0.0,
        theta1=0.0,
        c2=0.0,
        psi2=0.0,
        theta2=0.0,
        d_t1=False,
        d_12=False,
    )
    return SimpleNamespace(geometry=geometry, control=control)


class PairRegistryFakeTest(unittest.TestCase):
    """Verify registry-based relation extraction for the Phase-2 adapter."""

    def test_default_registry_resolves_tip_joint_template(self) -> None:
        """The default registry should resolve the template tip-joint extractor."""
        registry = build_default_pair_extractor_registry()
        relation_state = extract_relation_state_via_registry(
            _build_estimate(),
            active_module="joint1",
            passive_module="tip",
            relation_type="tip_joint",
            registry=registry,
        )
        self.assertEqual(relation_state.relation_type, "tip_joint")
        self.assertEqual(relation_state.diagnostics["pair_extractor_key"], "template:tip_joint")

    def test_registry_supports_exact_match_overrides(self) -> None:
        """An exact registry binding should take precedence over the template fallback."""
        registry = build_default_pair_extractor_registry()

        def _exact_tip_joint_extractor(
            estimate: object,
            active_module: str,
            passive_module: str,
        ) -> RelationState:
            del estimate
            return RelationState(
                active_module=active_module,
                passive_module=passive_module,
                relation_type="tip_joint",
                distance_mm=999.0,
                orientation_error_deg=0.0,
                coupled=False,
                observation_valid=True,
                diagnostics={"pair_prefix": "exact_override"},
            )

        registry.register_exact(
            PairDescriptor(
                active_module="joint1",
                passive_module="tip",
                relation_type="tip_joint",
                extractor_key="exact:joint1_tip",
            ),
            _exact_tip_joint_extractor,
        )
        relation_state = extract_relation_state_via_registry(
            _build_estimate(),
            active_module="joint1",
            passive_module="tip",
            relation_type="tip_joint",
            registry=registry,
        )
        self.assertEqual(relation_state.distance_mm, 999.0)
        self.assertEqual(relation_state.diagnostics["pair_extractor_key"], "exact:joint1_tip")
        self.assertEqual(relation_state.diagnostics["pair_resolution_kind"], "exact")

    def test_registry_prefers_more_specific_templates_before_older_generic_ones(self) -> None:
        """Template specificity should outrank registration order when both match."""
        registry = PairExtractorRegistry()

        def _generic_extractor(
            estimate: object,
            active_module: str,
            passive_module: str,
        ) -> RelationState:
            del estimate
            return RelationState(
                active_module=active_module,
                passive_module=passive_module,
                relation_type="joint_joint",
                distance_mm=111.0,
                orientation_error_deg=None,
                coupled=False,
                observation_valid=True,
                diagnostics={},
            )

        def _specific_extractor(
            estimate: object,
            active_module: str,
            passive_module: str,
        ) -> RelationState:
            del estimate
            return RelationState(
                active_module=active_module,
                passive_module=passive_module,
                relation_type="joint_joint",
                distance_mm=222.0,
                orientation_error_deg=None,
                coupled=False,
                observation_valid=True,
                diagnostics={},
            )

        registry.register_template(
            extractor_key="template:generic_joint_joint",
            relation_type="joint_joint",
            active_module="*",
            passive_module="*",
            matcher=lambda active_module, passive_module, relation_type: (
                relation_type == "joint_joint"
                and active_module.startswith("joint")
                and passive_module.startswith("joint")
            ),
            extractor=_generic_extractor,
        )
        registry.register_template(
            extractor_key="template:specific_joint3_joint2",
            relation_type="joint_joint",
            active_module="joint3",
            passive_module="*",
            matcher=lambda active_module, passive_module, relation_type: (
                relation_type == "joint_joint"
                and active_module == "joint3"
                and passive_module.startswith("joint")
            ),
            extractor=_specific_extractor,
        )

        relation_state = extract_relation_state_via_registry(
            _build_estimate(),
            active_module="joint3",
            passive_module="joint2",
            relation_type="joint_joint",
            registry=registry,
        )
        self.assertEqual(relation_state.distance_mm, 222.0)
        self.assertEqual(relation_state.diagnostics["pair_extractor_key"], "template:specific_joint3_joint2")
        self.assertEqual(relation_state.diagnostics["pair_resolution_kind"], "template")
        self.assertEqual(relation_state.diagnostics["pair_resolution_specificity"], 1.0)

    def test_adapter_resolves_tip_joint_without_inline_pair_logic(self) -> None:
        """Adapter should dispatch a tip-joint request through the registry."""
        adapter = SkillControllerAdapter()
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
        result = adapter.execute(spec, _build_estimate())
        self.assertEqual(result.status, "active")
        self.assertEqual(result.diagnostics["pair_extractor_key"], "template:tip_joint")
        self.assertEqual(result.diagnostics["resolved_pair_extractor_key"], "template:tip_joint")

    def test_adapter_resolves_joint_joint_without_inline_pair_logic(self) -> None:
        """Adapter should dispatch a joint-joint request through the registry."""
        adapter = SkillControllerAdapter()
        spec = SkillSpec(
            skill_key="coarse_approach",
            active_module="joint2",
            passive_module="joint1",
            relation_type="joint_joint",
            distance_done_mm=18.0,
            limits={"crawl_mm_s": 4.0},
            config={"gain": 0.2},
        )
        result = adapter.execute(spec, _build_estimate())
        self.assertEqual(result.status, "active")
        self.assertEqual(result.diagnostics["pair_extractor_key"], "template:joint_joint")

    def test_unresolved_pair_returns_clean_adapter_error(self) -> None:
        """Adapter should report a structured extractor-resolution failure."""
        adapter = SkillControllerAdapter()
        spec = SkillSpec(
            skill_key="coarse_approach",
            active_module="joint2",
            passive_module="joint1",
            relation_type="tip_joint",
            distance_done_mm=18.0,
        )
        result = adapter.execute(spec, _build_estimate())
        self.assertEqual(result.status, "blocked")
        self.assertEqual(result.block_reason, "no_pair_extractor:joint2->joint1:tip_joint")
        self.assertEqual(result.diagnostics["adapter_error"], "no_pair_extractor:joint2->joint1:tip_joint")
        self.assertEqual(result.primitive_references, [])


if __name__ == "__main__":
    unittest.main()

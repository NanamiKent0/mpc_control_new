"""Fake tests for generic `jointN` module extraction."""

from __future__ import annotations

import inspect
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mpc_control_new.controllers.adapters.legacy_extractors import (
    extract_module_state,
    is_joint_module,
    parse_joint_index,
)


def _build_estimate() -> SimpleNamespace:
    """Build a minimal fake estimate with generic joint3/joint4 fields."""
    geometry = SimpleNamespace(
        joint2_joint3_coupled=False,
        joint3_joint4_coupled=True,
    )
    control = SimpleNamespace(
        c3=12.0,
        psi3=-8.0,
        theta3=4.0,
        c3_dot=1.5,
        psi3_dot=-0.5,
        theta3_dot=0.25,
    )
    return SimpleNamespace(geometry=geometry, control=control)


class JointNModuleExtractionFakeTest(unittest.TestCase):
    """Verify generic `jointN` extraction and graceful degradation."""

    def test_joint_identifiers_parse_generically(self) -> None:
        """The helper parsing path should recognize arbitrary jointN identifiers."""
        self.assertEqual(parse_joint_index("joint3"), 3)
        self.assertEqual(parse_joint_index("joint14"), 14)
        self.assertIsNone(parse_joint_index("tip"))
        self.assertTrue(is_joint_module("joint4"))
        self.assertFalse(is_joint_module("jointx"))

    def test_joint3_extraction_returns_normalized_module_state(self) -> None:
        """A generic jointN request should populate the expected joint state fields."""
        module_state = extract_module_state(_build_estimate(), "joint3")
        self.assertEqual(module_state.module_type, "joint")
        self.assertEqual(module_state.dofs["crawl_mm"], 12.0)
        self.assertEqual(module_state.dofs["rotate_deg"], -8.0)
        self.assertEqual(module_state.velocities["crawl_mm_s"], 1.5)
        self.assertEqual(module_state.attach_state["joint2"], False)
        self.assertEqual(module_state.attach_state["joint4"], True)
        self.assertEqual(module_state.metadata["extractor_path"], "generic_jointN")
        self.assertEqual(module_state.metadata["joint_index"], 3)

    def test_missing_joint4_fields_degrade_gracefully(self) -> None:
        """Missing legacy fields should return an empty-but-usable joint module state."""
        module_state = extract_module_state(_build_estimate(), "joint4")
        self.assertEqual(module_state.module_type, "joint")
        self.assertEqual(module_state.dofs, {})
        self.assertIn("c4", str(module_state.metadata["legacy_dof_fields_missing"]))
        self.assertIn("joint5", str(module_state.metadata["legacy_attach_fields_missing"]))
        self.assertTrue(any("legacy dof fields unavailable" in note for note in module_state.notes))

    def test_main_extraction_path_no_longer_branches_on_joint1_joint2(self) -> None:
        """The main extraction logic should not depend on explicit joint1/joint2 branches."""
        source = inspect.getsource(extract_module_state)
        self.assertNotIn('module_id == "joint1"', source)
        self.assertNotIn('module_id == "joint2"', source)


if __name__ == "__main__":
    unittest.main()

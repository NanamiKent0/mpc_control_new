"""Fake tests for runtime-frame state builders."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mpc_control_new.runtime_integration.observation_types import (
    ModuleObservation,
    PairObservation,
    RuntimeObservationFrame,
)
from mpc_control_new.runtime_integration.runtime_state_builder import (
    RUNTIME_GEOMETRY_SCHEMA,
    RUNTIME_STATE_BUILDER_SOURCE,
    build_geometry_observation_from_frame,
    build_module_state_for_module,
    build_module_state_map,
    build_relation_state_from_frame,
)


def _frame() -> RuntimeObservationFrame:
    """Build a minimal runtime frame for state-builder tests."""
    return RuntimeObservationFrame(
        timestamp_ns=42,
        module_observations={
            "joint1": ModuleObservation(
                module_id="joint1",
                module_type="joint",
                dofs={"crawl_mm": 10.0, "rotate_deg": -5.0},
                velocities={"crawl_mm_s": 1.5},
                attach_state={"tip": False},
                source_name="runtime_fake",
            ),
            "tip": ModuleObservation(
                module_id="tip",
                module_type="tip",
                dofs={"growth_mm": 2.0},
                source_name="runtime_fake",
            ),
        },
        pair_observations={
            "joint1->tip": PairObservation(
                active_module="joint1",
                passive_module="tip",
                relation_type="tip_joint",
                distance_mm=14.0,
                orientation_error_deg=-9.0,
                coupled=False,
                observation_valid=True,
                diagnostics={"quality": "ok"},
                source_name="runtime_fake",
            )
        },
        metadata={"source_name": "runtime_fake"},
    )


class RuntimeStateBuilderFakeTest(unittest.TestCase):
    """Verify runtime frames can build module, geometry, and relation state directly."""

    def test_runtime_frame_builds_module_state_map(self) -> None:
        """Module observations should become normalized module states without legacy estimates."""
        frame = _frame()
        module_state_map = build_module_state_map(frame)
        placeholder = build_module_state_for_module(frame, "joint2")

        self.assertEqual(module_state_map["joint1"].dofs["crawl_mm"], 10.0)
        self.assertEqual(module_state_map["joint1"].metadata["state_builder_source"], RUNTIME_STATE_BUILDER_SOURCE)
        self.assertIn("runtime_module_observation_missing", placeholder.notes)
        self.assertEqual(placeholder.metadata["runtime_module_placeholder"], True)

    def test_runtime_frame_builds_geometry_observation(self) -> None:
        """Pair observations should become geometry observations with runtime provenance."""
        geometry = build_geometry_observation_from_frame(
            _frame(),
            active_module="joint1",
            passive_module="tip",
            relation_type="tip_joint",
        )

        self.assertEqual(geometry.source_schema, RUNTIME_GEOMETRY_SCHEMA)
        self.assertEqual(geometry.observation_origin, "runtime_frame")
        self.assertEqual(geometry.source_name, "runtime_fake")
        self.assertEqual(geometry.source_fields["distance_mm"], "runtime_frame.pair_observations.joint1->tip.distance_mm")

    def test_runtime_frame_builds_relation_state(self) -> None:
        """Geometry observations from runtime frames should become relation states directly."""
        relation_state = build_relation_state_from_frame(
            _frame(),
            active_module="joint1",
            passive_module="tip",
            relation_type="tip_joint",
        )

        self.assertTrue(relation_state.observation_valid)
        self.assertEqual(relation_state.diagnostics["state_builder_source"], RUNTIME_STATE_BUILDER_SOURCE)
        self.assertEqual(relation_state.diagnostics["geometry_source_schema"], RUNTIME_GEOMETRY_SCHEMA)
        self.assertNotIn("pair_extractor_key", relation_state.diagnostics)


if __name__ == "__main__":
    unittest.main()

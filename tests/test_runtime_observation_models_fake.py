"""Fake tests for runtime-facing observation dataclasses."""

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


class RuntimeObservationModelsFakeTest(unittest.TestCase):
    """Verify runtime-facing observation models normalize and preserve data."""

    def test_runtime_observation_frame_normalizes_module_and_pair_payloads(self) -> None:
        """Module/pair observations should normalize numeric payloads and preserve metadata."""
        frame = RuntimeObservationFrame(
            timestamp_ns=123456789,
            module_observations={
                "joint1": ModuleObservation(
                    module_id="joint1",
                    module_type="joint",
                    dofs={"crawl_mm": 12},
                    velocities={"crawl_mm_s": 3},
                    attach_state={"tip": False},
                    diagnostics={"raw_source": "sim"},
                    source_name="sim_provider",
                )
            },
            pair_observations={
                "joint1->tip": PairObservation(
                    active_module="joint1",
                    passive_module="tip",
                    relation_type="tip_joint",
                    distance_mm=18,
                    orientation_error_deg=-6,
                    coupled=False,
                    observation_valid=True,
                    diagnostics={"quality": "ok"},
                    source_name="sim_provider",
                )
            },
            metadata={"scene": "fake"},
        )

        self.assertEqual(frame.module_observations["joint1"].dofs["crawl_mm"], 12.0)
        self.assertEqual(frame.module_observations["joint1"].velocities["crawl_mm_s"], 3.0)
        self.assertEqual(frame.pair_observations["joint1->tip"].distance_mm, 18.0)
        self.assertEqual(frame.pair_observations["joint1->tip"].orientation_error_deg, -6.0)
        self.assertEqual(frame.metadata["scene"], "fake")

    def test_missing_fields_gracefully_degrade_and_reverse_lookup_is_supported(self) -> None:
        """Sparse payloads should degrade safely and allow reverse pair lookup."""
        frame = RuntimeObservationFrame(
            timestamp_ns=1,
            module_observations={
                "tip": ModuleObservation(module_id="tip"),
            },
            pair_observations={
                "tip->joint1": PairObservation(
                    active_module="tip",
                    passive_module="joint1",
                    relation_type="tip_joint",
                    observation_valid=False,
                    diagnostics={"quality": "missing"},
                )
            },
            metadata={"source_name": "fake_runtime"},
        )

        reverse_lookup = frame.get_pair_observation("joint1", "tip")
        self.assertIsNone(frame.module_observations["tip"].module_type)
        self.assertEqual(frame.metadata["source_name"], "fake_runtime")
        self.assertFalse(reverse_lookup.observation_valid)
        self.assertEqual(reverse_lookup.diagnostics["pair_reordered_from"], "tip->joint1")
        self.assertEqual(reverse_lookup.source_name, "runtime")


if __name__ == "__main__":
    unittest.main()

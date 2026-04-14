"""Fake contract tests for the standalone sim visualizer."""

from __future__ import annotations

import inspect
import os
import sys
import unittest
from pathlib import Path

os.environ.setdefault("MPLBACKEND", "Agg")

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mpc_control_new.runtime_integration.self_containment_checks import scan_file_for_forbidden_imports
from mpc_control_new.runtime_integration.sim_backend.types import SimState
from mpc_control_new.runtime_integration.sim_backend.visualizer import SimVisualizer


class SimVisualizerContractFakeTest(unittest.TestCase):
    """Verify the migrated visualizer stays self-contained and snapshot-driven."""

    def test_visualizer_constructs_and_consumes_state_snapshot(self) -> None:
        visualizer = SimVisualizer()
        payload = visualizer.consume_state_snapshot(
            {
                "state": SimState(
                    g=120.0,
                    c1=80.0,
                    c2=35.0,
                    c3=20.0,
                    c4=10.0,
                    c5=5.0,
                    theta1=15.0,
                    theta2=-8.0,
                    theta3=6.0,
                    theta4=-4.0,
                    theta5=2.0,
                    psi1=12.0,
                    psi2=-6.0,
                    psi3=5.0,
                    psi4=-3.0,
                    psi5=1.0,
                    seq=3,
                    tip_joint1_distance_mm=4.0,
                    joint1_joint2_distance_mm=7.5,
                ).to_dict()
            }
        )

        self.assertIn("module_paths", payload)
        self.assertIn("named_points", payload)
        self.assertTrue(payload["module_paths"]["tip"][0])
        self.assertTrue(payload["module_paths"]["joint5"][0])
        self.assertIn("joint5_end", payload["named_points"])
        self.assertIn("seq=3", payload["status_text"])

    def test_visualizer_lives_inside_new_package_and_has_no_legacy_imports(self) -> None:
        source_path = Path(inspect.getsourcefile(SimVisualizer) or "")

        self.assertIn("mpc_control_new/runtime_integration/sim_backend/visualizer.py", str(source_path))
        self.assertEqual(scan_file_for_forbidden_imports(source_path), [])


if __name__ == "__main__":
    unittest.main()

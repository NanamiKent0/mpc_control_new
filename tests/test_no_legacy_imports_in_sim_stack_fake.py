"""Fake tests ensuring the standalone sim stack is self-contained."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mpc_control_new.runtime_integration.self_containment_checks import (
    run_self_containment_check,
    scan_file_for_forbidden_imports,
)


class NoLegacyImportsInSimStackFakeTest(unittest.TestCase):
    """Verify migrated sim stack files do not import or reference the legacy project."""

    def test_self_containment_scan_covers_new_sim_stack_files(self) -> None:
        report = run_self_containment_check()

        self.assertTrue(report.passed, msg=str(report.violations))
        self.assertTrue(
            any(path.endswith("runtime_integration/sim_backend/ros2_backend.py") for path in report.scanned_paths)
        )
        self.assertTrue(
            any(path.endswith("runtime_integration/sim_backend/visualizer.py") for path in report.scanned_paths)
        )
        self.assertTrue(
            any(path.endswith("runtime_integration/sim_backend/start_gui_sim_stack.sh") for path in report.scanned_paths)
        )

    def test_key_sim_stack_files_have_no_forbidden_hits(self) -> None:
        files = [
            PROJECT_ROOT / "mpc_control_new" / "runtime_integration" / "sim_backend" / "ros2_backend.py",
            PROJECT_ROOT / "mpc_control_new" / "runtime_integration" / "sim_backend" / "visualizer.py",
            PROJECT_ROOT / "mpc_control_new" / "runtime_integration" / "sim_backend" / "start_gui_sim_stack.sh",
            PROJECT_ROOT / "mpc_control_new" / "runtime_integration" / "gui" / "gui_ros2.py",
        ]

        for file_path in files:
            with self.subTest(file_path=str(file_path)):
                self.assertEqual(scan_file_for_forbidden_imports(file_path), [])


if __name__ == "__main__":
    unittest.main()

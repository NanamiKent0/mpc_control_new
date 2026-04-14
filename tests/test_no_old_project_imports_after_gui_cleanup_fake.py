"""Fake tests ensuring GUI cleanup left one self-contained canonical path."""

from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mpc_control_new.runtime_integration.self_containment_checks import (
    run_self_containment_check,
    scan_file_for_forbidden_imports,
)


class NoOldProjectImportsAfterGuiCleanupFakeTest(unittest.TestCase):
    """Verify old GUI imports/files are gone after the canonical GUI cleanup."""

    def test_self_containment_scan_covers_canonical_gui_paths(self) -> None:
        report = run_self_containment_check()

        self.assertTrue(report.passed, msg=str(report.violations))
        self.assertTrue(
            any(path.endswith("runtime_integration/gui/gui_ros2.py") for path in report.scanned_paths)
        )
        self.assertTrue(
            any(path.endswith("runtime_integration/common/encoder_protocol.py") for path in report.scanned_paths)
        )
        self.assertTrue(
            any(path.endswith("runtime_integration/gui/backend_manager.py") for path in report.scanned_paths)
        )
        self.assertTrue(any(path.endswith("README_phase1.md") for path in report.scanned_paths))

    def test_scan_detects_old_project_and_top_level_encoder_imports(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            old_file = Path(temp_dir) / "bad_old_import.py"
            old_file.write_text("from MPC_control.gui.runtime import App\n", encoding="utf-8")
            encoder_file = Path(temp_dir) / "bad_encoder_import.py"
            encoder_file.write_text(
                "from encoder_protocol import counts_to_physical\n",
                encoding="utf-8",
            )
            old_hits = scan_file_for_forbidden_imports(old_file)
            encoder_hits = scan_file_for_forbidden_imports(encoder_file)

        self.assertIn("legacy_project_import", old_hits)
        self.assertIn("top_level_encoder_protocol_import", encoder_hits)


if __name__ == "__main__":
    unittest.main()

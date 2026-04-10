"""Fake tests for the self-containment import scan."""

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


class NoLegacyImportsFakeTest(unittest.TestCase):
    """Verify the static self-containment scan catches forbidden imports."""

    def test_current_package_passes_self_containment_scan(self) -> None:
        """The current `mpc_control_new` package should not import legacy project modules."""
        report = run_self_containment_check()

        self.assertTrue(report.passed, msg=str(report.violations))
        self.assertGreater(report.scanned_files, 0)

    def test_scan_file_detects_forbidden_legacy_imports(self) -> None:
        """A synthetic file with a forbidden import should be reported."""
        with tempfile.TemporaryDirectory() as temp_dir:
            file_path = Path(temp_dir) / "bad_import.py"
            file_path.write_text("from sim.sim_types import SimState\n", encoding="utf-8")

            hits = scan_file_for_forbidden_imports(file_path)

        self.assertIn("legacy_sim_import", hits)


if __name__ == "__main__":
    unittest.main()

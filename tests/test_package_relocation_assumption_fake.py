"""Fake tests for the package-relocation assumption."""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))


class PackageRelocationAssumptionFakeTest(unittest.TestCase):
    """Verify `mpc_control_new` can still run after being copied out of the legacy tree."""

    def test_package_copy_can_boot_sim_runtime_without_legacy_root(self) -> None:
        """A copied standalone package should import and run the sim runtime session."""
        package_root = PROJECT_ROOT / "mpc_control_new"
        with tempfile.TemporaryDirectory() as temp_dir:
            relocated_root = Path(temp_dir) / "mpc_control_new"
            shutil.copytree(package_root, relocated_root)
            env = dict(os.environ)
            env["PYTHONPATH"] = temp_dir
            code = """
from mpc_control_new.runtime_integration.runtime_session import build_sim_runtime_session
from mpc_control_new.runtime_integration.self_containment_checks import run_self_containment_check
report = run_self_containment_check()
assert report.passed, report.violations
session = build_sim_runtime_session()
result = session.step()
assert result.diagnostics["provider_kind"] == "sim"
assert result.diagnostics["input_source"] == "runtime_frame:sim"
"""
            completed = subprocess.run(
                [sys.executable, "-c", code],
                env=env,
                capture_output=True,
                text=True,
                check=False,
            )

        self.assertEqual(completed.returncode, 0, msg=completed.stderr or completed.stdout)


if __name__ == "__main__":
    unittest.main()

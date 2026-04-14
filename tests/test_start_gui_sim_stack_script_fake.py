"""Fake tests for the standalone sim stack launcher script."""

from __future__ import annotations

import subprocess
import sys
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mpc_control_new.runtime_integration.self_containment_checks import scan_file_for_forbidden_imports


class StartGuiSimStackScriptFakeTest(unittest.TestCase):
    """Verify the launcher script only uses the new package paths."""

    def test_script_exists_is_shell_valid_and_uses_new_paths(self) -> None:
        script_path = (
            PROJECT_ROOT
            / "mpc_control_new"
            / "runtime_integration"
            / "sim_backend"
            / "start_gui_sim_stack.sh"
        )
        script_text = script_path.read_text(encoding="utf-8")
        syntax = subprocess.run(
            ["bash", "-n", str(script_path)],
            capture_output=True,
            text=True,
            check=False,
        )

        self.assertTrue(script_path.exists())
        self.assertEqual(syntax.returncode, 0, msg=syntax.stderr)
        self.assertIn("runtime_integration/sim_backend/run_sim_backend.py", script_text)
        self.assertIn("runtime_integration/sim_backend/run_sim_visualizer.py", script_text)
        self.assertIn("runtime_integration/gui/gui_ros2.py", script_text)
        self.assertEqual(scan_file_for_forbidden_imports(script_path), [])


if __name__ == "__main__":
    unittest.main()

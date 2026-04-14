"""Fake launch tests for the standalone runtime entry points."""

from __future__ import annotations

import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))


class IndependentPathLaunchFakeTest(unittest.TestCase):
    """Verify demo entry points launch from the standalone package root."""

    def test_runtime_demo_and_gui_entry_launch_independently(self) -> None:
        """CLI entry points should run from an arbitrary cwd without legacy imports."""
        env = dict(os.environ)
        env["PYTHONPATH"] = str(PROJECT_ROOT)
        env.setdefault("QT_QPA_PLATFORM", "offscreen")
        gui_entry = PROJECT_ROOT / "mpc_control_new" / "runtime_integration" / "gui" / "gui_ros2.py"
        with tempfile.TemporaryDirectory() as temp_dir:
            sim_demo = subprocess.run(
                [
                    sys.executable,
                    "-m",
                    "mpc_control_new.runtime_integration.apps.run_sim_runtime_demo",
                    "--steps",
                    "2",
                ],
                cwd=temp_dir,
                env=env,
                capture_output=True,
                text=True,
                check=False,
            )
            gui_demo = subprocess.run(
                [
                    sys.executable,
                    str(gui_entry),
                    "--headless",
                    "--check-only",
                    "--disable-live-backend",
                ],
                cwd=temp_dir,
                env=env,
                capture_output=True,
                text=True,
                check=False,
            )
            ros2_demo = subprocess.run(
                [
                    sys.executable,
                    "-m",
                    "mpc_control_new.runtime_integration.gui.gui_ros2",
                    "--check-only",
                    "--headless",
                ],
                cwd=temp_dir,
                env=env,
                capture_output=True,
                text=True,
                check=False,
            )

        self.assertEqual(sim_demo.returncode, 0, msg=sim_demo.stderr or sim_demo.stdout)
        self.assertEqual(gui_demo.returncode, 0, msg=gui_demo.stderr or gui_demo.stdout)
        self.assertEqual(ros2_demo.returncode, 0, msg=ros2_demo.stderr or ros2_demo.stdout)
        gui_summary = json.loads(gui_demo.stdout.strip())
        self.assertEqual(gui_summary["dispatch_mode"], "none")
        ros2_summary = json.loads(ros2_demo.stdout.strip())
        self.assertIn("dispatch_mode", ros2_summary)
        self.assertEqual(ros2_summary["self_contained"], True)


if __name__ == "__main__":
    unittest.main()

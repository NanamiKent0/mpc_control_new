"""Final cleanup tests for the canonical GUI entrypoint."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
REPO_ROOT = PROJECT_ROOT / "mpc_control_new"
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))


def _stem(*parts: str) -> str:
    return "_".join(parts)


def _legacy_entry_source_paths() -> list[Path]:
    app_stems = (
        ("run", "gui", "ros2"),
        ("run", "sim", "with", "gui", "ros2"),
        ("run", "ros2", "gui", "ros2"),
        ("run", "runtime", "gui"),
    )
    gui_stems = (
        ("runtime", "gui", "app"),
        ("runtime", "view", "model"),
        ("runtime", "visualizer"),
        ("runtime", "gui", "controls"),
    )
    paths: list[Path] = []
    for parts in app_stems:
        paths.append(REPO_ROOT / "runtime_integration" / "apps" / f"{_stem(*parts)}.py")
    for parts in gui_stems:
        paths.append(REPO_ROOT / "runtime_integration" / "gui" / f"{_stem(*parts)}.py")
    return paths


class GuiEntrypointUniquenessFakeTest(unittest.TestCase):
    """Verify the repository exposes exactly one formal GUI launch file."""

    def test_only_canonical_gui_entry_source_remains(self) -> None:
        gui_entry = REPO_ROOT / "runtime_integration" / "gui" / "gui_ros2.py"
        self.assertTrue(gui_entry.exists())
        for path in _legacy_entry_source_paths():
            self.assertFalse(path.exists(), msg=str(path))

    def test_apps_directory_only_keeps_internal_runtime_demos(self) -> None:
        apps_dir = REPO_ROOT / "runtime_integration" / "apps"
        remaining = sorted(path.name for path in apps_dir.glob("*.py"))
        self.assertEqual(
            remaining,
            ["__init__.py", "run_ros2_runtime_demo.py", "run_sim_runtime_demo.py"],
        )

    def test_readme_declares_one_formal_gui_entry(self) -> None:
        readme_text = (REPO_ROOT / "README_phase1.md").read_text(encoding="utf-8")
        self.assertIn("gui_ros2.py` is now the only formal GUI launch entry", readme_text)
        self.assertIn("cd /home/cty/mpc_control_new/runtime_integration/gui", readme_text)
        self.assertIn("python3 gui_ros2.py", readme_text)
        self.assertIn("内部 demo / 诊断脚本", readme_text)


if __name__ == "__main__":
    unittest.main()

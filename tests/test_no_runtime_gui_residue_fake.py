"""Repository residue checks for the removed runtime GUI wrappers and caches."""

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


def _legacy_name_stems() -> tuple[str, ...]:
    return (
        _stem("runtime", "gui", "app"),
        _stem("runtime", "view", "model"),
        _stem("runtime", "visualizer"),
        _stem("runtime", "gui", "controls"),
        _stem("run", "gui", "ros2"),
        _stem("run", "sim", "with", "gui", "ros2"),
        _stem("run", "ros2", "gui", "ros2"),
        _stem("run", "runtime", "gui"),
    )


def _source_files() -> list[Path]:
    files = [
        path
        for path in REPO_ROOT.rglob("*")
        if path.is_file()
        and "__pycache__" not in path.parts
        and path.suffix in {".py", ".md"}
    ]
    return sorted(files)


class NoRuntimeGuiResidueFakeTest(unittest.TestCase):
    """Verify no legacy runtime-GUI wrappers or cache artifacts remain."""

    def test_source_tree_contains_no_legacy_gui_name_references(self) -> None:
        for path in _source_files():
            text = path.read_text(encoding="utf-8", errors="ignore")
            for stem in _legacy_name_stems():
                self.assertNotIn(stem, text, msg=f"{stem} referenced in {path}")

    def test_gui_and_apps_cache_dirs_contain_no_legacy_bytecode(self) -> None:
        cache_roots = [
            REPO_ROOT / "runtime_integration" / "apps" / "__pycache__",
            REPO_ROOT / "runtime_integration" / "gui" / "__pycache__",
        ]
        for root in cache_roots:
            if not root.exists():
                continue
            for path in root.rglob("*.pyc"):
                for stem in _legacy_name_stems():
                    self.assertNotIn(stem, path.name, msg=str(path))

    def test_gitignore_covers_python_cache_files(self) -> None:
        gitignore_path = REPO_ROOT / ".gitignore"
        self.assertTrue(gitignore_path.exists())
        text = gitignore_path.read_text(encoding="utf-8")
        self.assertIn("__pycache__/", text)
        self.assertIn("*.pyc", text)


if __name__ == "__main__":
    unittest.main()

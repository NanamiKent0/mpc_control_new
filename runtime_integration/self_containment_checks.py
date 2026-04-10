"""Static checks that guard `mpc_control_new` against legacy import dependencies."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
import re


FORBIDDEN_IMPORT_PATTERNS: tuple[tuple[str, re.Pattern[str]], ...] = (
    ("legacy_sim_import", re.compile(r"^\s*from\s+sim(?:\.|\s+import\b)", re.MULTILINE)),
    ("legacy_sim_import", re.compile(r"^\s*import\s+sim(?:\.|\b)", re.MULTILINE)),
    ("legacy_project_import", re.compile(r"^\s*from\s+MPC_control(?:\.|\s+import\b)", re.MULTILINE)),
    ("legacy_project_import", re.compile(r"^\s*import\s+MPC_control(?:\.|\b)", re.MULTILINE)),
    ("old_project_import", re.compile(r"^\s*from\s+old_project(?:\.|\s+import\b)", re.MULTILINE)),
    ("old_project_import", re.compile(r"^\s*import\s+old_project(?:\.|\b)", re.MULTILINE)),
    ("legacy_package_import", re.compile(r"^\s*from\s+legacy(?:\.|\s+import\b)", re.MULTILINE)),
    ("legacy_package_import", re.compile(r"^\s*import\s+legacy(?:\.|\b)", re.MULTILINE)),
)

FORBIDDEN_TEXT_PATTERNS: tuple[tuple[str, re.Pattern[str]], ...] = (
    ("legacy_absolute_path", re.compile(r"/home/cty/MPC_control")),
)


@dataclass(slots=True)
class SelfContainmentReport:
    """Structured result of the package self-containment scan."""

    passed: bool
    package_root: str
    scanned_files: int
    scanned_paths: list[str] = field(default_factory=list)
    violations: dict[str, list[str]] = field(default_factory=dict)


def default_package_root() -> Path:
    """Return the `mpc_control_new` package root used by the static scan."""
    return Path(__file__).resolve().parents[1]


def iter_python_source_files(package_root: str | Path | None = None) -> list[Path]:
    """Return Python source files covered by the self-containment scan."""
    root = Path(package_root) if package_root is not None else default_package_root()
    return [
        file_path
        for file_path in sorted(root.rglob("*.py"))
        if "__pycache__" not in file_path.parts
    ]


def run_self_containment_check(package_root: str | Path | None = None) -> SelfContainmentReport:
    """Scan the package tree for forbidden imports that would break after relocation."""
    root = Path(package_root) if package_root is not None else default_package_root()
    violations: dict[str, list[str]] = {}
    scanned_paths = [str(file_path) for file_path in iter_python_source_files(root)]
    for file_path in scanned_paths:
        matches = scan_file_for_forbidden_imports(file_path)
        if matches:
            violations[file_path] = matches
    return SelfContainmentReport(
        passed=not violations,
        package_root=str(root),
        scanned_files=len(scanned_paths),
        scanned_paths=scanned_paths,
        violations=violations,
    )


def scan_file_for_forbidden_imports(file_path: str | Path) -> list[str]:
    """Scan one Python file for forbidden import statements."""
    text = Path(file_path).read_text(encoding="utf-8")
    hits: list[str] = []
    for label, pattern in FORBIDDEN_IMPORT_PATTERNS:
        if pattern.search(text):
            hits.append(label)
    for label, pattern in FORBIDDEN_TEXT_PATTERNS:
        if pattern.search(text):
            hits.append(label)
    return hits

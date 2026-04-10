"""Fake tests for the Phase-5 skill registry."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mpc_control_new.control_core.orchestration.skill_registry import (
    SkillRegistry,
    build_default_skill_registry,
)


class SkillRegistryFakeTest(unittest.TestCase):
    """Verify the standalone skill registry behaviour."""

    def test_registry_can_register_and_resolve_skill(self) -> None:
        """A registered skill should be retrievable and resolvable by key."""
        registry = SkillRegistry()
        skill = object()
        registry.register("fake_skill", skill, metadata={"family": "fake"})

        self.assertIs(registry.get("fake_skill"), skill)
        resolution = registry.resolve("fake_skill")
        self.assertTrue(resolution.found)
        self.assertIs(resolution.skill_instance, skill)
        self.assertEqual(resolution.metadata["family"], "fake")

    def test_duplicate_registration_requires_override(self) -> None:
        """Re-registering a key without override should fail, with override should replace."""
        registry = SkillRegistry()
        first = object()
        second = object()
        registry.register("fake_skill", first)
        with self.assertRaisesRegex(ValueError, "skill_registry_duplicate"):
            registry.register("fake_skill", second)
        registry.register("fake_skill", second, override=True, metadata={"version": 2})

        resolution = registry.resolve("fake_skill")
        self.assertIs(resolution.skill_instance, second)
        self.assertEqual(resolution.metadata["version"], 2)

    def test_missing_skill_returns_structured_error(self) -> None:
        """Missing keys should return a structured lookup result instead of raising."""
        registry = SkillRegistry()

        resolution = registry.resolve("missing_skill")
        self.assertFalse(resolution.found)
        self.assertIsNone(resolution.skill_instance)
        self.assertEqual(resolution.error, "skill_not_registered:missing_skill")

    def test_default_registry_contains_core_skills(self) -> None:
        """The default registry should include the baseline coarse and fine skills."""
        registry = build_default_skill_registry()

        registered = {descriptor.skill_key for descriptor in registry.list_registered()}
        self.assertIn("coarse_approach", registered)
        self.assertIn("fine_dock", registered)


if __name__ == "__main__":
    unittest.main()

"""Registry helpers for resolving reusable relation skills by key."""

from __future__ import annotations

from ..models.skill_types import SkillDescriptor, SkillResolutionResult
from ..skills.coarse_approach import CoarseApproachSkill
from ..skills.fine_dock import FineDockSkill
from ..skills.terminal import TerminalNoopSkill
from ..topology.chain_topology import ChainTopology


class SkillRegistry:
    """Resolve concrete skill instances without hardcoding adapter routing logic."""

    def __init__(self, *, source_name: str = "custom_skill_registry") -> None:
        self.source_name = source_name
        self._entries: dict[str, tuple[SkillDescriptor, object]] = {}

    def register(
        self,
        skill_key: str,
        skill_instance: object,
        *,
        metadata: dict[str, object] | None = None,
        override: bool = False,
    ) -> SkillDescriptor:
        """Register one skill instance under a stable lookup key."""
        if not override and skill_key in self._entries:
            raise ValueError(f"skill_registry_duplicate:{skill_key}")
        descriptor = SkillDescriptor(
            skill_key=skill_key,
            metadata=dict(metadata or {}),
            registry_source=self.source_name,
        )
        self._entries[skill_key] = (descriptor, skill_instance)
        return descriptor

    def get(self, skill_key: str) -> object | None:
        """Return one registered skill instance with a nullable fallback."""
        entry = self._entries.get(skill_key)
        if entry is None:
            return None
        return entry[1]

    def resolve(self, skill_key: str) -> SkillResolutionResult:
        """Return a structured resolution result instead of raising on misses."""
        entry = self._entries.get(skill_key)
        if entry is None:
            return SkillResolutionResult(
                skill_key=skill_key,
                found=False,
                skill_instance=None,
                descriptor=None,
                registry_source=self.source_name,
                error=f"skill_not_registered:{skill_key}",
            )
        descriptor, skill_instance = entry
        return SkillResolutionResult(
            skill_key=skill_key,
            found=True,
            skill_instance=skill_instance,
            descriptor=descriptor,
            registry_source=self.source_name,
            error=None,
        )

    def list_registered(self) -> list[SkillDescriptor]:
        """Return the registered descriptors in insertion order."""
        return [descriptor for descriptor, _ in self._entries.values()]


def build_default_skill_registry(
    *,
    topology: ChainTopology | None = None,
    coarse_skill: CoarseApproachSkill | None = None,
    fine_skill: FineDockSkill | None = None,
    terminal_skill: TerminalNoopSkill | None = None,
) -> SkillRegistry:
    """Build the default Phase-6 registry used by adapters and schedulers."""
    registry = SkillRegistry(source_name="default_skill_registry")
    resolved_coarse = coarse_skill or CoarseApproachSkill(topology=topology)
    resolved_fine = fine_skill or FineDockSkill(topology=topology)
    resolved_terminal = terminal_skill or TerminalNoopSkill(topology=topology)
    if getattr(resolved_coarse, "topology", None) is None:
        resolved_coarse.topology = topology
    if getattr(resolved_fine, "topology", None) is None:
        resolved_fine.topology = topology
    if getattr(resolved_terminal, "topology", None) is None:
        resolved_terminal.topology = topology
    registry.register(
        "coarse_approach",
        resolved_coarse,
        metadata={"family": "relation_skill", "default_registry": True},
    )
    registry.register(
        "fine_dock",
        resolved_fine,
        metadata={"family": "relation_skill", "default_registry": True},
    )
    registry.register(
        "terminal_noop",
        resolved_terminal,
        metadata={"family": "terminal_skill", "default_registry": True},
    )
    return registry

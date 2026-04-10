"""Registry helpers for scheduler transition policies."""

from __future__ import annotations

from dataclasses import dataclass, field

from .transition_policy import (
    DefaultTransitionPolicy,
    FailFastTransitionPolicy,
    TransitionPolicy,
)


@dataclass(slots=True)
class TransitionPolicyDescriptor:
    """Descriptor stored for one registered transition policy."""

    policy_key: str
    metadata: dict[str, object] = field(default_factory=dict)
    registry_source: str | None = None


@dataclass(slots=True)
class TransitionPolicyResolutionResult:
    """Structured lookup result for transition policy resolution."""

    policy_key: str
    found: bool
    policy_instance: TransitionPolicy | None = None
    descriptor: TransitionPolicyDescriptor | None = None
    registry_source: str | None = None
    error: str | None = None

    @property
    def metadata(self) -> dict[str, object]:
        """Return descriptor metadata with a stable empty fallback."""
        if self.descriptor is None:
            return {}
        return dict(self.descriptor.metadata)

    @property
    def resolved_policy_key(self) -> str | None:
        """Return the resolved policy key when the lookup succeeded."""
        if self.descriptor is None:
            return None
        return self.descriptor.policy_key


class TransitionPolicyRegistry:
    """Resolve transition policies without hardcoding scheduler policy instances."""

    def __init__(
        self,
        *,
        source_name: str = "custom_transition_policy_registry",
        default_policy_key: str = "default",
    ) -> None:
        self.source_name = source_name
        self.default_policy_key = default_policy_key
        self._entries: dict[str, tuple[TransitionPolicyDescriptor, TransitionPolicy]] = {}

    def register(
        self,
        policy_key: str,
        policy_instance: TransitionPolicy,
        *,
        metadata: dict[str, object] | None = None,
        override: bool = False,
    ) -> TransitionPolicyDescriptor:
        """Register one transition policy under a stable lookup key."""
        if not override and policy_key in self._entries:
            raise ValueError(f"transition_policy_registry_duplicate:{policy_key}")
        descriptor = TransitionPolicyDescriptor(
            policy_key=policy_key,
            metadata=dict(metadata or {}),
            registry_source=self.source_name,
        )
        self._entries[policy_key] = (descriptor, policy_instance)
        return descriptor

    def resolve(self, policy_key: str | None = None) -> TransitionPolicyResolutionResult:
        """Return a structured resolution result instead of raising on misses."""
        requested_key = self.default_policy_key if policy_key is None else policy_key
        entry = self._entries.get(requested_key)
        if entry is None:
            return TransitionPolicyResolutionResult(
                policy_key=requested_key,
                found=False,
                policy_instance=None,
                descriptor=None,
                registry_source=self.source_name,
                error=f"transition_policy_not_registered:{requested_key}",
            )
        descriptor, policy_instance = entry
        return TransitionPolicyResolutionResult(
            policy_key=requested_key,
            found=True,
            policy_instance=policy_instance,
            descriptor=descriptor,
            registry_source=self.source_name,
            error=None,
        )

    def get(self, policy_key: str | None = None) -> TransitionPolicy | None:
        """Return one registered transition policy instance with nullable fallback."""
        resolution = self.resolve(policy_key)
        return resolution.policy_instance

    def list_registered(self) -> list[TransitionPolicyDescriptor]:
        """Return registered policy descriptors in insertion order."""
        return [descriptor for descriptor, _ in self._entries.values()]


def build_default_transition_policy_registry() -> TransitionPolicyRegistry:
    """Build the default scheduler transition policy registry."""
    registry = TransitionPolicyRegistry(
        source_name="default_transition_policy_registry",
        default_policy_key="default",
    )
    default_policy = DefaultTransitionPolicy()
    fail_fast_policy = FailFastTransitionPolicy()
    registry.register(
        "default",
        default_policy,
        metadata={"family": "transition_policy", "default_registry": True},
    )
    registry.register(
        "retry_then_failure_sink",
        default_policy,
        metadata={
            "family": "transition_policy",
            "default_registry": True,
            "alias_for": "default",
        },
    )
    registry.register(
        "fail_fast",
        fail_fast_policy,
        metadata={"family": "transition_policy", "default_registry": True},
    )
    return registry

"""Registry for resolving relation extractors by exact pair or pair template."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable

from .relation_state import RelationState, RelationType

PairExtractor = Callable[[Any, str, str], RelationState]
PairMatcher = Callable[[str, str, RelationType], bool]


@dataclass(slots=True, frozen=True)
class PairDescriptor:
    """Descriptor for one active/passive relation extractor binding."""

    active_module: str
    passive_module: str
    relation_type: RelationType
    extractor_key: str

    def pair_key(self) -> tuple[str, str, RelationType]:
        """Return the exact pair key used for registry lookups."""
        return (self.active_module, self.passive_module, self.relation_type)


@dataclass(slots=True, frozen=True)
class _TemplateRegistration:
    """Internal registry entry for template-style resolution."""

    descriptor: PairDescriptor
    matcher: PairMatcher
    extractor: PairExtractor
    specificity: int
    registration_index: int


@dataclass(slots=True, frozen=True)
class PairResolution:
    """Fully described extractor resolution result."""

    descriptor: PairDescriptor
    extractor: PairExtractor
    resolution_kind: str
    specificity: int
    registration_index: int
    registry_source: str


class PairExtractorRegistry:
    """Resolve relation extractors using exact or template-style matches."""

    def __init__(self, *, source_name: str = "custom_pair_registry") -> None:
        self.source_name = source_name
        self._exact_extractors: dict[tuple[str, str, RelationType], tuple[PairDescriptor, PairExtractor]] = {}
        self._template_extractors: list[_TemplateRegistration] = []
        self._registration_index = 0

    def register_exact(self, descriptor: PairDescriptor, extractor: PairExtractor) -> PairDescriptor:
        """Register an extractor for one exact active/passive relation."""
        self._exact_extractors[descriptor.pair_key()] = (descriptor, extractor)
        return descriptor

    def register_template(
        self,
        *,
        extractor_key: str,
        relation_type: RelationType,
        matcher: PairMatcher,
        extractor: PairExtractor,
        active_module: str = "*",
        passive_module: str = "*",
        specificity: int | None = None,
    ) -> PairDescriptor:
        """Register an extractor that matches a family of relation pairs."""
        descriptor = PairDescriptor(
            active_module=active_module,
            passive_module=passive_module,
            relation_type=relation_type,
            extractor_key=extractor_key,
        )
        resolved_specificity = specificity
        if resolved_specificity is None:
            resolved_specificity = int(active_module != "*") + int(passive_module != "*")
        self._template_extractors.append(
            _TemplateRegistration(
                descriptor=descriptor,
                matcher=matcher,
                extractor=extractor,
                specificity=resolved_specificity,
                registration_index=self._registration_index,
            )
        )
        self._registration_index += 1
        return descriptor

    def resolve(
        self,
        active_module: str,
        passive_module: str,
        relation_type: RelationType,
    ) -> PairResolution | None:
        """Resolve the highest-priority extractor for a relation request.

        Resolution order is:
        1. Exact descriptor matches.
        2. Template matches sorted by higher specificity first.
        3. For specificity ties, earlier registration order wins.
        """
        exact = self._exact_extractors.get((active_module, passive_module, relation_type))
        if exact is not None:
            descriptor, extractor = exact
            return PairResolution(
                descriptor=descriptor,
                extractor=extractor,
                resolution_kind="exact",
                specificity=100,
                registration_index=-1,
                registry_source=self.source_name,
            )
        matches = [
            registration
            for registration in self._template_extractors
            if registration.matcher(active_module, passive_module, relation_type)
        ]
        if matches:
            registration = sorted(
                matches,
                key=lambda item: (-item.specificity, item.registration_index),
            )[0]
            resolved_descriptor = PairDescriptor(
                active_module=active_module,
                passive_module=passive_module,
                relation_type=relation_type,
                extractor_key=registration.descriptor.extractor_key,
            )
            return PairResolution(
                descriptor=resolved_descriptor,
                extractor=registration.extractor,
                resolution_kind="template",
                specificity=registration.specificity,
                registration_index=registration.registration_index,
                registry_source=self.source_name,
            )
        return None

    def list_registered(self) -> list[PairDescriptor]:
        """Return all registered descriptors in deterministic order."""
        descriptors = [descriptor for descriptor, _ in self._exact_extractors.values()]
        descriptors.extend(registration.descriptor for registration in self._template_extractors)
        return descriptors

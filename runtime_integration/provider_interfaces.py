"""Observation-provider interfaces for runtime-facing control sessions."""

from __future__ import annotations

from typing import Protocol, runtime_checkable

from .observation_types import RuntimeObservationFrame


@runtime_checkable
class ObservationProvider(Protocol):
    """Minimal runtime observation provider contract."""

    def start(self) -> None:
        """Start any provider-owned resources."""

    def stop(self) -> None:
        """Stop any provider-owned resources."""

    def warmup(self) -> RuntimeObservationFrame | None:
        """Optionally fetch an initial frame and prime caches."""

    def is_ready(self) -> bool:
        """Return whether the provider can currently serve frames."""

    def get_latest_frame(self) -> RuntimeObservationFrame | None:
        """Return the most recent runtime observation frame."""


__all__ = ["ObservationProvider"]

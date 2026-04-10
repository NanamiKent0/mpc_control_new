"""Dispatcher interfaces for translating scheduler envelopes into runtime commands."""

from __future__ import annotations

from typing import Protocol, runtime_checkable

from ..control_core.models.runtime_bridge_types import DispatchResult, SchedulerDispatchEnvelope


@runtime_checkable
class CommandDispatcher(Protocol):
    """Minimal dispatcher contract consumed by runtime sessions."""

    def start(self) -> None:
        """Start any dispatcher-owned resources."""

    def stop(self) -> None:
        """Stop any dispatcher-owned resources."""

    def dispatch(self, envelope: SchedulerDispatchEnvelope) -> DispatchResult:
        """Dispatch one scheduler envelope into the target runtime."""

    def emergency_stop(self) -> DispatchResult:
        """Issue a minimal emergency-stop command or a structured rejection."""

    def flush(self) -> DispatchResult:
        """Flush any buffered commands or report that no flushing is needed."""


__all__ = ["CommandDispatcher"]

"""Explicit execution context carried through relation-skill execution."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..topology.chain_topology import ChainTopology


@dataclass(slots=True)
class ExecutionContext:
    """Explicit execution-time context for one skill invocation."""

    topology: ChainTopology | None = None
    dt: float | None = None
    target_ns: int | None = None
    metadata: dict[str, object] = field(default_factory=dict)
    input_source: str | None = None
    frame_timestamp_ns: int | None = None
    topology_source: str | None = None

    def merged_metadata(self, values: dict[str, object] | None = None) -> "ExecutionContext":
        """Return a shallow copy with additional metadata merged in."""
        merged = dict(self.metadata)
        merged.update(dict(values or {}))
        return ExecutionContext(
            topology=self.topology,
            dt=self.dt,
            target_ns=self.target_ns,
            metadata=merged,
            input_source=self.input_source,
            frame_timestamp_ns=self.frame_timestamp_ns,
            topology_source=self.topology_source,
        )

    def with_updates(
        self,
        *,
        topology: "ChainTopology | None" = None,
        dt: float | None = None,
        target_ns: int | None = None,
        metadata: dict[str, object] | None = None,
        input_source: str | None = None,
        frame_timestamp_ns: int | None = None,
        topology_source: str | None = None,
    ) -> "ExecutionContext":
        """Return a shallow copy with selected fields replaced."""
        return ExecutionContext(
            topology=self.topology if topology is None else topology,
            dt=self.dt if dt is None else dt,
            target_ns=self.target_ns if target_ns is None else target_ns,
            metadata=dict(self.metadata if metadata is None else metadata),
            input_source=self.input_source if input_source is None else input_source,
            frame_timestamp_ns=(
                self.frame_timestamp_ns if frame_timestamp_ns is None else frame_timestamp_ns
            ),
            topology_source=self.topology_source if topology_source is None else topology_source,
        )

    def metadata_value(self, key: str, default: object | None = None) -> object | None:
        """Return one metadata value with a stable default."""
        return self.metadata.get(key, default)

    def bridge_metadata(self) -> dict[str, object]:
        """Return a runtime-bridge-friendly context snapshot."""
        payload: dict[str, object] = dict(self.metadata)
        payload.setdefault("context_dt", self.dt)
        payload.setdefault("context_target_ns", self.target_ns)
        payload.setdefault("context_input_source", self.input_source)
        payload.setdefault("context_frame_timestamp_ns", self.frame_timestamp_ns)
        payload.setdefault("context_topology_source", self.topology_source)
        if self.topology is not None:
            payload.setdefault("topology_snapshot", self.topology.snapshot())
            payload.setdefault("frontier_snapshot", self.topology.frontier_snapshot())
            payload.setdefault("support_snapshot", self.topology.support_snapshot())
        return payload

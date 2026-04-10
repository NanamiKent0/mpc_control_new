"""Chain-level topology helpers for topology-aware relation skills."""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(slots=True)
class ChainTopology:
    """Chain topology state with frontier, support, and edge legality helpers."""

    ordered_modules: list[str] = field(default_factory=list)
    coupled_edges: set[tuple[str, str]] = field(default_factory=set)
    blocked_edges: set[tuple[str, str]] = field(default_factory=set)
    grounded_modules: set[str] = field(default_factory=set)
    active_frontier: tuple[str, str] | None = None
    support_modules: set[str] = field(default_factory=set)

    def __post_init__(self) -> None:
        """Normalize undirected edge sets after initialization."""
        self.coupled_edges = {self.normalize_edge(left, right) for left, right in self.coupled_edges}
        self.blocked_edges = {self.normalize_edge(left, right) for left, right in self.blocked_edges}

    @staticmethod
    def normalize_edge(left: str, right: str) -> tuple[str, str]:
        """Return a stable undirected edge key."""
        return tuple(sorted((left, right)))

    def is_coupled(self, left: str, right: str) -> bool:
        """Return whether a pair is marked as coupled."""
        return self.normalize_edge(left, right) in {
            self.normalize_edge(edge_left, edge_right) for edge_left, edge_right in self.coupled_edges
        }

    def set_coupled(self, left: str, right: str, value: bool) -> None:
        """Mark or clear the coupled state for one undirected edge."""
        edge = self.normalize_edge(left, right)
        if value:
            self.coupled_edges.add(edge)
            return
        self.coupled_edges.discard(edge)

    def edge_blocked(self, left: str, right: str) -> bool:
        """Return whether an undirected edge is explicitly blocked."""
        return self.normalize_edge(left, right) in self.blocked_edges

    def set_blocked_edge(self, left: str, right: str, value: bool) -> None:
        """Mark or clear an explicit blocked edge."""
        edge = self.normalize_edge(left, right)
        if value:
            self.blocked_edges.add(edge)
            return
        self.blocked_edges.discard(edge)

    def is_on_active_frontier(self, active: str, passive: str) -> bool:
        """Return whether a pair lies on the configured active frontier."""
        if self.active_frontier is None:
            return True
        return (active, passive) == self.active_frontier or (passive, active) == self.active_frontier

    def set_active_frontier(self, active: str, passive: str) -> None:
        """Set the current active frontier pair."""
        self.active_frontier = (active, passive)

    def clear_active_frontier(self) -> None:
        """Clear the current active frontier pair."""
        self.active_frontier = None

    def is_support_module(self, module_id: str) -> bool:
        """Return whether a module is protected as a support module."""
        return module_id in self.support_modules

    def set_support_module(self, module_id: str, value: bool) -> None:
        """Mark or clear one support-protected module."""
        if value:
            self.support_modules.add(module_id)
            return
        self.support_modules.discard(module_id)

    def is_adjacent(self, left: str, right: str) -> bool:
        """Return whether two modules are adjacent in the current chain order."""
        if left == right:
            return False
        if left not in self.ordered_modules or right not in self.ordered_modules:
            return False
        left_index = self.ordered_modules.index(left)
        right_index = self.ordered_modules.index(right)
        return abs(left_index - right_index) == 1

    def pair_allowed(
        self,
        active: str,
        passive: str,
        *,
        allow_off_frontier: bool = False,
    ) -> tuple[bool, str | None]:
        """Return whether a direct relation skill may target this pair."""
        if active not in self.ordered_modules or passive not in self.ordered_modules:
            return (False, "pair_modules_missing_from_topology")
        if active == passive:
            return (False, "identical_modules_not_allowed")
        if not self.is_adjacent(active, passive):
            return (False, f"non_adjacent_pair:{active}->{passive}")
        if self.edge_blocked(active, passive):
            return (False, f"blocked_edge:{active}->{passive}")
        if self.active_frontier is not None and not allow_off_frontier and not self.is_on_active_frontier(active, passive):
            return (False, f"off_frontier_pair:{active}->{passive}")
        return (True, None)

    def is_grounded(self, module_id: str) -> bool:
        """Return whether a module is grounded."""
        return module_id in self.grounded_modules

    def frontier_snapshot(self) -> dict[str, float | bool | str | None]:
        """Return the frontier-specific diagnostic snapshot."""
        frontier = None
        if self.active_frontier is not None:
            frontier = f"{self.active_frontier[0]}->{self.active_frontier[1]}"
        return {
            "topology_active_frontier": frontier,
            "topology_frontier_enforced": bool(self.active_frontier is not None),
        }

    def support_snapshot(self) -> dict[str, float | bool | str | None]:
        """Return support and blocking diagnostics."""
        blocked_edges = ",".join(
            f"{left}<->{right}" for left, right in sorted(self.blocked_edges)
        ) or None
        support_modules = ",".join(sorted(self.support_modules)) or None
        grounded_modules = ",".join(sorted(self.grounded_modules)) or None
        return {
            "topology_blocked_edges": blocked_edges,
            "topology_support_modules": support_modules,
            "topology_grounded_modules": grounded_modules,
        }

    def snapshot(self) -> dict[str, float | bool | str | None]:
        """Return a lightweight serializable snapshot for skill diagnostics."""
        normalized_edges = sorted(self.normalize_edge(left, right) for left, right in self.coupled_edges)
        coupled_edges = ",".join(f"{left}<->{right}" for left, right in normalized_edges) or None
        ordered_modules = ",".join(self.ordered_modules) or None
        return {
            "topology_ordered_modules": ordered_modules,
            "topology_coupled_edges": coupled_edges,
            **self.frontier_snapshot(),
            **self.support_snapshot(),
        }

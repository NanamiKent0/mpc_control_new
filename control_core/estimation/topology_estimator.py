"""Topology estimation derived from runtime hints plus relation estimates."""

from __future__ import annotations

from collections.abc import Iterable, Mapping
from dataclasses import dataclass, field

from ..topology.chain_topology import ChainTopology
from ..topology.relation_state import RelationState
from .module_state_estimator import resolve_estimation_module_order
from ...runtime_integration.observation_types import RuntimeObservationFrame
from ...runtime_integration.runtime_state_builder import build_topology_from_frame

TOPOLOGY_ESTIMATOR_SOURCE = "control_core.estimation.topology_estimator"
TopologyDiagnosticValue = float | bool | str | None


@dataclass(slots=True)
class TopologyEstimate:
    """Serializable topology estimate used by scheduling layers."""

    ordered_modules: tuple[str, ...]
    coupled_edges: tuple[tuple[str, str], ...] = ()
    active_frontier: tuple[str, str] | None = None
    blocked_edges: tuple[tuple[str, str], ...] = ()
    topology_valid: bool = False
    diagnostics: dict[str, TopologyDiagnosticValue] = field(default_factory=dict)
    support_modules: tuple[str, ...] = ()
    grounded_modules: tuple[str, ...] = ()

    def to_chain_topology(self) -> ChainTopology:
        """Convert the estimate into the shared `ChainTopology` model."""
        return ChainTopology(
            ordered_modules=list(self.ordered_modules),
            coupled_edges=set(self.coupled_edges),
            blocked_edges=set(self.blocked_edges),
            grounded_modules=set(self.grounded_modules),
            active_frontier=self.active_frontier,
            support_modules=set(self.support_modules),
        )


class TopologyEstimator:
    """Estimate scheduler-facing topology from hints and relation state."""

    def estimate(
        self,
        frame: RuntimeObservationFrame,
        *,
        relation_states: Mapping[str, RelationState] | None = None,
        ordered_modules: Iterable[str] | None = None,
    ) -> TopologyEstimate:
        """Return the current topology estimate."""
        resolved_order = resolve_estimation_module_order(frame, ordered_modules=ordered_modules)
        topology = build_topology_from_frame(
            frame,
            fallback_topology=ChainTopology(ordered_modules=list(resolved_order)),
        )
        if resolved_order:
            topology.ordered_modules = list(resolved_order)
        if relation_states:
            for relation_state in relation_states.values():
                if relation_state.coupled is None or not _trusted_coupling_state(relation_state):
                    continue
                topology.set_coupled(
                    relation_state.active_module,
                    relation_state.passive_module,
                    relation_state.coupled,
                )
        coupled_edges = tuple(sorted(topology.coupled_edges))
        blocked_edges = tuple(sorted(topology.blocked_edges))
        support_modules = tuple(sorted(topology.support_modules))
        grounded_modules = tuple(sorted(topology.grounded_modules))
        topology_valid = _topology_valid(
            ordered_modules=tuple(topology.ordered_modules),
            coupled_edges=coupled_edges,
            blocked_edges=blocked_edges,
            active_frontier=topology.active_frontier,
        )
        return TopologyEstimate(
            ordered_modules=tuple(topology.ordered_modules),
            coupled_edges=coupled_edges,
            active_frontier=topology.active_frontier,
            blocked_edges=blocked_edges,
            topology_valid=topology_valid,
            support_modules=support_modules,
            grounded_modules=grounded_modules,
            diagnostics={
                "topology_source": TOPOLOGY_ESTIMATOR_SOURCE,
                "ordered_module_count": float(len(topology.ordered_modules)),
                "coupled_edge_count": float(len(coupled_edges)),
                "blocked_edge_count": float(len(blocked_edges)),
                "support_module_count": float(len(support_modules)),
                "grounded_module_count": float(len(grounded_modules)),
                "has_active_frontier": topology.active_frontier is not None,
            },
        )


def _trusted_coupling_state(relation_state: RelationState) -> bool:
    runtime_observation_valid = relation_state.diagnostics.get("runtime_observation_valid")
    if isinstance(runtime_observation_valid, bool):
        return runtime_observation_valid
    return relation_state.observation_valid


def _topology_valid(
    *,
    ordered_modules: tuple[str, ...],
    coupled_edges: tuple[tuple[str, str], ...],
    blocked_edges: tuple[tuple[str, str], ...],
    active_frontier: tuple[str, str] | None,
) -> bool:
    if not ordered_modules:
        return False
    ordered_set = set(ordered_modules)
    if len(ordered_set) != len(ordered_modules):
        return False
    for left, right in (*coupled_edges, *blocked_edges):
        if left not in ordered_set or right not in ordered_set:
            return False
    if active_frontier is not None:
        left, right = active_frontier
        if left not in ordered_set or right not in ordered_set:
            return False
    return True


__all__ = [
    "TOPOLOGY_ESTIMATOR_SOURCE",
    "TopologyDiagnosticValue",
    "TopologyEstimate",
    "TopologyEstimator",
]

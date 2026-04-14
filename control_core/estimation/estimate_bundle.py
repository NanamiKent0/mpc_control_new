"""Unified estimation bundle for scheduler-facing semantic state."""

from __future__ import annotations

from collections.abc import Iterable, Mapping
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

from ..kinematics import ModuleGeometryConfig, compute_chain_snapshot
from ..kinematics.frame_conventions import pair_key
from ..models.module_state import ModuleState
from ..topology.relation_state import RelationState
from .availability_estimator import (
    AvailabilityEstimator,
    JointAvailability,
)
from .module_state_estimator import ModuleStateEstimator, resolve_estimation_module_order
from .relation_estimator import RelationEstimator
from .topology_estimator import TopologyEstimate, TopologyEstimator
from ...runtime_integration.observation_types import RuntimeObservationFrame

if TYPE_CHECKING:
    from ..kinematics import ChainSnapshot


ESTIMATE_BUNDLE_SOURCE = "control_core.estimation.estimate_bundle"


@dataclass(slots=True)
class EstimateBundle:
    """Unified semantic estimate emitted from runtime observation + kinematics."""

    timestamp_ns: int
    module_states: dict[str, ModuleState]
    relation_states: dict[str, RelationState]
    joint_availability: dict[str, JointAvailability]
    topology_estimate: TopologyEstimate
    diagnostics: dict[str, object] = field(default_factory=dict)
    chain_snapshot: "ChainSnapshot | None" = None

    def get_module_state(self, module_id: str) -> ModuleState | None:
        """Return one module state by module identifier."""
        return self.module_states.get(str(module_id))

    def get_relation_state(self, active_module: str, passive_module: str) -> RelationState | None:
        """Return one relation state by directed active/passive pair."""
        return self.relation_states.get(pair_key(active_module, passive_module))

    def get_joint_availability(self, joint_id: str) -> JointAvailability | None:
        """Return one joint availability record."""
        return self.joint_availability.get(str(joint_id))


def build_estimate_bundle(
    frame: RuntimeObservationFrame,
    *,
    ordered_modules: Iterable[str] | None = None,
    reference_timestamp_ns: int | None = None,
    geometry_config: ModuleGeometryConfig | None = None,
    module_state_estimator: ModuleStateEstimator | None = None,
    relation_estimator: RelationEstimator | None = None,
    availability_estimator: AvailabilityEstimator | None = None,
    topology_estimator: TopologyEstimator | None = None,
) -> EstimateBundle:
    """Build the full estimation bundle for one runtime frame."""
    resolved_order = resolve_estimation_module_order(frame, ordered_modules=ordered_modules)
    module_estimator = module_state_estimator or ModuleStateEstimator()
    module_states = module_estimator.estimate(
        frame,
        ordered_modules=resolved_order,
        reference_timestamp_ns=reference_timestamp_ns,
    )
    chain_snapshot = compute_chain_snapshot(
        module_states,
        ordered_modules=resolved_order,
        geometry_config=geometry_config,
    )
    module_states = module_estimator.estimate(
        frame,
        ordered_modules=resolved_order,
        chain_snapshot=chain_snapshot,
        reference_timestamp_ns=reference_timestamp_ns,
    )
    relation_states = (relation_estimator or RelationEstimator()).estimate(
        frame,
        module_states=module_states,
        chain_snapshot=chain_snapshot,
        ordered_modules=resolved_order,
        reference_timestamp_ns=reference_timestamp_ns,
    )
    topology_estimate = (topology_estimator or TopologyEstimator()).estimate(
        frame,
        relation_states=relation_states,
        ordered_modules=resolved_order,
    )
    joint_availability = (availability_estimator or AvailabilityEstimator()).estimate(
        module_states,
        frame_timestamp_ns=frame.timestamp_ns,
        reference_timestamp_ns=reference_timestamp_ns,
    )
    return EstimateBundle(
        timestamp_ns=frame.timestamp_ns,
        module_states=module_states,
        relation_states=relation_states,
        joint_availability=joint_availability,
        topology_estimate=topology_estimate,
        chain_snapshot=chain_snapshot,
        diagnostics={
            "estimate_bundle_source": ESTIMATE_BUNDLE_SOURCE,
            "timestamp_ns": frame.timestamp_ns,
            "ordered_modules": list(resolved_order),
            "module_state_count": len(module_states),
            "relation_state_count": len(relation_states),
            "joint_availability_count": len(joint_availability),
            "topology_valid": topology_estimate.topology_valid,
            "kinematics_snapshot_available": chain_snapshot is not None,
        },
    )


__all__ = [
    "ESTIMATE_BUNDLE_SOURCE",
    "EstimateBundle",
    "build_estimate_bundle",
]

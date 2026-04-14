"""Relation-state estimation that fuses runtime pair data with kinematics geometry."""

from __future__ import annotations

from collections.abc import Iterable, Mapping
from dataclasses import dataclass
import math

from ..kinematics import ChainSnapshot
from ..kinematics.frame_conventions import adjacent_active_passive_pairs, pair_key
from ..models.module_state import ModuleState
from ..topology.relation_state import RelationState, RelationType
from .module_state_estimator import (
    DEFAULT_ESTIMATE_FRESHNESS_TIMEOUT_NS,
    derive_module_estimate_validity,
    resolve_estimation_module_order,
)
from ...runtime_integration.observation_types import RuntimeObservationFrame

RELATION_ESTIMATOR_SOURCE = "control_core.estimation.relation_estimator"


@dataclass(frozen=True, slots=True)
class RelationEstimatorConfig:
    """Configuration for relation estimation."""

    freshness_timeout_ns: int = DEFAULT_ESTIMATE_FRESHNESS_TIMEOUT_NS
    allow_kinematics_fallback: bool = True


class RelationEstimator:
    """Estimate adjacent relation state from runtime observations and geometry."""

    def __init__(self, config: RelationEstimatorConfig | None = None) -> None:
        self.config = config or RelationEstimatorConfig()

    def estimate(
        self,
        frame: RuntimeObservationFrame,
        *,
        module_states: Mapping[str, ModuleState],
        chain_snapshot: ChainSnapshot,
        ordered_modules: Iterable[str] | None = None,
        reference_timestamp_ns: int | None = None,
    ) -> dict[str, RelationState]:
        """Return adjacent pair relation state estimates."""
        resolved_order = resolve_estimation_module_order(
            frame,
            ordered_modules=ordered_modules,
            chain_snapshot=chain_snapshot,
        )
        estimated_relations: dict[str, RelationState] = {}
        for active_module, passive_module in adjacent_active_passive_pairs(resolved_order):
            pair_state = self._estimate_pair(
                frame,
                active_module=active_module,
                passive_module=passive_module,
                module_states=module_states,
                chain_snapshot=chain_snapshot,
                reference_timestamp_ns=reference_timestamp_ns,
            )
            estimated_relations[pair_key(active_module, passive_module)] = pair_state
        return estimated_relations

    def _estimate_pair(
        self,
        frame: RuntimeObservationFrame,
        *,
        active_module: str,
        passive_module: str,
        module_states: Mapping[str, ModuleState],
        chain_snapshot: ChainSnapshot,
        reference_timestamp_ns: int | None,
    ) -> RelationState:
        relation_type = _relation_type_for_pair(active_module, passive_module)
        pair_observation = frame.get_pair_observation(active_module, passive_module)
        pair_geometry = chain_snapshot.get_pair_geometry(active_module, passive_module)
        runtime_observation_valid = bool(pair_observation and pair_observation.observation_valid)
        geometry_inputs_valid = _geometry_inputs_valid(
            module_states,
            pair=(active_module, passive_module),
            frame_timestamp_ns=frame.timestamp_ns,
            reference_timestamp_ns=reference_timestamp_ns,
            freshness_timeout_ns=self.config.freshness_timeout_ns,
        )
        geometry_valid = bool(
            self.config.allow_kinematics_fallback
            and pair_geometry is not None
            and geometry_inputs_valid
            and _finite_optional_scalar(pair_geometry.distance_mm)
            and _finite_optional_scalar(pair_geometry.orientation_error_deg)
        )

        distance_mm, distance_source = _resolve_metric(
            runtime_value=None if pair_observation is None else pair_observation.distance_mm,
            runtime_valid=runtime_observation_valid,
            geometry_value=None if pair_geometry is None else pair_geometry.distance_mm,
            geometry_valid=geometry_valid,
            metric_name="distance_mm",
        )
        orientation_error_deg, orientation_source = _resolve_metric(
            runtime_value=None if pair_observation is None else pair_observation.orientation_error_deg,
            runtime_valid=runtime_observation_valid,
            geometry_value=None if pair_geometry is None else pair_geometry.orientation_error_deg,
            geometry_valid=geometry_valid,
            metric_name="orientation_error_deg",
        )
        relation_valid = (
            distance_mm is not None
            and orientation_error_deg is not None
            and (runtime_observation_valid or geometry_valid)
        )
        coupled = None if pair_observation is None else pair_observation.coupled
        diagnostics = {
            "relation_source": RELATION_ESTIMATOR_SOURCE,
            "pair_key": pair_key(active_module, passive_module),
            "runtime_pair_present": pair_observation is not None,
            "runtime_observation_valid": runtime_observation_valid,
            "geometry_inputs_valid": geometry_inputs_valid,
            "kinematics_pair_geometry_available": pair_geometry is not None,
            "kinematics_pair_geometry_valid": geometry_valid,
            "distance_source": distance_source,
            "orientation_source": orientation_source,
            "coupled_source": (
                None
                if coupled is None
                else "runtime_pair_observation"
            ),
            "relation_validity_source": _relation_validity_source(
                runtime_observation_valid=runtime_observation_valid,
                geometry_valid=geometry_valid,
                distance_source=distance_source,
                orientation_source=orientation_source,
            ),
        }
        return RelationState(
            active_module=active_module,
            passive_module=passive_module,
            relation_type=relation_type,
            distance_mm=distance_mm,
            orientation_error_deg=orientation_error_deg,
            coupled=coupled,
            observation_valid=relation_valid,
            diagnostics=diagnostics,
        )


def _geometry_inputs_valid(
    module_states: Mapping[str, ModuleState],
    *,
    pair: tuple[str, str],
    frame_timestamp_ns: int,
    reference_timestamp_ns: int | None,
    freshness_timeout_ns: int,
) -> bool:
    for module_id in pair:
        module_state = module_states.get(module_id)
        if module_state is None:
            return False
        validity = derive_module_estimate_validity(
            module_state,
            frame_timestamp_ns=frame_timestamp_ns,
            reference_timestamp_ns=reference_timestamp_ns,
            freshness_timeout_ns=freshness_timeout_ns,
            required_dof_keys=_required_geometry_dofs_for_module(module_state),
        )
        if not validity.estimate_valid:
            return False
    return True


def _required_geometry_dofs_for_module(module_state: ModuleState) -> tuple[str, ...]:
    if module_state.module_type == "tip":
        return ("growth_mm",)
    if module_state.module_type == "joint":
        return ("crawl_mm", "bend_deg", "rotate_deg")
    return ()


def _resolve_metric(
    *,
    runtime_value: float | None,
    runtime_valid: bool,
    geometry_value: float | None,
    geometry_valid: bool,
    metric_name: str,
) -> tuple[float | None, str]:
    if runtime_valid and _finite_optional_scalar(runtime_value):
        return (float(runtime_value), "runtime_pair_observation")
    if geometry_valid and _finite_optional_scalar(geometry_value):
        return (float(geometry_value), "kinematics_pair_geometry")
    if _finite_optional_scalar(runtime_value):
        return (float(runtime_value), "runtime_pair_observation_unvalidated")
    if _finite_optional_scalar(geometry_value):
        return (float(geometry_value), "kinematics_pair_geometry_unvalidated")
    return (None, f"{metric_name}_unavailable")


def _relation_type_for_pair(active_module: str, passive_module: str) -> RelationType:
    if active_module == "tip" or passive_module == "tip":
        return "tip_joint"
    return "joint_joint"


def _relation_validity_source(
    *,
    runtime_observation_valid: bool,
    geometry_valid: bool,
    distance_source: str,
    orientation_source: str,
) -> str:
    if runtime_observation_valid and geometry_valid:
        if distance_source == "runtime_pair_observation" and orientation_source == "runtime_pair_observation":
            return "runtime_pair_observation"
        if distance_source == "kinematics_pair_geometry" and orientation_source == "kinematics_pair_geometry":
            return "kinematics_pair_geometry"
        return "mixed_runtime_and_kinematics"
    if runtime_observation_valid:
        return "runtime_pair_observation"
    if geometry_valid:
        return "kinematics_pair_geometry"
    return "relation_invalid"


def _finite_optional_scalar(value: object) -> bool:
    if value is None:
        return False
    try:
        return bool(math.isfinite(float(value)))
    except (TypeError, ValueError):
        return False


__all__ = [
    "RELATION_ESTIMATOR_SOURCE",
    "RelationEstimator",
    "RelationEstimatorConfig",
]

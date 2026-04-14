"""Module-state estimation built from runtime observations plus geometry context."""

from __future__ import annotations

from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass
import math

from ..kinematics import ChainSnapshot
from ..kinematics.frame_conventions import canonical_module_order
from ..models.module_state import ModuleState
from ...runtime_integration.observation_types import RuntimeObservationFrame
from ...runtime_integration.runtime_state_builder import build_module_state_for_module

MODULE_STATE_ESTIMATOR_SOURCE = "control_core.estimation.module_state_estimator"
DEFAULT_ESTIMATE_FRESHNESS_TIMEOUT_NS = 1_000_000_000


@dataclass(frozen=True, slots=True)
class ModuleEstimateValidity:
    """Normalized validity decision for one module estimate."""

    estimate_valid: bool
    estimate_fresh: bool
    state_present: bool
    values_finite: bool
    required_dofs_present: bool
    age_ns: int | None
    invalid_fields: tuple[str, ...] = ()
    missing_required_dofs: tuple[str, ...] = ()

    @property
    def age_ms(self) -> float | None:
        """Return the estimate age in milliseconds when known."""
        if self.age_ns is None:
            return None
        return float(self.age_ns) / 1_000_000.0


@dataclass(frozen=True, slots=True)
class ModuleStateEstimatorConfig:
    """Configuration for the module-state estimator."""

    freshness_timeout_ns: int = DEFAULT_ESTIMATE_FRESHNESS_TIMEOUT_NS


def derive_module_estimate_validity(
    module_state: ModuleState,
    *,
    frame_timestamp_ns: int | None,
    reference_timestamp_ns: int | None = None,
    freshness_timeout_ns: int = DEFAULT_ESTIMATE_FRESHNESS_TIMEOUT_NS,
    required_dof_keys: Iterable[str] = (),
) -> ModuleEstimateValidity:
    """Return a conservative validity decision for one module state."""
    resolved_required_dofs = tuple(str(key) for key in required_dof_keys)
    invalid_fields = tuple(_invalid_numeric_fields(module_state))
    missing_required_dofs = tuple(
        key
        for key in resolved_required_dofs
        if key not in module_state.dofs or not _finite_scalar(module_state.dofs.get(key))
    )
    state_present = bool(module_state.dofs or module_state.velocities or module_state.attach_state)
    values_finite = not invalid_fields
    required_dofs_present = not missing_required_dofs
    age_ns = _estimate_age_ns(
        frame_timestamp_ns=frame_timestamp_ns,
        reference_timestamp_ns=reference_timestamp_ns,
    )
    estimate_fresh = age_ns is not None and age_ns <= int(freshness_timeout_ns)
    estimate_valid = state_present and values_finite and required_dofs_present and estimate_fresh
    return ModuleEstimateValidity(
        estimate_valid=estimate_valid,
        estimate_fresh=estimate_fresh,
        state_present=state_present,
        values_finite=values_finite,
        required_dofs_present=required_dofs_present,
        age_ns=age_ns,
        invalid_fields=invalid_fields,
        missing_required_dofs=missing_required_dofs,
    )


def resolve_estimation_module_order(
    frame: RuntimeObservationFrame,
    *,
    ordered_modules: Iterable[str] | None = None,
    chain_snapshot: ChainSnapshot | None = None,
) -> tuple[str, ...]:
    """Resolve one stable module order for the estimation layer."""
    ordered: list[str] = []
    if ordered_modules is not None:
        ordered.extend(str(module_id) for module_id in ordered_modules)
    elif chain_snapshot is not None:
        ordered.extend(str(module_id) for module_id in chain_snapshot.ordered_modules)
    else:
        hinted_modules = (frame.topology_hint or {}).get("ordered_modules")
        if isinstance(hinted_modules, (list, tuple)):
            ordered.extend(str(module_id) for module_id in hinted_modules)

    extras: set[str] = set(frame.module_observations)
    for observation in frame.pair_observations.values():
        extras.add(str(observation.active_module))
        extras.add(str(observation.passive_module))
    if chain_snapshot is not None:
        extras.update(str(module_id) for module_id in chain_snapshot.ordered_modules)

    resolved_order: list[str] = []
    seen: set[str] = set()
    for module_id in ordered:
        if module_id in seen:
            continue
        seen.add(module_id)
        resolved_order.append(module_id)
    for module_id in canonical_module_order(extras):
        if module_id in seen:
            continue
        seen.add(module_id)
        resolved_order.append(module_id)
    return tuple(resolved_order)


class ModuleStateEstimator:
    """Estimate per-module semantic state from runtime observations."""

    def __init__(self, config: ModuleStateEstimatorConfig | None = None) -> None:
        self.config = config or ModuleStateEstimatorConfig()

    def estimate(
        self,
        frame: RuntimeObservationFrame,
        *,
        ordered_modules: Iterable[str] | None = None,
        chain_snapshot: ChainSnapshot | None = None,
        reference_timestamp_ns: int | None = None,
    ) -> dict[str, ModuleState]:
        """Return estimated module states for the current frame."""
        resolved_order = resolve_estimation_module_order(
            frame,
            ordered_modules=ordered_modules,
            chain_snapshot=chain_snapshot,
        )
        estimated_states: dict[str, ModuleState] = {}
        for module_id in resolved_order:
            base_state = build_module_state_for_module(frame, module_id)
            validity = derive_module_estimate_validity(
                base_state,
                frame_timestamp_ns=frame.timestamp_ns,
                reference_timestamp_ns=reference_timestamp_ns,
                freshness_timeout_ns=self.config.freshness_timeout_ns,
            )
            metadata = dict(base_state.metadata)
            metadata.update(
                {
                    "estimate_source": MODULE_STATE_ESTIMATOR_SOURCE,
                    "estimate_timestamp_ns": float(frame.timestamp_ns),
                    "estimate_age_ms": validity.age_ms,
                    "estimate_fresh": validity.estimate_fresh,
                    "estimate_valid": validity.estimate_valid,
                    "estimate_state_present": validity.state_present,
                    "estimate_values_finite": validity.values_finite,
                    "estimate_invalid_fields": ",".join(validity.invalid_fields) or None,
                    "estimate_pose_available": (
                        False
                        if chain_snapshot is None
                        else module_id in chain_snapshot.module_poses
                    ),
                    "module_observation_present": frame.get_module_observation(module_id) is not None,
                }
            )
            metadata.update(_coupling_metadata_for_module(module_id, frame))
            notes = list(base_state.notes)
            if not validity.state_present:
                notes.append("module_state_missing")
            if not validity.values_finite:
                notes.append("module_state_contains_non_finite_values")
            if not validity.estimate_fresh:
                notes.append("module_state_stale")
            estimated_states[module_id] = ModuleState(
                module_id=base_state.module_id,
                module_type=base_state.module_type,
                dofs=dict(base_state.dofs),
                velocities=dict(base_state.velocities),
                attach_state=dict(base_state.attach_state),
                metadata=metadata,
                notes=_deduped_notes(notes),
            )
        return estimated_states


def _coupling_metadata_for_module(
    module_id: str,
    frame: RuntimeObservationFrame,
) -> dict[str, bool | int | str | None]:
    true_coupled_neighbors: list[str] = []
    metadata: dict[str, bool | int | str | None] = {}
    for observation in frame.pair_observations.values():
        if module_id != observation.active_module and module_id != observation.passive_module:
            continue
        neighbor = (
            observation.passive_module
            if module_id == observation.active_module
            else observation.active_module
        )
        if observation.coupled is None:
            continue
        metadata[f"coupled_to_{neighbor}"] = bool(observation.coupled)
        if observation.coupled:
            true_coupled_neighbors.append(neighbor)
    metadata["coupled_neighbor_count"] = len(true_coupled_neighbors)
    metadata["coupled_neighbors"] = ",".join(sorted(true_coupled_neighbors)) or None
    return metadata


def _invalid_numeric_fields(module_state: ModuleState) -> list[str]:
    invalid_fields: list[str] = []
    for name, value in module_state.dofs.items():
        if not _finite_scalar(value):
            invalid_fields.append(f"dofs.{name}")
    for name, value in module_state.velocities.items():
        if not _finite_scalar(value):
            invalid_fields.append(f"velocities.{name}")
    return invalid_fields


def _estimate_age_ns(
    *,
    frame_timestamp_ns: int | None,
    reference_timestamp_ns: int | None,
) -> int | None:
    if frame_timestamp_ns is None:
        return None
    resolved_reference_ns = (
        int(frame_timestamp_ns)
        if reference_timestamp_ns is None
        else int(reference_timestamp_ns)
    )
    return max(0, resolved_reference_ns - int(frame_timestamp_ns))


def _finite_scalar(value: object) -> bool:
    if value is None:
        return False
    try:
        return bool(math.isfinite(float(value)))
    except (TypeError, ValueError):
        return False


def _deduped_notes(notes: Sequence[str]) -> list[str]:
    deduped: list[str] = []
    seen: set[str] = set()
    for note in notes:
        if note in seen:
            continue
        seen.add(note)
        deduped.append(note)
    return deduped


__all__ = [
    "DEFAULT_ESTIMATE_FRESHNESS_TIMEOUT_NS",
    "MODULE_STATE_ESTIMATOR_SOURCE",
    "ModuleEstimateValidity",
    "ModuleStateEstimator",
    "ModuleStateEstimatorConfig",
    "derive_module_estimate_validity",
    "resolve_estimation_module_order",
]

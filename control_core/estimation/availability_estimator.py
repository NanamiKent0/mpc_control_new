"""Joint availability estimation for scheduling-friendly idle selection."""

from __future__ import annotations

from collections.abc import Iterable, Mapping
from dataclasses import dataclass, field
import math

from ..kinematics.frame_conventions import canonical_module_order
from ..models.module_state import ModuleState
from .module_state_estimator import (
    DEFAULT_ESTIMATE_FRESHNESS_TIMEOUT_NS,
    derive_module_estimate_validity,
)

AVAILABILITY_ESTIMATOR_SOURCE = "control_core.estimation.availability_estimator"


@dataclass(slots=True)
class JointAvailability:
    """Idle/availability estimate for one joint module."""

    joint_id: str
    is_idle: bool
    bend_near_zero: bool
    motion_quiet: bool
    estimate_valid: bool
    reason: str | None = None
    diagnostics: dict[str, float | bool | str | None] = field(default_factory=dict)


@dataclass(frozen=True, slots=True)
class JointAvailabilityConfig:
    """Thresholds used by the joint availability estimator."""

    bend_idle_threshold_deg: float = 5.0
    crawl_quiet_threshold_mm_s: float = 1.0
    rotate_quiet_threshold_deg_s: float = 5.0
    bend_quiet_threshold_deg_s: float = 5.0
    freshness_timeout_ns: int = DEFAULT_ESTIMATE_FRESHNESS_TIMEOUT_NS


class AvailabilityEstimator:
    """Estimate which joints are currently idle and scheduler-eligible."""

    def __init__(self, config: JointAvailabilityConfig | None = None) -> None:
        self.config = config or JointAvailabilityConfig()

    def estimate(
        self,
        module_states: Mapping[str, ModuleState],
        *,
        frame_timestamp_ns: int | None = None,
        reference_timestamp_ns: int | None = None,
        joint_ids: Iterable[str] | None = None,
    ) -> dict[str, JointAvailability]:
        """Return one availability estimate for each requested joint."""
        resolved_joint_ids = _resolved_joint_ids(module_states, joint_ids=joint_ids)
        estimates: dict[str, JointAvailability] = {}
        for joint_id in resolved_joint_ids:
            module_state = module_states[joint_id]
            bend_value = module_state.dofs.get("bend_deg")
            bend_near_zero = _finite_scalar(bend_value) and abs(float(bend_value)) <= float(self.config.bend_idle_threshold_deg)
            motion_quiet, motion_source = self._estimate_motion_quiet(module_state)
            estimate_valid = self._estimate_valid(
                module_state,
                frame_timestamp_ns=frame_timestamp_ns,
                reference_timestamp_ns=reference_timestamp_ns,
            )
            is_idle = bool(bend_near_zero and motion_quiet and estimate_valid)
            reason = _availability_reason(
                bend_near_zero=bend_near_zero,
                motion_quiet=motion_quiet,
                estimate_valid=estimate_valid,
            )
            estimates[joint_id] = JointAvailability(
                joint_id=joint_id,
                is_idle=is_idle,
                bend_near_zero=bend_near_zero,
                motion_quiet=motion_quiet,
                estimate_valid=estimate_valid,
                reason=reason,
                diagnostics={
                    "availability_source": AVAILABILITY_ESTIMATOR_SOURCE,
                    "bend_deg": None if not _finite_scalar(bend_value) else float(bend_value),
                    "bend_idle_threshold_deg": float(self.config.bend_idle_threshold_deg),
                    "motion_quiet_source": motion_source,
                    "motion_velocity_observed": _has_motion_velocity_signal(module_state),
                    "estimate_valid_source": (
                        "module_state_metadata"
                        if isinstance(module_state.metadata.get("estimate_valid"), bool)
                        else "derived_from_module_state"
                    ),
                    "estimate_age_ms": _metadata_float(module_state.metadata.get("estimate_age_ms")),
                },
            )
        return estimates

    def _estimate_valid(
        self,
        module_state: ModuleState,
        *,
        frame_timestamp_ns: int | None,
        reference_timestamp_ns: int | None,
    ) -> bool:
        metadata_valid = module_state.metadata.get("estimate_valid")
        if isinstance(metadata_valid, bool):
            return bool(metadata_valid and _finite_scalar(module_state.dofs.get("bend_deg")))
        bend_validity = derive_module_estimate_validity(
            module_state,
            frame_timestamp_ns=frame_timestamp_ns,
            reference_timestamp_ns=reference_timestamp_ns,
            freshness_timeout_ns=self.config.freshness_timeout_ns,
            required_dof_keys=("bend_deg",),
        )
        return bool(bend_validity.estimate_valid)

    def _estimate_motion_quiet(self, module_state: ModuleState) -> tuple[bool, str]:
        if _has_motion_velocity_signal(module_state):
            thresholds = {
                "crawl_mm_s": float(self.config.crawl_quiet_threshold_mm_s),
                "rotate_deg_s": float(self.config.rotate_quiet_threshold_deg_s),
                "bend_deg_s": float(self.config.bend_quiet_threshold_deg_s),
            }
            for key, threshold in thresholds.items():
                value = module_state.velocities.get(key)
                if value is None:
                    continue
                if not _finite_scalar(value):
                    return (False, "velocity_signal_non_finite")
                if abs(float(value)) > threshold:
                    return (False, "velocity_threshold_exceeded")
            return (True, "velocity_threshold")
        metadata_is_moving = module_state.metadata.get("is_moving")
        if isinstance(metadata_is_moving, bool):
            return (not metadata_is_moving, "metadata_is_moving")
        return (True, "degraded_no_motion_signal")


def _resolved_joint_ids(
    module_states: Mapping[str, ModuleState],
    *,
    joint_ids: Iterable[str] | None,
) -> tuple[str, ...]:
    if joint_ids is not None:
        return tuple(
            str(joint_id)
            for joint_id in joint_ids
            if str(joint_id) in module_states and module_states[str(joint_id)].module_type == "joint"
        )
    return tuple(
        module_id
        for module_id in canonical_module_order(module_states)
        if module_states[module_id].module_type == "joint"
    )


def _availability_reason(
    *,
    bend_near_zero: bool,
    motion_quiet: bool,
    estimate_valid: bool,
) -> str:
    reasons: list[str] = []
    if not estimate_valid:
        reasons.append("estimate_invalid")
    if not bend_near_zero:
        reasons.append("bend_not_near_zero")
    if not motion_quiet:
        reasons.append("motion_not_quiet")
    if not reasons:
        return "idle"
    return "|".join(reasons)


def _has_motion_velocity_signal(module_state: ModuleState) -> bool:
    return any(
        key in module_state.velocities
        for key in ("crawl_mm_s", "rotate_deg_s", "bend_deg_s")
    )


def _finite_scalar(value: object) -> bool:
    if value is None:
        return False
    try:
        return bool(math.isfinite(float(value)))
    except (TypeError, ValueError):
        return False


def _metadata_float(value: object) -> float | None:
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


__all__ = [
    "AVAILABILITY_ESTIMATOR_SOURCE",
    "AvailabilityEstimator",
    "JointAvailability",
    "JointAvailabilityConfig",
]

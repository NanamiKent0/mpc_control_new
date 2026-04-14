"""Generic pair-docking MPC for distance and orientation closure."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping


@dataclass(slots=True)
class PairDockMPCOutput:
    """Output of the generic pair docking controller."""

    active: bool
    feed_velocity_mm_s: float
    rotate_velocity_deg_s: float
    distance_error_mm: float
    orientation_error_deg: float
    feed_active: bool
    rotate_active: bool
    notes: list[str] = field(default_factory=list)

    @property
    def crawl_velocity_mm_s(self) -> float:
        """Compatibility alias for legacy crawl-oriented callers."""
        return self.feed_velocity_mm_s


class PairDockMPC:
    """Minimal deterministic MPC placeholder for generic pair docking."""

    def compute(
        self,
        current_distance_mm: float,
        current_orientation_error_deg: float,
        distance_ref_mm: float,
        orientation_ref_deg: float,
        dt: float,
        limits: Mapping[str, Any] | None = None,
        config: Mapping[str, Any] | None = None,
        *,
        enable_orientation: bool = True,
    ) -> PairDockMPCOutput:
        """Compute feed and rotate corrections toward the docking target."""
        del dt
        limits = dict(limits or {})
        config = dict(config or {})
        feed_limit = abs(
            float(
                limits.get(
                    "feed_mm_s",
                    limits.get("crawl_mm_s", limits.get("growth_mm_s", limits.get("u_crawl_max", 3.0))),
                )
            )
        )
        rotate_limit = abs(float(limits.get("rotate_deg_s", limits.get("u_rotate_max", 5.0))))
        distance_gain = float(config.get("distance_gain", config.get("gain", 0.15)))
        orientation_gain = float(config.get("orientation_gain", 0.25))
        distance_deadband_mm = abs(float(config.get("distance_deadband_mm", config.get("deadband_mm", 0.5))))
        orientation_deadband_deg = abs(float(config.get("orientation_deadband_deg", 0.5)))
        rotate_control_sign = float(config.get("rotate_control_sign", 1.0))

        distance_error_mm = float(current_distance_mm) - float(distance_ref_mm)
        orientation_tracking_error_deg = float(orientation_ref_deg) - float(current_orientation_error_deg)
        notes: list[str] = []

        if distance_error_mm <= 0.0:
            feed_command = 0.0
            notes.append("distance already satisfied")
        elif distance_error_mm <= distance_deadband_mm:
            feed_command = 0.0
            notes.append("distance within deadband")
        else:
            feed_command = min(feed_limit, max(0.0, distance_gain * distance_error_mm))
            if feed_command >= feed_limit and feed_limit > 0.0:
                notes.append("feed saturated")

        if not enable_orientation:
            rotate_command = 0.0
            notes.append("orientation control disabled")
        elif abs(orientation_tracking_error_deg) <= orientation_deadband_deg:
            rotate_command = 0.0
            notes.append("orientation within deadband")
        else:
            raw_rotate = rotate_control_sign * orientation_gain * orientation_tracking_error_deg
            rotate_command = max(-rotate_limit, min(rotate_limit, raw_rotate))
            if abs(rotate_command) >= rotate_limit and rotate_limit > 0.0:
                notes.append("rotate saturated")

        return PairDockMPCOutput(
            active=True,
            feed_velocity_mm_s=feed_command,
            rotate_velocity_deg_s=rotate_command,
            distance_error_mm=distance_error_mm,
            orientation_error_deg=float(current_orientation_error_deg) - float(orientation_ref_deg),
            feed_active=bool(feed_command != 0.0),
            rotate_active=bool(rotate_command != 0.0),
            notes=notes,
        )

"""Deterministic 2-channel docking solver used as a Phase-1 MPC placeholder."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping


@dataclass(slots=True)
class PairDockMPCOutput:
    """Output of the placeholder distance/orientation docking solver."""

    active: bool
    crawl_velocity_mm_s: float
    rotate_velocity_deg_s: float
    distance_error_mm: float
    orientation_error_deg: float
    crawl_active: bool
    rotate_active: bool
    notes: list[str] = field(default_factory=list)


class PairDockMPC:
    """Minimal deterministic wrapper for pair docking control."""

    def compute(
        self,
        current_distance_mm: float,
        current_orientation_error_deg: float,
        distance_ref_mm: float,
        orientation_ref_deg: float,
        dt: float,
        limits: Mapping[str, Any] | None = None,
        config: Mapping[str, Any] | None = None,
    ) -> PairDockMPCOutput:
        """Compute crawl and rotate corrections toward the docking target."""
        del dt
        limits = dict(limits or {})
        config = dict(config or {})
        crawl_limit = abs(float(limits.get("crawl_mm_s", limits.get("u_crawl_max", 3.0))))
        rotate_limit = abs(float(limits.get("rotate_deg_s", limits.get("u_rotate_max", 5.0))))
        distance_gain = float(config.get("distance_gain", 0.15))
        orientation_gain = float(config.get("orientation_gain", 0.25))
        distance_deadband_mm = abs(float(config.get("distance_deadband_mm", 0.5)))
        orientation_deadband_deg = abs(float(config.get("orientation_deadband_deg", 0.5)))
        rotate_control_sign = float(config.get("rotate_control_sign", 1.0))

        distance_error_mm = float(current_distance_mm) - float(distance_ref_mm)
        orientation_tracking_error_deg = float(orientation_ref_deg) - float(current_orientation_error_deg)
        notes: list[str] = []

        if distance_error_mm <= 0.0:
            crawl_command = 0.0
            notes.append("distance already satisfied")
        elif distance_error_mm <= distance_deadband_mm:
            crawl_command = 0.0
            notes.append("distance within deadband")
        else:
            crawl_command = min(crawl_limit, max(0.0, distance_gain * distance_error_mm))
            if crawl_command >= crawl_limit and crawl_limit > 0.0:
                notes.append("crawl saturated")

        if abs(orientation_tracking_error_deg) <= orientation_deadband_deg:
            rotate_command = 0.0
            notes.append("orientation within deadband")
        else:
            raw_rotate = rotate_control_sign * orientation_gain * orientation_tracking_error_deg
            rotate_command = max(-rotate_limit, min(rotate_limit, raw_rotate))
            if abs(rotate_command) >= rotate_limit and rotate_limit > 0.0:
                notes.append("rotate saturated")

        return PairDockMPCOutput(
            active=True,
            crawl_velocity_mm_s=crawl_command,
            rotate_velocity_deg_s=rotate_command,
            distance_error_mm=distance_error_mm,
            orientation_error_deg=float(current_orientation_error_deg) - float(orientation_ref_deg),
            crawl_active=bool(crawl_command != 0.0),
            rotate_active=bool(rotate_command != 0.0),
            notes=notes,
        )

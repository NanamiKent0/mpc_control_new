"""Generic pair-cooperation MPC for coupled feed and posture stabilization."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping


@dataclass(slots=True)
class CooperateMPCOutput:
    """Output of the generic pair cooperation controller."""

    active: bool
    feed_velocity_mm_s: float
    rotate_velocity_deg_s: float
    bend_velocity_deg_s: float
    distance_error_mm: float
    orientation_error_deg: float
    bend_error_deg: float
    feed_active: bool
    rotate_active: bool
    bend_active: bool
    notes: list[str] = field(default_factory=list)

    @property
    def crawl_velocity_mm_s(self) -> float:
        """Compatibility alias for legacy crawl-oriented callers."""
        return self.feed_velocity_mm_s


class CooperateMPC:
    """Minimal deterministic MPC placeholder for generic pair cooperation."""

    def compute(
        self,
        *,
        feed_forward_mm_s: float,
        current_distance_mm: float,
        distance_ref_mm: float,
        current_orientation_error_deg: float,
        orientation_ref_deg: float,
        current_bend_error_deg: float,
        bend_ref_deg: float,
        dt: float,
        limits: Mapping[str, Any] | None = None,
        config: Mapping[str, Any] | None = None,
    ) -> CooperateMPCOutput:
        """Compute one feed-forward coupled motion with posture stabilization."""
        del dt
        limits = dict(limits or {})
        config = dict(config or {})
        feed_limit = abs(
            float(
                limits.get(
                    "feed_mm_s",
                    limits.get("crawl_mm_s", limits.get("growth_mm_s", 4.0)),
                )
            )
        )
        rotate_limit = abs(float(limits.get("rotate_deg_s", 5.0)))
        bend_limit = abs(float(limits.get("bend_deg_s", 5.0)))
        distance_gain = float(config.get("distance_gain", 0.05))
        orientation_gain = float(config.get("orientation_gain", 0.2))
        bend_gain = float(config.get("bend_gain", 0.2))
        distance_deadband_mm = abs(float(config.get("distance_deadband_mm", 0.5)))
        orientation_deadband_deg = abs(float(config.get("orientation_deadband_deg", 0.5)))
        bend_deadband_deg = abs(float(config.get("bend_deadband_deg", 0.5)))
        allow_reverse_feed = bool(config.get("allow_reverse_feed", False))

        distance_error_mm = float(current_distance_mm) - float(distance_ref_mm)
        orientation_error_deg = float(current_orientation_error_deg) - float(orientation_ref_deg)
        bend_error_deg = float(current_bend_error_deg) - float(bend_ref_deg)
        notes: list[str] = []

        feed_command = float(feed_forward_mm_s)
        if abs(distance_error_mm) > distance_deadband_mm:
            feed_command += distance_gain * distance_error_mm
        else:
            notes.append("distance stabilized")
        if allow_reverse_feed:
            feed_command = max(-feed_limit, min(feed_limit, feed_command))
        else:
            feed_command = max(0.0, min(feed_limit, feed_command))
        if abs(feed_command) >= feed_limit and feed_limit > 0.0:
            notes.append("feed saturated")

        if abs(orientation_error_deg) <= orientation_deadband_deg:
            rotate_command = 0.0
            notes.append("orientation stabilized")
        else:
            rotate_command = max(-rotate_limit, min(rotate_limit, -orientation_gain * orientation_error_deg))
            if abs(rotate_command) >= rotate_limit and rotate_limit > 0.0:
                notes.append("rotate saturated")

        if abs(bend_error_deg) <= bend_deadband_deg:
            bend_command = 0.0
            notes.append("bend stabilized")
        else:
            bend_command = max(-bend_limit, min(bend_limit, -bend_gain * bend_error_deg))
            if abs(bend_command) >= bend_limit and bend_limit > 0.0:
                notes.append("bend saturated")

        return CooperateMPCOutput(
            active=True,
            feed_velocity_mm_s=feed_command,
            rotate_velocity_deg_s=rotate_command,
            bend_velocity_deg_s=bend_command,
            distance_error_mm=distance_error_mm,
            orientation_error_deg=orientation_error_deg,
            bend_error_deg=bend_error_deg,
            feed_active=bool(feed_command != 0.0),
            rotate_active=bool(rotate_command != 0.0),
            bend_active=bool(bend_command != 0.0),
            notes=notes,
        )

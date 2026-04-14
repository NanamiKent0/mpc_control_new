"""Generic scalar feed MPC used for pair approach or coupled feed motion."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping


@dataclass(slots=True)
class ScalarFeedMPCOutput:
    """Output of the generic scalar feed controller."""

    active: bool
    feed_velocity_mm_s: float
    distance_error_mm: float
    target_distance_mm: float
    predicted_terminal_distance_mm: float
    saturated: bool
    notes: list[str] = field(default_factory=list)

    @property
    def crawl_velocity_mm_s(self) -> float:
        """Compatibility alias for legacy crawl-oriented callers."""
        return self.feed_velocity_mm_s


class ScalarFeedMPC:
    """Minimal deterministic MPC placeholder for one scalar feed axis."""

    def compute(
        self,
        current_distance_mm: float,
        distance_ref_mm: float,
        dt: float,
        limits: Mapping[str, Any] | None = None,
        config: Mapping[str, Any] | None = None,
    ) -> ScalarFeedMPCOutput:
        """Compute a feed command that only closes positive distance error."""
        limits = dict(limits or {})
        config = dict(config or {})
        max_feed = abs(
            float(
                limits.get(
                    "feed_mm_s",
                    limits.get("crawl_mm_s", limits.get("growth_mm_s", limits.get("u_max", 5.0))),
                )
            )
        )
        gain = float(config.get("gain", config.get("distance_gain", 0.1)))
        deadband_mm = abs(float(config.get("deadband_mm", config.get("distance_deadband_mm", 0.5))))
        current = float(current_distance_mm)
        target = float(distance_ref_mm)
        error_mm = current - target
        notes: list[str] = []

        if error_mm <= 0.0:
            command = 0.0
            notes.append("distance already satisfied")
        elif error_mm <= deadband_mm:
            command = 0.0
            notes.append("distance within deadband")
        else:
            command = min(max_feed, max(0.0, gain * error_mm))
            if command >= max_feed and max_feed > 0.0:
                notes.append("feed saturated")

        predicted_terminal_distance_mm = max(target, current - max(float(dt), 0.0) * command)
        return ScalarFeedMPCOutput(
            active=True,
            feed_velocity_mm_s=command,
            distance_error_mm=error_mm,
            target_distance_mm=target,
            predicted_terminal_distance_mm=predicted_terminal_distance_mm,
            saturated=bool(command >= max_feed and max_feed > 0.0),
            notes=notes,
        )

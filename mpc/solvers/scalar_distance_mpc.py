"""Deterministic scalar distance controller used as a Phase-1 MPC placeholder."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping


@dataclass(slots=True)
class ScalarDistanceMPCOutput:
    """Output of the placeholder scalar distance solver."""

    active: bool
    crawl_velocity_mm_s: float
    distance_error_mm: float
    target_distance_mm: float
    predicted_terminal_distance_mm: float
    saturated: bool
    notes: list[str] = field(default_factory=list)


class ScalarDistanceMPC:
    """Minimal deterministic wrapper for coarse distance reduction."""

    def compute(
        self,
        current_distance_mm: float,
        distance_ref_mm: float,
        dt: float,
        limits: Mapping[str, Any] | None = None,
        config: Mapping[str, Any] | None = None,
    ) -> ScalarDistanceMPCOutput:
        """Compute a positive crawl command that only closes distance."""
        limits = dict(limits or {})
        config = dict(config or {})
        max_crawl = abs(float(limits.get("crawl_mm_s", limits.get("u_max", 5.0))))
        gain = float(config.get("gain", 0.1))
        deadband_mm = abs(float(config.get("deadband_mm", 0.5)))
        current = float(current_distance_mm)
        target = float(distance_ref_mm)
        error_mm = current - target
        notes: list[str] = []

        if error_mm <= 0.0:
            notes.append("already near; does not back away")
            command = 0.0
        elif error_mm <= deadband_mm:
            notes.append("within deadband")
            command = 0.0
        else:
            command = min(max_crawl, max(0.0, gain * error_mm))
            if command >= max_crawl and max_crawl > 0.0:
                notes.append("saturated at crawl limit")

        predicted_terminal_distance_mm = max(target, current - max(float(dt), 0.0) * command)
        return ScalarDistanceMPCOutput(
            active=True,
            crawl_velocity_mm_s=command,
            distance_error_mm=error_mm,
            target_distance_mm=target,
            predicted_terminal_distance_mm=predicted_terminal_distance_mm,
            saturated=bool(command >= max_crawl and max_crawl > 0.0),
            notes=notes,
        )

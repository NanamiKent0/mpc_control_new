"""Generic posture-adjust MPC for release and local alignment stages."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping


@dataclass(slots=True)
class PostureAdjustMPCOutput:
    """Output of the generic posture adjustment controller."""

    active: bool
    rotate_velocity_deg_s: float
    bend_velocity_deg_s: float
    orientation_error_deg: float
    bend_error_deg: float
    rotate_active: bool
    bend_active: bool
    posture_aligned: bool
    notes: list[str] = field(default_factory=list)


class PostureAdjustMPC:
    """Minimal deterministic MPC placeholder for local posture adjustment."""

    def compute(
        self,
        *,
        current_orientation_error_deg: float,
        orientation_ref_deg: float,
        current_bend_error_deg: float,
        bend_ref_deg: float,
        dt: float,
        limits: Mapping[str, Any] | None = None,
        config: Mapping[str, Any] | None = None,
    ) -> PostureAdjustMPCOutput:
        """Compute rotate and bend corrections toward the posture target."""
        del dt
        limits = dict(limits or {})
        config = dict(config or {})
        rotate_limit = abs(float(limits.get("rotate_deg_s", 5.0)))
        bend_limit = abs(float(limits.get("bend_deg_s", 5.0)))
        orientation_gain = float(config.get("orientation_gain", 0.2))
        bend_gain = float(config.get("bend_gain", 0.2))
        orientation_deadband_deg = abs(float(config.get("orientation_deadband_deg", 1.0)))
        bend_deadband_deg = abs(float(config.get("bend_deadband_deg", 1.0)))

        orientation_error_deg = float(current_orientation_error_deg) - float(orientation_ref_deg)
        bend_error_deg = float(current_bend_error_deg) - float(bend_ref_deg)
        notes: list[str] = []

        if abs(orientation_error_deg) <= orientation_deadband_deg:
            rotate_command = 0.0
            notes.append("orientation aligned")
        else:
            rotate_command = max(-rotate_limit, min(rotate_limit, -orientation_gain * orientation_error_deg))
            if abs(rotate_command) >= rotate_limit and rotate_limit > 0.0:
                notes.append("rotate saturated")

        if abs(bend_error_deg) <= bend_deadband_deg:
            bend_command = 0.0
            notes.append("bend aligned")
        else:
            bend_command = max(-bend_limit, min(bend_limit, -bend_gain * bend_error_deg))
            if abs(bend_command) >= bend_limit and bend_limit > 0.0:
                notes.append("bend saturated")

        posture_aligned = bool(rotate_command == 0.0 and bend_command == 0.0)
        return PostureAdjustMPCOutput(
            active=True,
            rotate_velocity_deg_s=rotate_command,
            bend_velocity_deg_s=bend_command,
            orientation_error_deg=orientation_error_deg,
            bend_error_deg=bend_error_deg,
            rotate_active=bool(rotate_command != 0.0),
            bend_active=bool(bend_command != 0.0),
            posture_aligned=posture_aligned,
            notes=notes,
        )

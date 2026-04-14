"""Pair-wise geometry helpers built on top of module poses."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .frame_conventions import pair_key, signed_angle_about_axis_deg
from .module_kinematics import ModulePose


@dataclass(slots=True)
class PairGeometry:
    """Geometry relation snapshot for one active/passive module pair."""

    active_module: str
    passive_module: str
    distance_mm: float
    orientation_error_deg: float
    active_point: np.ndarray
    passive_point: np.ndarray
    active_frame: dict[str, np.ndarray]
    passive_frame: dict[str, np.ndarray]
    diagnostics: dict[str, object] = field(default_factory=dict)

    def key(self) -> str:
        """Return the stable pair key."""
        return pair_key(self.active_module, self.passive_module)


def compute_pair_distance(
    active_pose: ModulePose,
    passive_pose: ModulePose,
) -> float:
    """Return the interface-point distance for one active/passive pair."""
    delta = np.asarray(passive_pose.start_point, dtype=float) - np.asarray(active_pose.end_point, dtype=float)
    return float(np.linalg.norm(delta))


def compute_pair_orientation_error(
    active_pose: ModulePose,
    passive_pose: ModulePose,
) -> float:
    """Return the signed twist mismatch between two module interface frames."""
    passive_frame = passive_pose.start_frame
    active_frame = active_pose.end_frame
    return float(
        signed_angle_about_axis_deg(
            passive_frame["y_axis"],
            active_frame["y_axis"],
            passive_frame["x_axis"],
        )
    )


def compute_pair_geometry(
    active_pose: ModulePose,
    passive_pose: ModulePose,
) -> PairGeometry:
    """Build the full pair geometry snapshot for one active/passive interface."""
    active_point = np.asarray(active_pose.end_point, dtype=float).copy()
    passive_point = np.asarray(passive_pose.start_point, dtype=float).copy()
    distance_mm = compute_pair_distance(active_pose, passive_pose)
    orientation_error_deg = compute_pair_orientation_error(active_pose, passive_pose)
    interface_delta = passive_point - active_point
    return PairGeometry(
        active_module=active_pose.module_id,
        passive_module=passive_pose.module_id,
        distance_mm=distance_mm,
        orientation_error_deg=orientation_error_deg,
        active_point=active_point,
        passive_point=passive_point,
        active_frame=active_pose.end_frame,
        passive_frame=passive_pose.start_frame,
        diagnostics={
            "pair_key": pair_key(active_pose.module_id, passive_pose.module_id),
            "interface_delta_mm": interface_delta.astype(float, copy=True),
            "distance_rule": "active.end to passive.start",
            "orientation_rule": "signed twist passive.start.y -> active.end.y about passive.start.x",
        },
    )


__all__ = [
    "PairGeometry",
    "compute_pair_distance",
    "compute_pair_geometry",
    "compute_pair_orientation_error",
]

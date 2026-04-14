"""Module-level forward geometry primitives for the unified kinematics layer."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from ..models.module_state import ModuleState
from .frame_conventions import (
    copy_frame,
    frame_dict_from_rotation,
    frame_dict_from_transform,
    normalize_vector,
)


@dataclass(frozen=True, slots=True)
class ModuleGeometryConfig:
    """Shared geometric lengths for the chain model."""

    tip_length_mm: float = 150.0
    joint_segment_1_mm: float = 80.0
    joint_segment_2_mm: float = 75.0
    joint_segment_3_mm: float = 55.0

    @property
    def joint_total_length_mm(self) -> float:
        """Return the total joint length used by the current geometry model."""
        return (
            float(self.joint_segment_1_mm)
            + float(self.joint_segment_2_mm)
            + float(self.joint_segment_3_mm)
        )

    def segment_lengths(self) -> dict[str, float]:
        """Return the lengths as a JSON-friendly mapping."""
        return {
            "tip_length_mm": float(self.tip_length_mm),
            "joint_segment_1_mm": float(self.joint_segment_1_mm),
            "joint_segment_2_mm": float(self.joint_segment_2_mm),
            "joint_segment_3_mm": float(self.joint_segment_3_mm),
            "joint_total_length_mm": float(self.joint_total_length_mm),
        }


@dataclass(slots=True)
class ModulePose:
    """Geometry snapshot for one module anchored in the shared base frame."""

    module_id: str
    module_type: str
    path_labels: tuple[str, ...]
    path_points: tuple[np.ndarray, ...]
    named_points: dict[str, np.ndarray]
    named_frames: dict[str, dict[str, np.ndarray]]
    start_transform: np.ndarray
    end_transform: np.ndarray
    transforms: tuple[np.ndarray, ...]
    metadata: dict[str, object] = field(default_factory=dict)

    @property
    def start_point(self) -> np.ndarray:
        """Return the module start point in the shared base frame."""
        return self.path_points[0]

    @property
    def end_point(self) -> np.ndarray:
        """Return the module end point in the shared base frame."""
        return self.path_points[-1]

    @property
    def start_frame(self) -> dict[str, np.ndarray]:
        """Return the frame at the module start interface."""
        return self.named_frames[f"{self.module_id}_start_frame"]

    @property
    def end_frame(self) -> dict[str, np.ndarray]:
        """Return the frame at the module end interface."""
        return self.named_frames[f"{self.module_id}_end_frame"]

    def xyz_path(self) -> tuple[list[float], list[float], list[float]]:
        """Return the path points as renderer-friendly xyz coordinate lists."""
        xs = [float(point[0]) for point in self.path_points]
        ys = [float(point[1]) for point in self.path_points]
        zs = [float(point[2]) for point in self.path_points]
        return xs, ys, zs

    def copied_named_points(self) -> dict[str, np.ndarray]:
        """Return a defensive copy of the point map."""
        return {
            name: np.asarray(point, dtype=float).copy()
            for name, point in self.named_points.items()
        }

    def copied_named_frames(self) -> dict[str, dict[str, np.ndarray]]:
        """Return a defensive copy of the frame map."""
        return {
            name: copy_frame(frame)
            for name, frame in self.named_frames.items()
        }


def compute_module_pose(
    module_state: ModuleState,
    *,
    geometry_config: ModuleGeometryConfig | None = None,
    anchor_transform: np.ndarray | None = None,
    entry_offset_mm: float | None = None,
) -> ModulePose:
    """Compute the pose and geometry path for one module."""
    config = geometry_config or ModuleGeometryConfig()
    resolved_anchor = np.eye(4, dtype=float) if anchor_transform is None else np.asarray(anchor_transform, dtype=float).reshape(4, 4)
    if module_state.module_type == "tip":
        return _compute_tip_pose(
            module_state,
            geometry_config=config,
            anchor_transform=resolved_anchor,
            entry_offset_mm=entry_offset_mm,
        )
    if module_state.module_type == "joint":
        return _compute_joint_pose(
            module_state,
            geometry_config=config,
            anchor_transform=resolved_anchor,
            entry_offset_mm=entry_offset_mm,
        )
    raise ValueError(f"unsupported_module_type:{module_state.module_type}")


def _compute_tip_pose(
    module_state: ModuleState,
    *,
    geometry_config: ModuleGeometryConfig,
    anchor_transform: np.ndarray,
    entry_offset_mm: float | None,
) -> ModulePose:
    growth_mm = float(module_state.dofs.get("growth_mm", geometry_config.tip_length_mm))
    resolved_entry_offset_mm = (
        float(entry_offset_mm)
        if entry_offset_mm is not None
        else growth_mm - float(geometry_config.tip_length_mm)
    )
    start_transform = anchor_transform @ _tip_transform(resolved_entry_offset_mm)
    end_transform = start_transform @ _translation_x(geometry_config.tip_length_mm)
    start_point = _point_from_transform(start_transform)
    end_point = _point_from_transform(end_transform)
    start_frame = frame_dict_from_transform(start_transform, origin=start_point)
    end_rotation = _module_frame_rotation(start_point, end_point, roll_rad=0.0)
    end_frame = frame_dict_from_rotation(end_rotation, origin=end_point)
    named_points = {
        f"{module_state.module_id}_start": start_point.copy(),
        f"{module_state.module_id}_end": end_point.copy(),
    }
    named_frames = {
        f"{module_state.module_id}_start_frame": start_frame,
        f"{module_state.module_id}_end_frame": end_frame,
    }
    return ModulePose(
        module_id=module_state.module_id,
        module_type=module_state.module_type,
        path_labels=(f"{module_state.module_id}_start", f"{module_state.module_id}_end"),
        path_points=(start_point, end_point),
        named_points=named_points,
        named_frames=named_frames,
        start_transform=start_transform.copy(),
        end_transform=end_transform.copy(),
        transforms=(start_transform.copy(), end_transform.copy()),
        metadata={
            "entry_offset_mm": resolved_entry_offset_mm,
            "growth_mm": growth_mm,
            "segment_length_mm": float(geometry_config.tip_length_mm),
        },
    )


def _compute_joint_pose(
    module_state: ModuleState,
    *,
    geometry_config: ModuleGeometryConfig,
    anchor_transform: np.ndarray,
    entry_offset_mm: float | None,
) -> ModulePose:
    crawl_mm = float(module_state.dofs.get("crawl_mm", 0.0))
    bend_deg = float(module_state.dofs.get("bend_deg", 0.0))
    rotate_deg = float(module_state.dofs.get("rotate_deg", 0.0))
    resolved_entry_offset_mm = 0.0 if entry_offset_mm is None else float(entry_offset_mm)
    t01, t12, t23, t34, t04 = _joint_transforms(
        resolved_entry_offset_mm,
        bend_rad=float(np.deg2rad(bend_deg)),
        rotate_rad=float(np.deg2rad(rotate_deg)),
        geometry_config=geometry_config,
    )
    start_transform = anchor_transform @ t01
    mid1_transform = anchor_transform @ t01 @ t12
    mid2_transform = anchor_transform @ t01 @ t12 @ t23
    end_transform = anchor_transform @ t04
    start_point = _point_from_transform(start_transform)
    mid1_point = _point_from_transform(mid1_transform)
    mid2_point = _point_from_transform(mid2_transform)
    end_point = _point_from_transform(end_transform)
    start_frame = frame_dict_from_transform(start_transform, origin=start_point)
    end_rotation = _joint_frame_rotation(
        mid2_point,
        end_point,
        bend_axis=end_transform[:3, 2],
    )
    end_frame = frame_dict_from_rotation(end_rotation, origin=end_point)
    bend_vectors = _joint_bend_diagnostics(
        start_point=start_point,
        mid1_point=mid1_point,
        mid2_point=mid2_point,
        end_point=end_point,
        bend_axis=end_rotation[:, 2],
    )
    named_points = {
        f"{module_state.module_id}_start": start_point.copy(),
        f"{module_state.module_id}_mid1": mid1_point.copy(),
        f"{module_state.module_id}_mid2": mid2_point.copy(),
        f"{module_state.module_id}_end": end_point.copy(),
    }
    named_frames = {
        f"{module_state.module_id}_start_frame": start_frame,
        f"{module_state.module_id}_end_frame": end_frame,
    }
    return ModulePose(
        module_id=module_state.module_id,
        module_type=module_state.module_type,
        path_labels=(
            f"{module_state.module_id}_start",
            f"{module_state.module_id}_mid1",
            f"{module_state.module_id}_mid2",
            f"{module_state.module_id}_end",
        ),
        path_points=(start_point, mid1_point, mid2_point, end_point),
        named_points=named_points,
        named_frames=named_frames,
        start_transform=start_transform.copy(),
        end_transform=end_transform.copy(),
        transforms=(
            start_transform.copy(),
            mid1_transform.copy(),
            mid2_transform.copy(),
            end_transform.copy(),
        ),
        metadata={
            "entry_offset_mm": resolved_entry_offset_mm,
            "crawl_mm": crawl_mm,
            "bend_deg": bend_deg,
            "rotate_deg": rotate_deg,
            "bend_angle_deg": float(bend_vectors["bend_angle_deg"]),
            "bend_angle_rad": float(bend_vectors["bend_angle_rad"]),
            "bend_axis": np.asarray(bend_vectors["bend_axis"], dtype=float).copy(),
            "bend_vectors": {
                "v_in": np.asarray(bend_vectors["v_in"], dtype=float).copy(),
                "v_out": np.asarray(bend_vectors["v_out"], dtype=float).copy(),
                "bend_axis": np.asarray(bend_vectors["bend_axis"], dtype=float).copy(),
                "bend_angle_deg": float(bend_vectors["bend_angle_deg"]),
                "bend_angle_rad": float(bend_vectors["bend_angle_rad"]),
            },
        },
    )


def _joint_transforms(
    x_mm: float,
    *,
    bend_rad: float,
    rotate_rad: float,
    geometry_config: ModuleGeometryConfig,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    cos_rotate = float(np.cos(rotate_rad))
    sin_rotate = float(np.sin(rotate_rad))
    half_bend = bend_rad / 2.0
    cos_half_bend = float(np.cos(half_bend))
    sin_half_bend = float(np.sin(half_bend))
    t01 = np.array(
        [
            [1.0, 0.0, 0.0, float(x_mm)],
            [0.0, cos_rotate, -sin_rotate, 0.0],
            [0.0, sin_rotate, cos_rotate, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )
    t12 = np.array(
        [
            [cos_half_bend, -sin_half_bend, 0.0, float(geometry_config.joint_segment_1_mm)],
            [sin_half_bend, cos_half_bend, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )
    t23 = np.array(
        [
            [cos_half_bend, -sin_half_bend, 0.0, float(geometry_config.joint_segment_2_mm)],
            [sin_half_bend, cos_half_bend, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )
    t34 = _translation_x(geometry_config.joint_segment_3_mm)
    t04 = t01 @ t12 @ t23 @ t34
    return t01, t12, t23, t34, t04


def _tip_transform(x_mm: float) -> np.ndarray:
    return _translation_x(x_mm)


def _translation_x(distance_mm: float) -> np.ndarray:
    return np.array(
        [
            [1.0, 0.0, 0.0, float(distance_mm)],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def _point_from_transform(transform: np.ndarray) -> np.ndarray:
    return np.asarray(transform[:3, 3], dtype=float).copy()


def _module_frame_rotation(
    start_point: np.ndarray,
    end_point: np.ndarray,
    *,
    roll_rad: float,
) -> np.ndarray:
    x_axis = normalize_vector(np.asarray(end_point, dtype=float) - np.asarray(start_point, dtype=float))
    z_ref = np.array([0.0, 0.0, 1.0], dtype=float)
    if abs(float(np.dot(x_axis, z_ref))) > 0.95:
        z_ref = np.array([0.0, 1.0, 0.0], dtype=float)
    y_base = normalize_vector(np.cross(z_ref, x_axis))
    z_base = normalize_vector(np.cross(x_axis, y_base))
    cos_roll = float(np.cos(roll_rad))
    sin_roll = float(np.sin(roll_rad))
    y_axis = normalize_vector(cos_roll * y_base + sin_roll * z_base)
    z_axis = normalize_vector(np.cross(x_axis, y_axis))
    return np.column_stack((x_axis, y_axis, z_axis)).astype(float, copy=False)


def _joint_frame_rotation(
    start_point: np.ndarray,
    end_point: np.ndarray,
    *,
    bend_axis: np.ndarray,
) -> np.ndarray:
    x_axis = normalize_vector(np.asarray(end_point, dtype=float) - np.asarray(start_point, dtype=float))
    z_hint = normalize_vector(bend_axis)
    z_projected = z_hint - float(np.dot(z_hint, x_axis)) * x_axis
    z_axis = normalize_vector(z_projected)
    y_axis = normalize_vector(np.cross(z_axis, x_axis))
    z_axis = normalize_vector(np.cross(x_axis, y_axis))
    return np.column_stack((x_axis, y_axis, z_axis)).astype(float, copy=False)


def _joint_bend_diagnostics(
    *,
    start_point: np.ndarray,
    mid1_point: np.ndarray,
    mid2_point: np.ndarray,
    end_point: np.ndarray,
    bend_axis: np.ndarray,
) -> dict[str, np.ndarray | float]:
    v_in = np.asarray(mid1_point, dtype=float).reshape(3) - np.asarray(start_point, dtype=float).reshape(3)
    v_out = np.asarray(end_point, dtype=float).reshape(3) - np.asarray(mid2_point, dtype=float).reshape(3)
    bend_angle_deg = _angle_between_vectors_deg(v_in, v_out)
    return {
        "v_in": v_in.astype(float, copy=True),
        "v_out": v_out.astype(float, copy=True),
        "bend_axis": normalize_vector(bend_axis).astype(float, copy=True),
        "bend_angle_deg": float(bend_angle_deg),
        "bend_angle_rad": float(np.deg2rad(bend_angle_deg)),
    }


def _angle_between_vectors_deg(v1: np.ndarray, v2: np.ndarray) -> float:
    u1 = normalize_vector(v1)
    u2 = normalize_vector(v2)
    cosine = float(np.clip(np.dot(u1, u2), -1.0, 1.0))
    return float(np.rad2deg(np.arccos(cosine)))


__all__ = [
    "ModuleGeometryConfig",
    "ModulePose",
    "compute_module_pose",
]

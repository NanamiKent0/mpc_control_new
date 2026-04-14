"""Shared frame and ordering conventions for the geometry layer."""

from __future__ import annotations

from dataclasses import dataclass
import re
from typing import Iterable, Sequence

import numpy as np


_JOINT_MODULE_PATTERN = re.compile(r"^joint(\d+)$")


@dataclass(frozen=True, slots=True)
class FrameConvention:
    """Serializable description of the shared geometry conventions."""

    world_frame: str = "base_frame"
    linear_units: str = "mm"
    angular_units: str = "deg"
    forward_axis: str = "+x"
    lateral_axis: str = "+y"
    normal_axis: str = "+z"
    logical_module_order: str = "distal_to_proximal"
    geometry_build_order: str = "proximal_to_distal"
    pair_distance_rule: str = "active.end to passive.start"
    pair_orientation_rule: str = "signed twist passive.start.y -> active.end.y about passive.start.x"

    def snapshot(self) -> dict[str, str]:
        """Return a JSON-friendly description of the conventions."""
        return {
            "world_frame": self.world_frame,
            "linear_units": self.linear_units,
            "angular_units": self.angular_units,
            "forward_axis": self.forward_axis,
            "lateral_axis": self.lateral_axis,
            "normal_axis": self.normal_axis,
            "logical_module_order": self.logical_module_order,
            "geometry_build_order": self.geometry_build_order,
            "pair_distance_rule": self.pair_distance_rule,
            "pair_orientation_rule": self.pair_orientation_rule,
        }


DEFAULT_FRAME_CONVENTION = FrameConvention()


def pair_key(active_module: str, passive_module: str) -> str:
    """Return the stable directed key for one pair geometry result."""
    return f"{active_module}->{passive_module}"


def joint_index(module_id: str) -> int | None:
    """Return the numeric suffix for `jointN` identifiers."""
    match = _JOINT_MODULE_PATTERN.fullmatch(str(module_id))
    if match is None:
        return None
    return int(match.group(1))


def is_joint_module(module_id: str) -> bool:
    """Return whether one identifier follows the shared `jointN` convention."""
    return joint_index(module_id) is not None


def canonical_module_order(module_ids: Iterable[str]) -> list[str]:
    """Return the shared logical chain order: `tip`, then `joint1`, `joint2`, ..."""
    resolved = [str(module_id) for module_id in module_ids]
    joints = sorted(
        {module_id for module_id in resolved if is_joint_module(module_id)},
        key=lambda module_id: int(joint_index(module_id) or 0),
    )
    others = sorted(
        {
            module_id
            for module_id in resolved
            if module_id != "tip" and not is_joint_module(module_id)
        }
    )
    ordered: list[str] = []
    if "tip" in resolved:
        ordered.append("tip")
    ordered.extend(joints)
    ordered.extend(others)
    return ordered


def proximal_to_distal_order(ordered_modules: Sequence[str]) -> list[str]:
    """Convert logical `tip -> joint1 -> joint2` order into geometry build order."""
    resolved = [str(module_id) for module_id in ordered_modules]
    if not resolved:
        return []
    tip_modules = [module_id for module_id in resolved if module_id == "tip"]
    non_tip_modules = [module_id for module_id in resolved if module_id != "tip"]
    non_tip_modules.reverse()
    return [*non_tip_modules, *tip_modules]


def adjacent_active_passive_pairs(ordered_modules: Sequence[str]) -> list[tuple[str, str]]:
    """Return adjacent directed pairs using the shared logical order."""
    resolved = [str(module_id) for module_id in ordered_modules]
    return [
        (active_module, passive_module)
        for passive_module, active_module in zip(resolved, resolved[1:], strict=False)
    ]


def normalize_vector(vector: np.ndarray) -> np.ndarray:
    """Return a unit-length vector and reject degenerate inputs."""
    resolved = np.asarray(vector, dtype=float).reshape(3)
    norm = float(np.linalg.norm(resolved))
    if norm <= 1e-12:
        raise ValueError("cannot normalize a near-zero vector")
    return resolved / norm


def frame_dict_from_rotation(
    rotation: np.ndarray,
    *,
    origin: np.ndarray,
) -> dict[str, np.ndarray]:
    """Return the shared frame dictionary shape used across the geometry layer."""
    resolved_rotation = np.asarray(rotation, dtype=float).reshape(3, 3)
    resolved_origin = np.asarray(origin, dtype=float).reshape(3)
    return {
        "origin": resolved_origin.astype(float, copy=True),
        "R": resolved_rotation.astype(float, copy=True),
        "x_axis": resolved_rotation[:, 0].astype(float, copy=True),
        "y_axis": resolved_rotation[:, 1].astype(float, copy=True),
        "z_axis": resolved_rotation[:, 2].astype(float, copy=True),
    }


def frame_dict_from_transform(
    transform: np.ndarray,
    *,
    origin: np.ndarray | None = None,
) -> dict[str, np.ndarray]:
    """Build one shared frame dictionary from a homogeneous transform."""
    resolved_transform = np.asarray(transform, dtype=float).reshape(4, 4)
    if origin is None:
        resolved_origin = resolved_transform[:3, 3]
    else:
        resolved_origin = np.asarray(origin, dtype=float).reshape(3)
    return frame_dict_from_rotation(
        resolved_transform[:3, :3],
        origin=resolved_origin,
    )


def copy_frame(frame: dict[str, np.ndarray]) -> dict[str, np.ndarray]:
    """Return a deep-enough copy for one shared frame dictionary."""
    return {
        name: np.asarray(value, dtype=float).copy()
        for name, value in dict(frame).items()
    }


def signed_angle_about_axis_deg(
    reference_vector: np.ndarray,
    target_vector: np.ndarray,
    axis_vector: np.ndarray,
) -> float:
    """Return the signed angle from `reference_vector` to `target_vector` about `axis_vector`."""
    axis = normalize_vector(axis_vector)
    reference = np.asarray(reference_vector, dtype=float).reshape(3)
    target = np.asarray(target_vector, dtype=float).reshape(3)
    reference_projected = reference - float(np.dot(reference, axis)) * axis
    target_projected = target - float(np.dot(target, axis)) * axis
    if float(np.linalg.norm(reference_projected)) <= 1e-12 or float(np.linalg.norm(target_projected)) <= 1e-12:
        return 0.0
    ref_unit = normalize_vector(reference_projected)
    target_unit = normalize_vector(target_projected)
    sine = float(np.dot(axis, np.cross(ref_unit, target_unit)))
    cosine = float(np.clip(np.dot(ref_unit, target_unit), -1.0, 1.0))
    return float(np.rad2deg(np.arctan2(sine, cosine)))


__all__ = [
    "DEFAULT_FRAME_CONVENTION",
    "FrameConvention",
    "adjacent_active_passive_pairs",
    "canonical_module_order",
    "copy_frame",
    "frame_dict_from_rotation",
    "frame_dict_from_transform",
    "is_joint_module",
    "joint_index",
    "normalize_vector",
    "pair_key",
    "proximal_to_distal_order",
    "signed_angle_about_axis_deg",
]

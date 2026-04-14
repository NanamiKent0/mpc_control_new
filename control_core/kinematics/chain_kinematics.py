"""Chain-level geometry composition for the unified kinematics layer."""

from __future__ import annotations

from collections.abc import Iterable, Mapping
from dataclasses import dataclass, field

import numpy as np

from ..models.module_state import ModuleState
from .frame_conventions import (
    DEFAULT_FRAME_CONVENTION,
    adjacent_active_passive_pairs,
    canonical_module_order,
    copy_frame,
    frame_dict_from_rotation,
    pair_key,
    proximal_to_distal_order,
)
from .module_kinematics import ModuleGeometryConfig, ModulePose, compute_module_pose
from .pair_kinematics import PairGeometry, compute_pair_geometry


@dataclass(slots=True)
class ChainSnapshot:
    """Complete geometry snapshot for one chain state."""

    ordered_modules: tuple[str, ...]
    geometry_order: tuple[str, ...]
    module_poses: dict[str, ModulePose]
    pair_geometries: dict[str, PairGeometry]
    named_points: dict[str, np.ndarray]
    named_frames: dict[str, dict[str, np.ndarray]]
    module_paths: dict[str, tuple[list[float], list[float], list[float]]]
    module_path_labels: dict[str, tuple[str, ...]]
    frame_conventions: dict[str, str]
    metadata: dict[str, object] = field(default_factory=dict)

    def get_pair_geometry(self, active_module: str, passive_module: str) -> PairGeometry | None:
        """Return one stored pair geometry by its directed key."""
        return self.pair_geometries.get(pair_key(active_module, passive_module))

    def copied_named_points(self) -> dict[str, np.ndarray]:
        """Return a defensive copy of the named point map."""
        return {
            name: np.asarray(point, dtype=float).copy()
            for name, point in self.named_points.items()
        }

    def copied_named_frames(self) -> dict[str, dict[str, np.ndarray]]:
        """Return a defensive copy of the named frame map."""
        return {
            name: copy_frame(frame)
            for name, frame in self.named_frames.items()
        }


def compute_chain_snapshot(
    module_states: Mapping[str, ModuleState] | Iterable[ModuleState],
    *,
    ordered_modules: Iterable[str] | None = None,
    geometry_config: ModuleGeometryConfig | None = None,
) -> ChainSnapshot:
    """Compute the full chain geometry snapshot from normalized module states."""
    config = geometry_config or ModuleGeometryConfig()
    state_map = _normalize_module_state_map(module_states)
    resolved_order = _resolve_ordered_modules(state_map, ordered_modules=ordered_modules)
    geometry_order = tuple(proximal_to_distal_order(resolved_order))
    named_points: dict[str, np.ndarray] = {"base_origin": np.zeros(3, dtype=float)}
    named_frames: dict[str, dict[str, np.ndarray]] = {
        "base_frame": frame_dict_from_rotation(np.eye(3, dtype=float), origin=np.zeros(3, dtype=float))
    }
    module_poses: dict[str, ModulePose] = {}
    module_paths: dict[str, tuple[list[float], list[float], list[float]]] = {}
    module_path_labels: dict[str, tuple[str, ...]] = {}
    all_point_labels: list[str] = ["base_origin"]
    transform_chain: list[np.ndarray] = [np.eye(4, dtype=float)]
    joint_modules = [
        module_id
        for module_id in geometry_order
        if state_map[module_id].module_type == "joint"
    ]
    current_transform = np.eye(4, dtype=float)
    previous_crawl_mm: float | None = None
    has_tip = any(state.module_type == "tip" for state in state_map.values())

    for module_id in geometry_order:
        module_state = state_map[module_id]
        entry_offset_mm = _module_entry_offset(
            module_state,
            config=config,
            joint_count=len(joint_modules),
            previous_crawl_mm=previous_crawl_mm,
            has_tip=has_tip,
        )
        pose = compute_module_pose(
            module_state,
            geometry_config=config,
            anchor_transform=current_transform,
            entry_offset_mm=entry_offset_mm,
        )
        module_poses[module_id] = pose
        module_paths[module_id] = pose.xyz_path()
        module_path_labels[module_id] = pose.path_labels
        for name, point in pose.named_points.items():
            named_points[name] = np.asarray(point, dtype=float).copy()
        for name, frame in pose.named_frames.items():
            named_frames[name] = copy_frame(frame)
        all_point_labels.extend(pose.path_labels)
        transform_chain.extend(np.asarray(transform, dtype=float).copy() for transform in pose.transforms)
        current_transform = np.asarray(pose.end_transform, dtype=float).copy()
        if module_state.module_type == "joint":
            previous_crawl_mm = float(module_state.dofs.get("crawl_mm", 0.0))

    pair_geometries: dict[str, PairGeometry] = {}
    for active_module, passive_module in adjacent_active_passive_pairs(resolved_order):
        active_pose = module_poses.get(active_module)
        passive_pose = module_poses.get(passive_module)
        if active_pose is None or passive_pose is None:
            continue
        pair_geometry = compute_pair_geometry(active_pose, passive_pose)
        pair_geometries[pair_geometry.key()] = pair_geometry

    metadata = {
        "ordered_modules": list(resolved_order),
        "geometry_order": list(geometry_order),
        "joint_count": len(joint_modules),
        "joint_processing_order": list(joint_modules),
        "segment_lengths": config.segment_lengths(),
        "all_point_labels": list(all_point_labels),
        "transform_chain": [np.asarray(transform, dtype=float).copy() for transform in transform_chain],
        "named_points": {
            name: np.asarray(point, dtype=float).copy()
            for name, point in named_points.items()
        },
        "named_frames": {
            name: copy_frame(frame)
            for name, frame in named_frames.items()
        },
        "module_path_labels": {
            module_id: tuple(labels)
            for module_id, labels in module_path_labels.items()
        },
        "pair_keys": list(pair_geometries),
        "frame_conventions": DEFAULT_FRAME_CONVENTION.snapshot(),
    }
    for module_id, pose in module_poses.items():
        bend_vectors = pose.metadata.get("bend_vectors")
        if bend_vectors is not None:
            metadata[f"{module_id}_bend_vectors"] = bend_vectors
        if pose.module_type == "tip":
            metadata["tip_growth_mm"] = float(pose.metadata.get("growth_mm", config.tip_length_mm))
        if pose.module_type == "joint":
            metadata[f"{module_id}_state"] = {
                "crawl_mm": float(pose.metadata.get("crawl_mm", 0.0)),
                "bend_deg": float(pose.metadata.get("bend_deg", 0.0)),
                "rotate_deg": float(pose.metadata.get("rotate_deg", 0.0)),
            }

    return ChainSnapshot(
        ordered_modules=resolved_order,
        geometry_order=geometry_order,
        module_poses=module_poses,
        pair_geometries=pair_geometries,
        named_points=named_points,
        named_frames=named_frames,
        module_paths=module_paths,
        module_path_labels=module_path_labels,
        frame_conventions=DEFAULT_FRAME_CONVENTION.snapshot(),
        metadata=metadata,
    )


def _normalize_module_state_map(
    module_states: Mapping[str, ModuleState] | Iterable[ModuleState],
) -> dict[str, ModuleState]:
    if isinstance(module_states, Mapping):
        normalized: dict[str, ModuleState] = {}
        for module_id, module_state in module_states.items():
            if not isinstance(module_state, ModuleState):
                raise TypeError(f"unsupported_module_state_value:{type(module_state)!r}")
            normalized[str(module_state.module_id or module_id)] = module_state
        return normalized
    normalized = {}
    for module_state in module_states:
        if not isinstance(module_state, ModuleState):
            raise TypeError(f"unsupported_module_state_value:{type(module_state)!r}")
        normalized[str(module_state.module_id)] = module_state
    return normalized


def _resolve_ordered_modules(
    state_map: Mapping[str, ModuleState],
    *,
    ordered_modules: Iterable[str] | None,
) -> tuple[str, ...]:
    if ordered_modules is None:
        return tuple(canonical_module_order(state_map))
    resolved = tuple(str(module_id) for module_id in ordered_modules)
    missing = [module_id for module_id in resolved if module_id not in state_map]
    if missing:
        raise KeyError(f"missing_module_states:{','.join(missing)}")
    return resolved


def _module_entry_offset(
    module_state: ModuleState,
    *,
    config: ModuleGeometryConfig,
    joint_count: int,
    previous_crawl_mm: float | None,
    has_tip: bool,
) -> float:
    if module_state.module_type == "tip":
        growth_mm = float(module_state.dofs.get("growth_mm", config.tip_length_mm))
        if previous_crawl_mm is None:
            return growth_mm - float(config.tip_length_mm)
        return growth_mm - previous_crawl_mm
    crawl_mm = float(module_state.dofs.get("crawl_mm", 0.0))
    if previous_crawl_mm is None:
        tip_length_mm = float(config.tip_length_mm) if has_tip else 0.0
        return crawl_mm - float(joint_count) * float(config.joint_total_length_mm) - tip_length_mm
    return crawl_mm - previous_crawl_mm


__all__ = [
    "ChainSnapshot",
    "compute_chain_snapshot",
]

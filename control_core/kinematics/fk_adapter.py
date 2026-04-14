"""Compatibility adapter layered on top of the unified geometry API."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..models.module_state import make_joint_module_state, make_tip_module_state
from .chain_kinematics import ChainSnapshot, compute_chain_snapshot
from .frame_conventions import copy_frame
from .module_kinematics import ModuleGeometryConfig


@dataclass(frozen=True)
class JointKinematicState:
    """Kinematic input for one deformable joint module."""

    crawl_mm: float
    bend_deg: float
    rotate_deg: float


@dataclass(frozen=True)
class TipKinematicState:
    """Kinematic input for the tip growth module."""

    growth_mm: float


@dataclass(frozen=True)
class FKResult:
    """Forward-kinematics result expressed as pure numeric arrays."""

    base_origin: np.ndarray
    tip_connection: np.ndarray
    tip_end: np.ndarray
    all_points: list[np.ndarray]
    transform_chain: list[np.ndarray]
    metadata: dict[str, object]

    @property
    def tip_start(self) -> np.ndarray:
        """Primary tip start point; `tip_connection` remains a compatibility alias."""
        return self.tip_connection


class ForwardKinematicsAdapter:
    """Compatibility wrapper for legacy callers that still expect `FKResult`."""

    def __init__(
        self,
        model_source_path: str | None = None,
        *,
        l_top: float = 150.0,
        l1: float = 80.0,
        l2: float = 75.0,
        l3: float = 55.0,
    ) -> None:
        self.model_source_path = model_source_path
        self.geometry_config = ModuleGeometryConfig(
            tip_length_mm=float(l_top),
            joint_segment_1_mm=float(l1),
            joint_segment_2_mm=float(l2),
            joint_segment_3_mm=float(l3),
        )

    def compute_tip_only(self, tip: TipKinematicState) -> FKResult:
        """Compute the tip pose when no joint modules are active."""
        module_states = {"tip": make_tip_module_state(growth_mm=float(tip.growth_mm))}
        snapshot = compute_chain_snapshot(
            module_states,
            ordered_modules=("tip",),
            geometry_config=self.geometry_config,
        )
        return self._result_from_snapshot(snapshot)

    def compute_tip_joint1(
        self,
        tip: TipKinematicState,
        joint1: JointKinematicState,
    ) -> FKResult:
        """Compute the tip pose for the current tip + joint1 model."""
        module_states = {
            "tip": make_tip_module_state(growth_mm=float(tip.growth_mm)),
            "joint1": make_joint_module_state(
                "joint1",
                crawl_mm=float(joint1.crawl_mm),
                bend_deg=float(joint1.bend_deg),
                rotate_deg=float(joint1.rotate_deg),
            ),
        }
        snapshot = compute_chain_snapshot(
            module_states,
            ordered_modules=("tip", "joint1"),
            geometry_config=self.geometry_config,
        )
        return self._result_from_snapshot(snapshot)

    def compute_tip_joint1_joint2(
        self,
        tip: TipKinematicState,
        joint1: JointKinematicState,
        joint2: JointKinematicState,
    ) -> FKResult:
        """Compute the tip pose for the current tip + joint1 + joint2 model."""
        module_states = {
            "tip": make_tip_module_state(growth_mm=float(tip.growth_mm)),
            "joint1": make_joint_module_state(
                "joint1",
                crawl_mm=float(joint1.crawl_mm),
                bend_deg=float(joint1.bend_deg),
                rotate_deg=float(joint1.rotate_deg),
            ),
            "joint2": make_joint_module_state(
                "joint2",
                crawl_mm=float(joint2.crawl_mm),
                bend_deg=float(joint2.bend_deg),
                rotate_deg=float(joint2.rotate_deg),
            ),
        }
        snapshot = compute_chain_snapshot(
            module_states,
            ordered_modules=("tip", "joint1", "joint2"),
            geometry_config=self.geometry_config,
        )
        return self._result_from_snapshot(snapshot)

    def _result_from_snapshot(self, snapshot: ChainSnapshot) -> FKResult:
        named_points = snapshot.copied_named_points()
        metadata = dict(snapshot.metadata)
        metadata["segment_lengths"] = {
            "l_top": float(self.geometry_config.tip_length_mm),
            "l1": float(self.geometry_config.joint_segment_1_mm),
            "l2": float(self.geometry_config.joint_segment_2_mm),
            "l3": float(self.geometry_config.joint_segment_3_mm),
            "joint_total": float(self.geometry_config.joint_total_length_mm),
        }
        metadata["named_points"] = named_points
        metadata["named_frames"] = {
            name: copy_frame(frame)
            for name, frame in snapshot.named_frames.items()
        }
        metadata["tip_point_names"] = {
            "primary_start": "tip_start",
            "end": "tip_end",
        }
        metadata["deprecated_aliases"] = {
            "tip_connection": "tip_start",
        }
        metadata.setdefault("tip_growth_mm", float(snapshot.module_poses["tip"].metadata.get("growth_mm", 0.0)))
        tip_start = np.asarray(named_points["tip_start"], dtype=float).copy()
        return FKResult(
            base_origin=np.asarray(named_points["base_origin"], dtype=float).copy(),
            tip_connection=tip_start,
            tip_end=np.asarray(named_points["tip_end"], dtype=float).copy(),
            all_points=[
                np.asarray(named_points[label], dtype=float).copy()
                for label in list(metadata.get("all_point_labels") or [])
            ],
            transform_chain=[
                np.asarray(transform, dtype=float).copy()
                for transform in list(metadata.get("transform_chain") or [])
            ],
            metadata=metadata,
        )


__all__ = [
    "FKResult",
    "ForwardKinematicsAdapter",
    "JointKinematicState",
    "TipKinematicState",
]

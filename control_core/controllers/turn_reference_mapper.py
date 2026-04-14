"""Heading helpers used by turn supervision and front cooperation control."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any

import numpy as np

from ..kinematics import ChainSnapshot
from ..models.skill_types import SkillSpec
from ..topology.relation_state import RelationState


@dataclass(frozen=True, slots=True)
class FrontCooperateHeadingReference:
    """Resolved heading state used by the front-cooperate controller."""

    tip_heading_current_deg: float
    tip_heading_target_deg: float
    heading_error_deg: float
    joint1_heading_current_deg: float | None
    joint1_bend_current_deg: float
    joint1_rotate_reference: float
    joint1_bend_reference: float


def heading_deg_from_frame(frame: dict[str, np.ndarray] | None) -> float | None:
    """Return the planar heading of one geometry frame from its x-axis."""
    if frame is None:
        return None
    x_axis = frame.get("x_axis")
    if x_axis is None:
        return None
    try:
        resolved_x_axis = np.asarray(x_axis, dtype=float).reshape(3)
    except (TypeError, ValueError):
        return None
    if not np.isfinite(resolved_x_axis).all():
        return None
    return float(np.rad2deg(np.arctan2(float(resolved_x_axis[1]), float(resolved_x_axis[0]))))


def module_heading_deg(
    chain_snapshot: ChainSnapshot | None,
    module_id: str,
    *,
    frame_name: str = "end",
) -> float | None:
    """Resolve the current planar heading for one module from the chain snapshot."""
    if chain_snapshot is None:
        return None
    pose = chain_snapshot.module_poses.get(str(module_id))
    if pose is None:
        return None
    if frame_name == "start":
        return heading_deg_from_frame(pose.start_frame)
    return heading_deg_from_frame(pose.end_frame)


def heading_error_deg(current_heading_deg: float, target_heading_deg: float) -> float:
    """Return the signed shortest heading error in degrees."""
    raw_error = float(current_heading_deg) - float(target_heading_deg)
    wrapped = math.fmod(raw_error + 180.0, 360.0)
    if wrapped < 0.0:
        wrapped += 360.0
    return float(wrapped - 180.0)


def tip_turn_heading_target(
    chain_snapshot: ChainSnapshot | None,
    *,
    target_heading_delta_deg: float,
) -> dict[str, float | None]:
    """Resolve the current and target tip heading for one turn intent."""
    tip_heading_current_deg = module_heading_deg(chain_snapshot, "tip")
    if tip_heading_current_deg is None:
        return {
            "tip_heading_current_deg": None,
            "tip_heading_target_deg": None,
        }
    return {
        "tip_heading_current_deg": float(tip_heading_current_deg),
        "tip_heading_target_deg": float(tip_heading_current_deg) + float(target_heading_delta_deg),
    }


def resolve_front_cooperate_heading_reference(
    relation_state: RelationState,
    spec: SkillSpec,
    *,
    context_metadata: dict[str, object] | None = None,
    rotate_reference_deg_s: float,
    bend_reference_deg_s: float,
) -> FrontCooperateHeadingReference | None:
    """Resolve the heading-tracking reference used by front cooperation."""
    tip_heading_current_deg = _resolve_float(
        spec,
        relation_state,
        "tip_heading_current_deg",
        context_metadata=context_metadata,
        prefer_relation_diagnostics=True,
    )
    tip_heading_target_deg = _resolve_float(
        spec,
        relation_state,
        "tip_heading_target_deg",
        context_metadata=context_metadata,
    )
    if tip_heading_current_deg is None or tip_heading_target_deg is None:
        return None
    joint1_heading_current_deg = _resolve_float(
        spec,
        relation_state,
        "joint1_heading_current_deg",
        context_metadata=context_metadata,
        prefer_relation_diagnostics=True,
    )
    joint1_bend_current_deg = _resolve_float(
        spec,
        relation_state,
        "joint1_bend_current_deg",
        context_metadata=context_metadata,
        prefer_relation_diagnostics=True,
    )
    return FrontCooperateHeadingReference(
        tip_heading_current_deg=float(tip_heading_current_deg),
        tip_heading_target_deg=float(tip_heading_target_deg),
        heading_error_deg=heading_error_deg(tip_heading_current_deg, tip_heading_target_deg),
        joint1_heading_current_deg=joint1_heading_current_deg,
        joint1_bend_current_deg=float(joint1_bend_current_deg or 0.0),
        joint1_rotate_reference=float(rotate_reference_deg_s),
        joint1_bend_reference=float(bend_reference_deg_s),
    )


def _resolve_float(
    spec: SkillSpec,
    relation_state: RelationState,
    name: str,
    *,
    context_metadata: dict[str, object] | None = None,
    prefer_relation_diagnostics: bool = False,
) -> float | None:
    value: Any
    if prefer_relation_diagnostics and name in relation_state.diagnostics:
        value = relation_state.diagnostics.get(name)
    elif name in spec.params:
        value = spec.params.get(name)
    elif name in spec.metadata:
        value = spec.metadata.get(name)
    elif context_metadata is not None and name in context_metadata:
        value = context_metadata.get(name)
    else:
        value = relation_state.diagnostics.get(name)
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


__all__ = [
    "FrontCooperateHeadingReference",
    "heading_deg_from_frame",
    "heading_error_deg",
    "module_heading_deg",
    "resolve_front_cooperate_heading_reference",
    "tip_turn_heading_target",
]

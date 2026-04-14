"""Unified geometry and kinematics layer for `mpc_control_new`."""

from .chain_kinematics import ChainSnapshot, compute_chain_snapshot
from .fk_adapter import FKResult, ForwardKinematicsAdapter, JointKinematicState, TipKinematicState
from .frame_conventions import DEFAULT_FRAME_CONVENTION, FrameConvention, pair_key
from .module_kinematics import ModuleGeometryConfig, ModulePose, compute_module_pose
from .pair_kinematics import (
    PairGeometry,
    compute_pair_distance,
    compute_pair_geometry,
    compute_pair_orientation_error,
)

__all__ = [
    "ChainSnapshot",
    "DEFAULT_FRAME_CONVENTION",
    "FKResult",
    "ForwardKinematicsAdapter",
    "FrameConvention",
    "JointKinematicState",
    "ModuleGeometryConfig",
    "ModulePose",
    "PairGeometry",
    "TipKinematicState",
    "compute_chain_snapshot",
    "compute_module_pose",
    "compute_pair_distance",
    "compute_pair_geometry",
    "compute_pair_orientation_error",
    "pair_key",
]

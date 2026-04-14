"""Shared fake support for turn-planner unit tests."""

from __future__ import annotations

import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mpc_control_new.control_core.estimation import EstimateBundle, JointAvailability, TopologyEstimate

JOINT_IDS = ("joint1", "joint2", "joint3", "joint4", "joint5")


def build_estimate_bundle(*, idle_joint_ids: set[str]) -> EstimateBundle:
    """Build a minimal fake estimate bundle for fixed-range planner tests."""
    joint_availability = {
        joint_id: JointAvailability(
            joint_id=joint_id,
            is_idle=joint_id in idle_joint_ids,
            bend_near_zero=joint_id in idle_joint_ids,
            motion_quiet=True,
            estimate_valid=True,
            reason="idle" if joint_id in idle_joint_ids else "bend_not_near_zero",
        )
        for joint_id in JOINT_IDS
    }
    return EstimateBundle(
        timestamp_ns=1,
        module_states={},
        relation_states={},
        joint_availability=joint_availability,
        topology_estimate=TopologyEstimate(
            ordered_modules=("tip", *JOINT_IDS),
            topology_valid=True,
        ),
    )


def node_signature(node: object) -> tuple[object, ...]:
    """Return the node kind and endpoints for concise assertions."""
    return (
        getattr(node, "node_kind", None),
        getattr(node, "active_module", None),
        getattr(node, "passive_module", None),
        getattr(node, "node_id", None),
    )

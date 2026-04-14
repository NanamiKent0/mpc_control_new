"""Fixed-range idle joint selection policy for the current robot layout."""

from __future__ import annotations

from typing import TYPE_CHECKING, Final

from ..estimation import JointAvailability
from .joint_selection_result import JointSelectionResult

if TYPE_CHECKING:
    from ..estimation import EstimateBundle


JOINT_SELECTION_POLICY_SOURCE = "control_core.supervisor.selection_policy"
FIXED_FRONT_SELECTION_JOINT_IDS: Final[tuple[str, ...]] = (
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
)
FIXED_FRONT_SELECTION_JOINT_INDEX: Final[dict[str, int]] = {
    "joint1": 1,
    "joint2": 2,
    "joint3": 3,
    "joint4": 4,
    "joint5": 5,
}


def can_front_cooperate_now(estimate_bundle: "EstimateBundle") -> bool:
    """Return whether `joint1` can directly enter front cooperation now."""
    joint1_availability = estimate_bundle.get_joint_availability("joint1")
    return bool(joint1_availability is not None and joint1_availability.is_idle)


def find_first_idle_joint_from_joint1_to_joint5(
    estimate_bundle: "EstimateBundle",
) -> JointSelectionResult:
    """Return the first idle joint in the fixed `joint1` to `joint5` range."""
    return _select_first_idle_joint(
        estimate_bundle,
        candidate_joint_ids=FIXED_FRONT_SELECTION_JOINT_IDS,
    )


def select_front_joint_candidate(estimate_bundle: "EstimateBundle") -> JointSelectionResult:
    """Resolve the current front-joint candidate using the robot's fixed workflow."""
    if can_front_cooperate_now(estimate_bundle):
        return _build_selected_result(
            estimate_bundle,
            joint_id="joint1",
            searched_joint_ids=("joint1",),
        )
    return _select_first_idle_joint(
        estimate_bundle,
        candidate_joint_ids=FIXED_FRONT_SELECTION_JOINT_IDS[1:],
        searched_joint_ids_prefix=("joint1",),
    )


def assert_idle_joint_invariant(estimate_bundle: "EstimateBundle") -> None:
    """Raise if the fixed `joint1~joint5` range contains no idle joint."""
    idle_joint_ids = _collect_idle_joint_ids(_build_availability_snapshot(estimate_bundle))
    if idle_joint_ids:
        return
    raise AssertionError(
        "invariant_violated:no_idle_joint_found:expected_idle_joint_within_joint1_to_joint5"
    )


def _select_first_idle_joint(
    estimate_bundle: "EstimateBundle",
    *,
    candidate_joint_ids: tuple[str, ...],
    searched_joint_ids_prefix: tuple[str, ...] = (),
) -> JointSelectionResult:
    availability_snapshot = _build_availability_snapshot(estimate_bundle)
    idle_joint_ids = _collect_idle_joint_ids(availability_snapshot)
    searched_joint_ids = list(searched_joint_ids_prefix)
    for joint_id in candidate_joint_ids:
        if joint_id not in searched_joint_ids:
            searched_joint_ids.append(joint_id)
        if availability_snapshot[joint_id]["is_idle"] is not True:
            continue
        return _build_selected_result(
            estimate_bundle,
            joint_id=joint_id,
            searched_joint_ids=tuple(searched_joint_ids),
            availability_snapshot=availability_snapshot,
            idle_joint_ids=idle_joint_ids,
        )
    return JointSelectionResult(
        selected_joint_id=None,
        selected_joint_index=None,
        direct_front_cooperation=False,
        requires_recursive_transfer=False,
        selection_reason="no_idle_joint_found",
        diagnostics=_build_selection_diagnostics(
            availability_snapshot=availability_snapshot,
            idle_joint_ids=idle_joint_ids,
            searched_joint_ids=tuple(searched_joint_ids),
            selected_joint_id=None,
            selected_joint_index=None,
            invariant_satisfied=False,
            error_code="invariant_violated",
        ),
    )


def _build_selected_result(
    estimate_bundle: "EstimateBundle",
    *,
    joint_id: str,
    searched_joint_ids: tuple[str, ...],
    availability_snapshot: dict[str, dict[str, object]] | None = None,
    idle_joint_ids: list[str] | None = None,
) -> JointSelectionResult:
    availability_snapshot = availability_snapshot or _build_availability_snapshot(estimate_bundle)
    idle_joint_ids = idle_joint_ids or _collect_idle_joint_ids(availability_snapshot)
    joint_index = FIXED_FRONT_SELECTION_JOINT_INDEX[joint_id]
    direct_front_cooperation = joint_id == "joint1"
    return JointSelectionResult(
        selected_joint_id=joint_id,
        selected_joint_index=joint_index,
        direct_front_cooperation=direct_front_cooperation,
        requires_recursive_transfer=not direct_front_cooperation,
        selection_reason=(
            "joint1_idle_direct_front_cooperation"
            if direct_front_cooperation
            else "first_idle_joint_selected_requires_recursive_transfer"
        ),
        diagnostics=_build_selection_diagnostics(
            availability_snapshot=availability_snapshot,
            idle_joint_ids=idle_joint_ids,
            searched_joint_ids=searched_joint_ids,
            selected_joint_id=joint_id,
            selected_joint_index=joint_index,
            invariant_satisfied=True,
        ),
    )


def _build_selection_diagnostics(
    *,
    availability_snapshot: dict[str, dict[str, object]],
    idle_joint_ids: list[str],
    searched_joint_ids: tuple[str, ...],
    selected_joint_id: str | None,
    selected_joint_index: int | None,
    invariant_satisfied: bool,
    error_code: str | None = None,
) -> dict[str, object]:
    diagnostics: dict[str, object] = {
        "selection_policy_source": JOINT_SELECTION_POLICY_SOURCE,
        "joint_order": list(FIXED_FRONT_SELECTION_JOINT_IDS),
        "searched_joint_ids": list(searched_joint_ids),
        "selected_joint_id": selected_joint_id,
        "selected_joint_index": selected_joint_index,
        "joint1_idle": availability_snapshot["joint1"]["is_idle"] is True,
        "idle_joint_ids": list(idle_joint_ids),
        "idle_joint_count": len(idle_joint_ids),
        "invariant_satisfied": invariant_satisfied,
        "missing_joint_availability_ids": [
            joint_id
            for joint_id, snapshot in availability_snapshot.items()
            if snapshot["availability_present"] is not True
        ],
        "availability_snapshot": availability_snapshot,
    }
    if error_code is not None:
        diagnostics["error_code"] = error_code
    return diagnostics


def _build_availability_snapshot(
    estimate_bundle: "EstimateBundle",
) -> dict[str, dict[str, object]]:
    snapshot: dict[str, dict[str, object]] = {}
    for joint_id in FIXED_FRONT_SELECTION_JOINT_IDS:
        snapshot[joint_id] = _availability_snapshot_entry(
            estimate_bundle.get_joint_availability(joint_id),
        )
    return snapshot


def _availability_snapshot_entry(
    availability: JointAvailability | None,
) -> dict[str, object]:
    if availability is None:
        return {
            "availability_present": False,
            "is_idle": None,
            "reason": "availability_missing",
            "bend_near_zero": None,
            "motion_quiet": None,
            "estimate_valid": None,
            "diagnostics": {},
        }
    return {
        "availability_present": True,
        "is_idle": bool(availability.is_idle),
        "reason": availability.reason,
        "bend_near_zero": bool(availability.bend_near_zero),
        "motion_quiet": bool(availability.motion_quiet),
        "estimate_valid": bool(availability.estimate_valid),
        "diagnostics": dict(availability.diagnostics),
    }


def _collect_idle_joint_ids(
    availability_snapshot: dict[str, dict[str, object]],
) -> list[str]:
    return [
        joint_id
        for joint_id in FIXED_FRONT_SELECTION_JOINT_IDS
        if availability_snapshot[joint_id]["is_idle"] is True
    ]


__all__ = [
    "FIXED_FRONT_SELECTION_JOINT_IDS",
    "JOINT_SELECTION_POLICY_SOURCE",
    "assert_idle_joint_invariant",
    "can_front_cooperate_now",
    "find_first_idle_joint_from_joint1_to_joint5",
    "select_front_joint_candidate",
]

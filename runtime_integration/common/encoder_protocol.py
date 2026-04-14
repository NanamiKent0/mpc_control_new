"""Self-contained encoder/motor protocol helpers used by the new runtime GUI path.

This module intentionally carries the minimum protocol surface that
`runtime_integration/gui/gui_ros2.py` and the new sim/ROS2 bridges need so
`mpc_control_new` can run after relocation without importing the old project.
"""

from __future__ import annotations

from dataclasses import dataclass
import re


LEGACY_FEEDBACK_LEN = 2
EXTENDED_FEEDBACK_LEN = 6

STATUS_BOARD_ALIVE = 1 << 0
STATUS_CONTROL_ACTIVE = 1 << 1
STATUS_IS_MOVING = 1 << 2
STATUS_STALE_SOURCE = 1 << 3

SOURCE_LOCAL_BOARD = 0
SOURCE_SLAVE_FORWARDED = 1
SOURCE_RESERVED = 2

MOTOR_CONVERSION_PARAMS: dict[int, dict[str, float | str]] = {
    1: {
        "name": "linear_motor",
        "mm_per_revolution": 18.4,
        "counts_per_revolution": 4550,
        "unit_suffix": " mm",
    },
    2: {
        "name": "rotate_motor",
        "deg_per_revolution": 56.25,
        "counts_per_revolution": 11837,
        "unit_suffix": " °",
    },
    3: {
        "name": "bend_motor",
        "deg_per_revolution": 105.0,
        "counts_per_revolution": 11837,
        "unit_suffix": " °",
    },
    4: {
        "name": "tip_motor",
        "mm_per_revolution": 30.16,
        "counts_per_revolution": 11837,
        "unit_suffix": " mm",
    },
}

NAMESPACE_TO_MOTOR_IDS: dict[str, list[int]] = {
    "tip": [4],
    "joint1": [1, 2, 3],
    "joint2": [1, 2, 3],
    "joint3": [1, 2, 3],
    "joint4": [1, 2, 3],
    "joint5": [1, 2, 3],
}

JOINT_NAME_TO_NAMESPACE: dict[str, str] = {
    "尖端": "tip",
    "关节1": "joint1",
    "关节2": "joint2",
    "关节3": "joint3",
    "关节4": "joint4",
    "关节5": "joint5",
}


@dataclass(slots=True)
class ParsedFeedback:
    """Normalized feedback frame decoded from a compact or extended array."""

    motor_id: int
    raw_pulses: int
    device_time_ms: int | None = None
    seq: int | None = None
    status_flags: int | None = None
    source_id: int | None = None
    is_extended: bool = False


def _normalize_namespace_or_joint_name(value: object | None) -> str | None:
    """Normalize one namespace or GUI joint label."""
    if value is None:
        return None
    return str(value).strip().lower().strip("/")


def motor_ids_for_namespace_or_joint_name(
    value: object | None = None,
    *,
    namespace: str | None = None,
    joint_name: str | None = None,
) -> list[int]:
    """Resolve the motor ids targeted by one namespace or human joint label."""
    target = value
    if namespace is not None:
        target = namespace
    elif joint_name is not None:
        target = joint_name
    if target is None:
        return []
    if target in JOINT_NAME_TO_NAMESPACE:
        target = JOINT_NAME_TO_NAMESPACE[str(target)]
    normalized = _normalize_namespace_or_joint_name(target)
    if normalized in NAMESPACE_TO_MOTOR_IDS:
        return list(NAMESPACE_TO_MOTOR_IDS[normalized])
    if normalized == "tip":
        return [4]
    if normalized is not None and normalized.startswith("joint"):
        return [1, 2, 3]
    if re.fullmatch(r"关节\d+", str(target).strip()):
        return [1, 2, 3]
    if str(target).strip() == "尖端":
        return [4]
    return []


def counts_to_physical(motor_id: int, counts: float | int) -> tuple[float, str]:
    """Convert encoder counts into the GUI's physical unit for one motor id."""
    params = MOTOR_CONVERSION_PARAMS[motor_id]
    counts_value = float(counts)
    if "mm_per_revolution" in params:
        value = counts_value * float(params["mm_per_revolution"]) / float(
            params["counts_per_revolution"]
        )
    elif "deg_per_revolution" in params:
        value = counts_value * float(params["deg_per_revolution"]) / float(
            params["counts_per_revolution"]
        )
    else:
        value = counts_value
    return value, str(params.get("unit_suffix", ""))


def physical_to_counts(motor_id: int, physical_value: float | int) -> float:
    """Convert one GUI-side physical target back into encoder counts per second."""
    params = MOTOR_CONVERSION_PARAMS[motor_id]
    value = float(physical_value)
    if "mm_per_revolution" in params:
        return value / float(params["mm_per_revolution"]) * float(
            params["counts_per_revolution"]
        )
    if "deg_per_revolution" in params:
        return value / float(params["deg_per_revolution"]) * float(
            params["counts_per_revolution"]
        )
    return value


def parse_feedback_array(data: object) -> ParsedFeedback:
    """Decode one compact or extended motor feedback array."""
    values = list(data)  # type: ignore[arg-type]
    if len(values) < LEGACY_FEEDBACK_LEN:
        raise ValueError(f"feedback_fields_too_short:{len(values)}")
    parsed = ParsedFeedback(
        motor_id=int(values[0]),
        raw_pulses=int(values[1]),
    )
    if len(values) >= EXTENDED_FEEDBACK_LEN:
        parsed.device_time_ms = int(values[2])
        parsed.seq = int(values[3])
        parsed.status_flags = int(values[4])
        parsed.source_id = int(values[5])
        parsed.is_extended = True
    return parsed


__all__ = [
    "EXTENDED_FEEDBACK_LEN",
    "JOINT_NAME_TO_NAMESPACE",
    "LEGACY_FEEDBACK_LEN",
    "MOTOR_CONVERSION_PARAMS",
    "NAMESPACE_TO_MOTOR_IDS",
    "ParsedFeedback",
    "SOURCE_LOCAL_BOARD",
    "SOURCE_RESERVED",
    "SOURCE_SLAVE_FORWARDED",
    "STATUS_BOARD_ALIVE",
    "STATUS_CONTROL_ACTIVE",
    "STATUS_IS_MOVING",
    "STATUS_STALE_SOURCE",
    "counts_to_physical",
    "motor_ids_for_namespace_or_joint_name",
    "parse_feedback_array",
    "physical_to_counts",
]

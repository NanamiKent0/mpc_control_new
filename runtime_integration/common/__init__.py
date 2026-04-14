"""Common runtime-integration helpers shared by GUI, sim, and ROS2 paths."""

from .encoder_protocol import (
    EXTENDED_FEEDBACK_LEN,
    LEGACY_FEEDBACK_LEN,
    MOTOR_CONVERSION_PARAMS,
    NAMESPACE_TO_MOTOR_IDS,
    STATUS_STALE_SOURCE,
    ParsedFeedback,
    counts_to_physical,
    motor_ids_for_namespace_or_joint_name,
    parse_feedback_array,
    physical_to_counts,
)

__all__ = [
    "EXTENDED_FEEDBACK_LEN",
    "LEGACY_FEEDBACK_LEN",
    "MOTOR_CONVERSION_PARAMS",
    "NAMESPACE_TO_MOTOR_IDS",
    "STATUS_STALE_SOURCE",
    "ParsedFeedback",
    "counts_to_physical",
    "motor_ids_for_namespace_or_joint_name",
    "parse_feedback_array",
    "physical_to_counts",
]

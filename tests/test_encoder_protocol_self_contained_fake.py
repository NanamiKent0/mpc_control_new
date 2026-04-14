"""Fake tests for the self-contained encoder protocol used by gui_ros2."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mpc_control_new.runtime_integration.common import encoder_protocol


class EncoderProtocolSelfContainedFakeTest(unittest.TestCase):
    """Verify the copied protocol surface is self-contained and usable."""

    def test_protocol_resolves_from_new_package(self) -> None:
        self.assertIn("runtime_integration/common/encoder_protocol.py", encoder_protocol.__file__)

    def test_gui_required_symbols_exist_and_round_trip(self) -> None:
        counts = encoder_protocol.physical_to_counts(1, 18.4)
        value, suffix = encoder_protocol.counts_to_physical(1, counts)

        self.assertAlmostEqual(value, 18.4, places=3)
        self.assertEqual(suffix, " mm")
        self.assertEqual(encoder_protocol.motor_ids_for_namespace_or_joint_name(namespace="tip"), [4])
        self.assertEqual(
            encoder_protocol.motor_ids_for_namespace_or_joint_name(joint_name="关节3"),
            [1, 2, 3],
        )

    def test_parse_feedback_array_supports_extended_layout(self) -> None:
        parsed = encoder_protocol.parse_feedback_array([1, 123, 45, 6, 7, 8])

        self.assertEqual(parsed.motor_id, 1)
        self.assertEqual(parsed.raw_pulses, 123)
        self.assertTrue(parsed.is_extended)
        self.assertEqual(parsed.status_flags, 7)


if __name__ == "__main__":
    unittest.main()

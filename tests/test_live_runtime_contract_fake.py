"""Fake tests for live runtime provider/dispatcher skeleton contracts."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mpc_control_new.control_core.models.runtime_bridge_types import SchedulerDispatchEnvelope
from mpc_control_new.control_core.models.task_types import SchedulerState
from mpc_control_new.runtime_integration.live_command_dispatcher import LiveRuntimeCommandDispatcher
from mpc_control_new.runtime_integration.live_runtime_provider import LiveRuntimeObservationProvider


class LiveRuntimeContractFakeTest(unittest.TestCase):
    """Verify live runtime skeletons keep the contract stable and structured."""

    def test_live_provider_returns_none_without_throwing(self) -> None:
        """The provider should return structured unavailable state without throwing."""
        provider = LiveRuntimeObservationProvider()
        provider.start()

        self.assertIsNone(provider.warmup())
        self.assertIsNone(provider.get_latest_frame())
        self.assertFalse(provider.is_ready())
        self.assertEqual(provider.last_reason, "ros2_unavailable")
        self.assertEqual(provider.runtime_diagnostics()["provider_kind"], "ros2")
        self.assertEqual(provider.runtime_diagnostics()["ros2_available"], False)

    def test_live_dispatcher_rejects_structurally_without_throwing(self) -> None:
        """The dispatcher should return structured unavailable results instead of raw errors."""
        dispatcher = LiveRuntimeCommandDispatcher()
        dispatcher.start()
        result = dispatcher.dispatch(
            SchedulerDispatchEnvelope(
                scheduler_state=SchedulerState(graph_id="live_contract_graph"),
                scheduled_command=None,
            )
        )
        stop_result = dispatcher.emergency_stop()
        flush_result = dispatcher.flush()

        self.assertFalse(result.accepted)
        self.assertEqual(result.reason, "ros2_unavailable")
        self.assertFalse(stop_result.accepted)
        self.assertFalse(flush_result.accepted)
        self.assertEqual(result.diagnostics["dispatcher"], "live_command_dispatcher")
        self.assertEqual(result.diagnostics["dispatcher_kind"], "ros2")


if __name__ == "__main__":
    unittest.main()

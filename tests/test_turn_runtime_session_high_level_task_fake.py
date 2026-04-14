"""Fake runtime-session test for high-level turn task loading."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from tests.turn_workflow_fake_support import build_turn_runtime_frame

from mpc_control_new.control_core.models.runtime_bridge_types import DispatchResult, SchedulerDispatchEnvelope
from mpc_control_new.control_core.models.task_types import (
    HighLevelTaskRequest,
    TIP_TURN_AUTONOMOUS,
)
from mpc_control_new.runtime_integration.runtime_session import RuntimeSession


class _StaticProvider:
    """Provider double that exposes one stable runtime frame."""

    def __init__(self, frame) -> None:
        self.frame = frame
        self.started = False
        self.start_calls = 0
        self.stop_calls = 0
        self.warmup_calls = 0

    def start(self) -> None:
        self.started = True
        self.start_calls += 1

    def stop(self) -> None:
        self.started = False
        self.stop_calls += 1

    def warmup(self):
        self.warmup_calls += 1
        return self.frame

    def is_ready(self) -> bool:
        return self.started

    def get_latest_frame(self):
        return self.frame if self.started else None


class _RecordingDispatcher:
    """Dispatcher double that records the last envelope."""

    def __init__(self) -> None:
        self.started = False
        self.last_envelope: SchedulerDispatchEnvelope | None = None

    def start(self) -> None:
        self.started = True

    def stop(self) -> None:
        self.started = False

    def dispatch(self, envelope: SchedulerDispatchEnvelope) -> DispatchResult:
        self.last_envelope = envelope
        return DispatchResult(
            accepted=True,
            dispatched_commands=["fake_dispatch"],
            reason="accepted_by_test",
            diagnostics={"dispatcher": "recording_dispatcher"},
        )

    def emergency_stop(self) -> DispatchResult:
        return DispatchResult(accepted=True, dispatched_commands=["stop"], reason="stopped")


class TurnRuntimeSessionHighLevelTaskFakeTest(unittest.TestCase):
    """Verify runtime session can compile and run a high-level turn request."""

    def test_load_high_level_turn_task_compiles_from_provider_frame(self) -> None:
        provider = _StaticProvider(build_turn_runtime_frame(idle_joint_ids={"joint2"}))
        dispatcher = _RecordingDispatcher()
        session = RuntimeSession(
            observation_provider=provider,
            command_dispatcher=dispatcher,
            graph_spec=None,
        )

        graph = session.load_high_level_task(
            HighLevelTaskRequest(
                task_kind=TIP_TURN_AUTONOMOUS,
                metadata={"target_heading_delta_deg": 0.0},
            )
        )
        result = session.step()

        self.assertEqual(provider.start_calls, 2)
        self.assertEqual(provider.warmup_calls, 2)
        self.assertEqual(provider.stop_calls, 1)
        self.assertEqual(graph.metadata["high_level_task_kind"], TIP_TURN_AUTONOMOUS)
        self.assertEqual(graph.metadata["target_heading_delta_deg"], 0.0)
        self.assertEqual(session.last_turn_plan.selected_joint_id, "joint2")
        self.assertTrue(result.accepted)
        self.assertEqual(result.high_level_task_kind, TIP_TURN_AUTONOMOUS)
        self.assertEqual(result.diagnostics["target_heading_delta_deg"], 0.0)
        self.assertEqual(result.diagnostics["selected_joint_id"], "joint2")
        self.assertEqual(result.diagnostics["current_plan_node_kind"], "local_transfer")


if __name__ == "__main__":
    unittest.main()

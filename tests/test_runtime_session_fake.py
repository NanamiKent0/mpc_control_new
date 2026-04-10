"""Fake tests for the runtime-facing session main path."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mpc_control_new.control_core.models.runtime_bridge_types import DispatchResult, SchedulerDispatchEnvelope
from mpc_control_new.control_core.orchestration.graph_factories import build_runtime_demo_pair_graph
from mpc_control_new.runtime_integration.observation_types import (
    ModuleObservation,
    PairObservation,
    RuntimeObservationFrame,
)
from mpc_control_new.runtime_integration.runtime_session import RuntimeSession


def _frame(distance_mm: float = 20.0, orientation_error_deg: float = -8.0) -> RuntimeObservationFrame:
    """Build a minimal runtime frame for session tests."""
    return RuntimeObservationFrame(
        timestamp_ns=10,
        module_observations={
            "joint1": ModuleObservation(module_id="joint1", module_type="joint", dofs={"crawl_mm": 0.0}),
            "tip": ModuleObservation(module_id="tip", module_type="tip", dofs={"growth_mm": 0.0}),
        },
        pair_observations={
            "joint1->tip": PairObservation(
                active_module="joint1",
                passive_module="tip",
                relation_type="tip_joint",
                distance_mm=distance_mm,
                orientation_error_deg=orientation_error_deg,
                coupled=False,
                observation_valid=True,
                source_name="runtime_session_fake",
            )
        },
        topology_hint={"ordered_modules": ["tip", "joint1", "joint2"]},
        metadata={"source_name": "runtime_session_fake"},
    )


class _StaticProvider:
    """Provider double that returns the same frame until replaced."""

    def __init__(self, frame: RuntimeObservationFrame | None) -> None:
        self.frame = frame
        self.started = False

    def start(self) -> None:
        self.started = True

    def stop(self) -> None:
        self.started = False

    def warmup(self) -> RuntimeObservationFrame | None:
        return self.frame

    def is_ready(self) -> bool:
        return self.started

    def get_latest_frame(self) -> RuntimeObservationFrame | None:
        return self.frame if self.started else None


class _RecordingDispatcher:
    """Dispatcher double that records the last envelope and returns a fixed result."""

    def __init__(self, *, accepted: bool = True, reason: str = "accepted_by_test") -> None:
        self.accepted = accepted
        self.reason = reason
        self.started = False
        self.last_envelope: SchedulerDispatchEnvelope | None = None

    def start(self) -> None:
        self.started = True

    def stop(self) -> None:
        self.started = False

    def dispatch(self, envelope: SchedulerDispatchEnvelope) -> DispatchResult:
        self.last_envelope = envelope
        return DispatchResult(
            accepted=self.accepted,
            dispatched_commands=["fake_dispatch"] if self.accepted else [],
            reason=self.reason,
            diagnostics={"dispatcher": "recording_dispatcher"},
        )

    def emergency_stop(self) -> DispatchResult:
        return DispatchResult(accepted=True, dispatched_commands=["stop"], reason="stopped")

    def flush(self) -> DispatchResult:
        return DispatchResult(accepted=True, dispatched_commands=[], reason="flushed")


class RuntimeSessionFakeTest(unittest.TestCase):
    """Verify the new runtime session main path works without legacy estimates."""

    def test_runtime_session_runs_provider_scheduler_dispatch_chain(self) -> None:
        """A runtime frame should flow through the scheduler and into a dispatcher envelope."""
        provider = _StaticProvider(_frame())
        dispatcher = _RecordingDispatcher()
        session = RuntimeSession(
            observation_provider=provider,
            command_dispatcher=dispatcher,
            graph_spec=build_runtime_demo_pair_graph(
                graph_id="runtime_session_fake_graph",
                active_module="joint1",
                passive_module="tip",
                relation_type="tip_joint",
                coarse_distance_threshold_mm=12.0,
                dock_distance_done_mm=2.0,
                dock_orientation_done_deg=5.0,
                include_finalize_node=False,
            ),
        )

        result = session.step()

        self.assertTrue(result.accepted)
        self.assertIsNotNone(result.scheduler_step_result)
        self.assertIsNotNone(result.dispatch_envelope)
        self.assertIsNotNone(result.dispatch_result)
        self.assertEqual(result.scheduler_step_result.diagnostics["scheduler_input_source"], "runtime_frame")
        self.assertEqual(result.dispatch_envelope.context_metadata["graph_id"], "runtime_session_fake_graph")
        self.assertEqual(result.dispatch_envelope.context_metadata["runtime_session"], True)
        self.assertEqual(dispatcher.last_envelope.scheduled_command.skill_key, "coarse_approach")

    def test_runtime_session_returns_structured_error_when_observation_is_missing(self) -> None:
        """Missing provider frames should produce a structured step result."""
        session = RuntimeSession(
            observation_provider=_StaticProvider(None),
            command_dispatcher=_RecordingDispatcher(),
            graph_spec=build_runtime_demo_pair_graph(
                graph_id="runtime_session_missing_frame",
                active_module="joint1",
                passive_module="tip",
                relation_type="tip_joint",
                coarse_distance_threshold_mm=12.0,
                dock_distance_done_mm=2.0,
                dock_orientation_done_deg=5.0,
                include_finalize_node=False,
            ),
        )

        result = session.step()

        self.assertFalse(result.accepted)
        self.assertEqual(result.reason, "observation_unavailable")
        self.assertIsNone(result.scheduler_step_result)
        self.assertIsNone(result.dispatch_envelope)

    def test_runtime_session_surfaces_dispatch_rejection_structurally(self) -> None:
        """Dispatcher rejection should be visible on the runtime-step result."""
        session = RuntimeSession(
            observation_provider=_StaticProvider(_frame()),
            command_dispatcher=_RecordingDispatcher(accepted=False, reason="rejected_by_test"),
            graph_spec=build_runtime_demo_pair_graph(
                graph_id="runtime_session_reject_graph",
                active_module="joint1",
                passive_module="tip",
                relation_type="tip_joint",
                coarse_distance_threshold_mm=12.0,
                dock_distance_done_mm=2.0,
                dock_orientation_done_deg=5.0,
                include_finalize_node=False,
            ),
        )

        result = session.step()

        self.assertFalse(result.accepted)
        self.assertEqual(result.reason, "rejected_by_test")
        self.assertEqual(result.dispatch_result.reason, "rejected_by_test")
        self.assertEqual(result.scheduler_step_result.diagnostics["dispatch_ready"], True)


if __name__ == "__main__":
    unittest.main()

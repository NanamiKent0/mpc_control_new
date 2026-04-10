"""Fake tests ensuring the runtime main path no longer depends on legacy extractors."""

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
from mpc_control_new.runtime_integration.runtime_state_builder import RUNTIME_STATE_BUILDER_SOURCE


class _OneFrameProvider:
    """Provider double that exposes exactly one runtime frame repeatedly."""

    def __init__(self, frame: RuntimeObservationFrame) -> None:
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


class _AcceptingDispatcher:
    """Dispatcher double that accepts any scheduler envelope."""

    def start(self) -> None:
        return None

    def stop(self) -> None:
        return None

    def dispatch(self, envelope: SchedulerDispatchEnvelope) -> DispatchResult:
        return DispatchResult(
            accepted=True,
            dispatched_commands=["accepted"],
            reason="accepted",
            diagnostics={"node_id": None if envelope.scheduled_command is None else envelope.scheduled_command.node_id},
        )

    def emergency_stop(self) -> DispatchResult:
        return DispatchResult(accepted=True, dispatched_commands=["stop"], reason="stop")

    def flush(self) -> DispatchResult:
        return DispatchResult(accepted=True, dispatched_commands=[], reason="flush")


class LegacyIndependentMainPathFakeTest(unittest.TestCase):
    """Verify the runtime-facing main path does not require legacy estimates."""

    def test_runtime_session_executes_from_runtime_frame_without_legacy_estimate(self) -> None:
        """A plain runtime frame should be enough to drive the new main path."""
        frame = RuntimeObservationFrame(
            timestamp_ns=5,
            module_observations={
                "joint1": ModuleObservation(module_id="joint1", module_type="joint", dofs={"crawl_mm": 0.0}),
                "tip": ModuleObservation(module_id="tip", module_type="tip", dofs={"growth_mm": 0.0}),
            },
            pair_observations={
                "joint1->tip": PairObservation(
                    active_module="joint1",
                    passive_module="tip",
                    relation_type="tip_joint",
                    distance_mm=16.0,
                    orientation_error_deg=-8.0,
                    coupled=False,
                    observation_valid=True,
                    source_name="runtime_frame_only",
                )
            },
            topology_hint={"ordered_modules": ["tip", "joint1", "joint2"]},
            metadata={"source_name": "runtime_frame_only"},
        )
        session = RuntimeSession(
            observation_provider=_OneFrameProvider(frame),
            command_dispatcher=_AcceptingDispatcher(),
            graph_spec=build_runtime_demo_pair_graph(
                graph_id="legacy_independent_main_path",
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
        skill_result = result.scheduler_step_result.skill_result

        self.assertTrue(result.accepted)
        self.assertEqual(result.diagnostics["input_source"], "runtime_frame")
        self.assertEqual(result.scheduler_step_result.diagnostics["scheduler_input_source"], "runtime_frame")
        self.assertEqual(result.scheduler_step_result.diagnostics["legacy_path_used"], False)
        self.assertEqual(skill_result.diagnostics["input_source"], "runtime_frame")
        self.assertEqual(skill_result.diagnostics["state_builder_source"], RUNTIME_STATE_BUILDER_SOURCE)
        self.assertNotIn("pair_extractor_key", skill_result.relation_state.diagnostics)


if __name__ == "__main__":
    unittest.main()

"""Shared support helpers for turn-workflow orchestration tests."""

from __future__ import annotations

import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mpc_control_new.runtime_integration.observation_types import (
    ModuleObservation,
    PairObservation,
    RuntimeObservationFrame,
)
from mpc_control_new.control_core.models.runtime_bridge_types import DispatchResult, SchedulerDispatchEnvelope


ORDERED_MODULES = ("tip", "joint1", "joint2", "joint3", "joint4", "joint5")
ADJACENT_PAIRS = (
    ("joint1", "tip"),
    ("joint2", "joint1"),
    ("joint3", "joint2"),
    ("joint4", "joint3"),
    ("joint5", "joint4"),
)


def build_turn_runtime_frame(
    *,
    idle_joint_ids: set[str],
    pair_overrides: dict[str, dict[str, object]] | None = None,
    module_overrides: dict[str, dict[str, object]] | None = None,
    timestamp_ns: int = 100,
) -> RuntimeObservationFrame:
    """Build a runtime frame that can drive the fixed turn workflow end to end."""
    overrides = dict(pair_overrides or {})
    module_override_map = dict(module_overrides or {})
    module_observations = {
        "tip": ModuleObservation(
            **{
                "module_id": "tip",
                "module_type": "tip",
                "dofs": {"growth_mm": 0.0},
                "velocities": {"growth_mm_s": 0.0},
                "attach_state": {"joint1": False},
                "source_name": "turn_workflow_fake",
                **module_override_map.get("tip", {}),
            }
        )
    }
    for joint_id in ORDERED_MODULES[1:]:
        module_observations[joint_id] = ModuleObservation(
            **{
                "module_id": joint_id,
                "module_type": "joint",
                "dofs": {
                    "crawl_mm": 0.0,
                    "bend_deg": 0.0 if joint_id in idle_joint_ids else 12.0,
                    "rotate_deg": 0.0,
                },
                "velocities": {
                    "crawl_mm_s": 0.0,
                    "bend_deg_s": 0.0,
                    "rotate_deg_s": 0.0,
                },
                "attach_state": {},
                "source_name": "turn_workflow_fake",
                **module_override_map.get(joint_id, {}),
            }
        )
    pair_observations: dict[str, PairObservation] = {}
    for active_module, passive_module in ADJACENT_PAIRS:
        pair_label = f"{active_module}->{passive_module}"
        values = {
            "active_module": active_module,
            "passive_module": passive_module,
            "relation_type": "tip_joint" if passive_module == "tip" else "joint_joint",
            "distance_mm": 0.5,
            "orientation_error_deg": 0.5,
            "coupled": True,
            "observation_valid": True,
            "diagnostics": {"cooperate_progress": 1.0},
            "source_name": "turn_workflow_fake",
        }
        values.update(overrides.get(pair_label, {}))
        pair_observations[pair_label] = PairObservation(**values)
    return RuntimeObservationFrame(
        timestamp_ns=timestamp_ns,
        module_observations=module_observations,
        pair_observations=pair_observations,
        topology_hint={"ordered_modules": list(ORDERED_MODULES)},
        metadata={"source_name": "turn_workflow_fake"},
    )


def run_scheduler_until_finished(
    scheduler: object,
    frame_source: RuntimeObservationFrame | list[RuntimeObservationFrame] | tuple[RuntimeObservationFrame, ...],
    *,
    max_steps: int = 20,
) -> list[object]:
    """Run one scheduler against a stable runtime frame until it finishes."""
    results: list[object] = []
    for _ in range(max_steps):
        frame = _frame_for_step(frame_source, len(results))
        result = scheduler.step(frame)
        results.append(result)
        if result.scheduler_state.is_finished:
            return results
    raise AssertionError(f"turn_workflow_scheduler_not_finished_within:{max_steps}")


def _frame_for_step(
    frame_source: RuntimeObservationFrame | list[RuntimeObservationFrame] | tuple[RuntimeObservationFrame, ...],
    step_index: int,
) -> RuntimeObservationFrame:
    """Return the frame used for one scheduler step."""
    if isinstance(frame_source, RuntimeObservationFrame):
        return frame_source
    if not frame_source:
        raise AssertionError("turn_workflow_frame_source_empty")
    return frame_source[min(step_index, len(frame_source) - 1)]


class SequenceProvider:
    """Provider double that advances through a fixed frame sequence."""

    def __init__(self, frames: list[RuntimeObservationFrame] | tuple[RuntimeObservationFrame, ...]) -> None:
        if not frames:
            raise ValueError("sequence_provider_requires_frames")
        self.frames = list(frames)
        self.started = False
        self.index = 0

    def start(self) -> None:
        self.started = True

    def stop(self) -> None:
        self.started = False

    def warmup(self) -> RuntimeObservationFrame | None:
        return self.frames[min(self.index, len(self.frames) - 1)]

    def is_ready(self) -> bool:
        return self.started

    def get_latest_frame(self) -> RuntimeObservationFrame | None:
        if not self.started:
            return None
        frame = self.frames[min(self.index, len(self.frames) - 1)]
        if self.index < len(self.frames) - 1:
            self.index += 1
        return frame


class StaticProvider(SequenceProvider):
    """Provider double that keeps returning the same frame."""

    def __init__(self, frame: RuntimeObservationFrame) -> None:
        super().__init__([frame])


class RecordingDispatcher:
    """Dispatcher double that records the last envelope and always accepts."""

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

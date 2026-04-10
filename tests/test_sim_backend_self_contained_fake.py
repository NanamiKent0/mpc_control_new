"""Fake tests for the self-contained sim backend."""

from __future__ import annotations

import inspect
import sys
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mpc_control_new.control_core.models.runtime_bridge_types import SchedulerDispatchEnvelope, ScheduledSkillCommand
from mpc_control_new.control_core.models.skill_types import PrimitiveReference
from mpc_control_new.control_core.models.task_types import SchedulerState
from mpc_control_new.runtime_integration.sim_backend import SimRuntimeBackend, SimState
from mpc_control_new.runtime_integration.sim_command_dispatcher import SimCommandDispatcher
from mpc_control_new.runtime_integration.sim_observation_provider import SimObservationProvider


class SimBackendSelfContainedFakeTest(unittest.TestCase):
    """Verify the new sim path is fully internal to `mpc_control_new`."""

    def test_sim_backend_types_live_inside_mpc_control_new(self) -> None:
        """The sim backend should resolve from the new package instead of legacy imports."""
        source_path = Path(inspect.getsourcefile(SimState) or "")
        self.assertIn("mpc_control_new/runtime_integration/sim_backend", str(source_path))

    def test_provider_dispatcher_and_backend_work_together(self) -> None:
        """The self-contained sim backend should support provider and dispatcher smoke coverage."""
        backend = SimRuntimeBackend(
            state=SimState(
                tip_joint1_distance_mm=20.0,
                tip_joint1_orientation_error_deg=-10.0,
                tip_joint1_coupled=False,
            ),
            dt=1.0,
        )
        provider = SimObservationProvider(backend=backend)
        dispatcher = SimCommandDispatcher(backend=backend)
        provider.start()
        dispatcher.start()

        envelope = SchedulerDispatchEnvelope(
            scheduler_state=SchedulerState(graph_id="sim_backend_fake"),
            scheduled_command=ScheduledSkillCommand(
                graph_id="sim_backend_fake",
                node_id="coarse_approach",
                skill_key="coarse_approach",
                active_module="joint1",
                passive_module="tip",
                relation_type="tip_joint",
                selected_primitives=["joint_crawl"],
                primitive_references=[
                    PrimitiveReference(
                        module_id="joint1",
                        primitive_name="joint_crawl",
                        axis="crawl",
                        reference_kind="velocity",
                        reference_value=5.0,
                        units="mm/s",
                        primary=True,
                        semantic="reduce_distance",
                    )
                ],
            ),
            input_source="runtime_frame:sim",
            dispatch_target="sim",
            provider_kind="sim",
            dispatcher_kind="sim",
        )

        before = provider.get_latest_frame()
        result = dispatcher.dispatch(envelope)
        after = provider.get_latest_frame()

        self.assertTrue(result.accepted)
        self.assertEqual(result.diagnostics["dispatch_target"], "sim")
        self.assertEqual(result.diagnostics["accepted_modules"], ["joint1"])
        self.assertEqual(before.metadata["provider_source"], "self_contained_sim_backend")
        self.assertLess(after.get_pair_observation("joint1", "tip").distance_mm, before.get_pair_observation("joint1", "tip").distance_mm)


if __name__ == "__main__":
    unittest.main()

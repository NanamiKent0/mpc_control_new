"""Fake tests for the unchanged `joint-joint` coupling rule."""

from __future__ import annotations

import unittest

from mpc_control_new.control_core.controllers.fine_dock_controller import FineDockController
from mpc_control_new.control_core.estimation import ModuleStateEstimator, RelationEstimator
from mpc_control_new.control_core.kinematics import compute_chain_snapshot
from mpc_control_new.control_core.models.runtime_bridge_types import SchedulerDispatchEnvelope, ScheduledSkillCommand
from mpc_control_new.control_core.models.skill_types import SkillSpec
from mpc_control_new.control_core.models.task_types import SchedulerState
from mpc_control_new.control_core.topology.relation_state import RelationState
from mpc_control_new.runtime_integration.observation_types import ModuleObservation, PairObservation, RuntimeObservationFrame
from mpc_control_new.runtime_integration.sim_backend import SimCommand, SimRuntimeBackend, SimState


def _joint_joint_frame(*, coupled: bool | None) -> RuntimeObservationFrame:
    return RuntimeObservationFrame(
        timestamp_ns=100,
        module_observations={
            "tip": ModuleObservation(module_id="tip", module_type="tip", dofs={"growth_mm": 150.0}),
            "joint1": ModuleObservation(
                module_id="joint1",
                module_type="joint",
                dofs={"crawl_mm": 220.0, "bend_deg": 0.0, "rotate_deg": 0.0},
            ),
            "joint2": ModuleObservation(
                module_id="joint2",
                module_type="joint",
                dofs={"crawl_mm": 295.0, "bend_deg": 0.0, "rotate_deg": 0.0},
            ),
        },
        pair_observations={
            "joint2->joint1": PairObservation(
                active_module="joint2",
                passive_module="joint1",
                relation_type="joint_joint",
                distance_mm=0.5,
                orientation_error_deg=45.0,
                coupled=coupled,
                observation_valid=True,
            )
        },
        topology_hint={"ordered_modules": ["tip", "joint1", "joint2"]},
    )


def _joint_joint_spec() -> SkillSpec:
    return SkillSpec(
        skill_key="local_transfer",
        active_module="joint2",
        passive_module="joint1",
        relation_type="joint_joint",
        distance_done_mm=2.0,
        orientation_done_deg=2.0,
        limits={"crawl_mm_s": 4.0, "rotate_deg_s": 6.0, "bend_deg_s": 5.0},
    )


class JointJointCouplingRuleStillOrientationSensitiveFakeTest(unittest.TestCase):
    """Verify joint-joint capture still requires orientation closure."""

    def test_relation_estimator_keeps_joint_joint_orientation_sensitive(self) -> None:
        frame = _joint_joint_frame(coupled=None)
        module_states = ModuleStateEstimator().estimate(frame, ordered_modules=("tip", "joint1", "joint2"))
        snapshot = compute_chain_snapshot(module_states, ordered_modules=("tip", "joint1", "joint2"))
        relation_states = RelationEstimator().estimate(
            frame,
            module_states=module_states,
            chain_snapshot=snapshot,
            ordered_modules=("tip", "joint1", "joint2"),
        )

        relation_state = relation_states["joint2->joint1"]
        self.assertFalse(relation_state.coupled)
        self.assertTrue(relation_state.diagnostics["orientation_required_for_coupling"])

    def test_sim_backend_refuses_joint_joint_capture_when_orientation_is_large(self) -> None:
        backend = SimRuntimeBackend(
            SimState(
                joint1_joint2_distance_mm=0.5,
                joint1_joint2_orientation_error_deg=45.0,
                joint1_joint2_coupled=False,
            )
        )
        command = SimCommand()
        envelope = SchedulerDispatchEnvelope(
            scheduler_state=SchedulerState(graph_id="joint_joint_capture"),
            scheduled_command=ScheduledSkillCommand(
                graph_id="joint_joint_capture",
                node_id="local_transfer_joint2_to_joint1",
                skill_key="local_transfer",
                active_module="joint2",
                passive_module="joint1",
                relation_type="joint_joint",
            ),
        )

        backend.apply_command(command, envelope=envelope)

        self.assertFalse(backend.snapshot_state().joint1_joint2_coupled)

    def test_fine_dock_controller_does_not_complete_joint_joint_capture_without_orientation(self) -> None:
        controller = FineDockController()
        relation_state = RelationState(
            active_module="joint2",
            passive_module="joint1",
            relation_type="joint_joint",
            distance_mm=0.5,
            orientation_error_deg=45.0,
            coupled=False,
            observation_valid=True,
        )

        result = controller.control(relation_state, _joint_joint_spec())

        self.assertEqual(result.status, "active")
        self.assertEqual(result.diagnostics["coupling_rule"], "distance_and_orientation")
        self.assertTrue(result.diagnostics["orientation_required_for_coupling"])


if __name__ == "__main__":
    unittest.main()

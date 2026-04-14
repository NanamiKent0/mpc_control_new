"""Fake tests for the special `joint1-tip` capture rule."""

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


def _tip_joint_frame(*, coupled: bool | None) -> RuntimeObservationFrame:
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
            "joint1->tip": PairObservation(
                active_module="joint1",
                passive_module="tip",
                relation_type="tip_joint",
                distance_mm=0.5,
                orientation_error_deg=45.0,
                coupled=coupled,
                observation_valid=True,
            )
        },
        topology_hint={"ordered_modules": ["tip", "joint1", "joint2"]},
    )


def _tip_joint_spec() -> SkillSpec:
    return SkillSpec(
        skill_key="front_cooperate",
        active_module="joint1",
        passive_module="tip",
        relation_type="tip_joint",
        distance_done_mm=2.0,
        orientation_done_deg=2.0,
        limits={"crawl_mm_s": 4.0, "rotate_deg_s": 6.0, "bend_deg_s": 5.0},
        params={"required_axes": ["distance"]},
    )


class TipJointSpecialCouplingRuleFakeTest(unittest.TestCase):
    """Verify `joint1-tip` can capture without orientation alignment."""

    def test_relation_estimator_keeps_orientation_observation_but_allows_distance_only_capture(self) -> None:
        frame = _tip_joint_frame(coupled=None)
        module_states = ModuleStateEstimator().estimate(frame, ordered_modules=("tip", "joint1", "joint2"))
        snapshot = compute_chain_snapshot(module_states, ordered_modules=("tip", "joint1", "joint2"))
        relation_states = RelationEstimator().estimate(
            frame,
            module_states=module_states,
            chain_snapshot=snapshot,
            ordered_modules=("tip", "joint1", "joint2"),
        )

        relation_state = relation_states["joint1->tip"]
        self.assertTrue(relation_state.coupled)
        self.assertEqual(relation_state.orientation_error_deg, 45.0)
        self.assertFalse(relation_state.diagnostics["orientation_required_for_coupling"])

    def test_sim_backend_sets_tip_joint_capture_true_even_when_orientation_is_large(self) -> None:
        backend = SimRuntimeBackend(
            SimState(
                tip_joint1_distance_mm=0.5,
                tip_joint1_orientation_error_deg=45.0,
                tip_joint1_coupled=False,
            )
        )
        command = SimCommand()
        envelope = SchedulerDispatchEnvelope(
            scheduler_state=SchedulerState(graph_id="tip_joint_capture"),
            scheduled_command=ScheduledSkillCommand(
                graph_id="tip_joint_capture",
                node_id="front_cooperate_joint1_to_tip",
                skill_key="front_cooperate",
                active_module="joint1",
                passive_module="tip",
                relation_type="tip_joint",
            ),
        )

        backend.apply_command(command, envelope=envelope)

        self.assertTrue(backend.snapshot_state().tip_joint1_coupled)
        self.assertEqual(backend.snapshot_state().tip_joint1_orientation_error_deg, 45.0)

    def test_fine_dock_controller_treats_tip_joint_capture_as_distance_only(self) -> None:
        controller = FineDockController()
        relation_state = RelationState(
            active_module="joint1",
            passive_module="tip",
            relation_type="tip_joint",
            distance_mm=0.5,
            orientation_error_deg=45.0,
            coupled=False,
            observation_valid=True,
        )

        result = controller.control(relation_state, _tip_joint_spec())

        self.assertEqual(result.status, "done")
        self.assertEqual(result.diagnostics["coupling_rule"], "joint1_tip_capture_only")
        self.assertFalse(result.diagnostics["orientation_required_for_coupling"])


if __name__ == "__main__":
    unittest.main()

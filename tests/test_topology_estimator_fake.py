"""Fake tests for topology estimation."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mpc_control_new.control_core.estimation import TopologyEstimator
from mpc_control_new.control_core.topology.relation_state import RelationState
from mpc_control_new.runtime_integration.observation_types import (
    ModuleObservation,
    PairObservation,
    RuntimeObservationFrame,
)


class TopologyEstimatorFakeTest(unittest.TestCase):
    """Verify topology estimation preserves order, coupling, and frontier hints."""

    def test_topology_estimate_contains_order_edges_and_frontier(self) -> None:
        frame = RuntimeObservationFrame(
            timestamp_ns=1,
            module_observations={
                "tip": ModuleObservation(module_id="tip", module_type="tip", dofs={"growth_mm": 150.0}),
                "joint1": ModuleObservation(module_id="joint1", module_type="joint", dofs={"bend_deg": 0.0}),
                "joint2": ModuleObservation(module_id="joint2", module_type="joint", dofs={"bend_deg": 0.0}),
            },
            pair_observations={
                "joint2->joint1": PairObservation(
                    active_module="joint2",
                    passive_module="joint1",
                    relation_type="joint_joint",
                    coupled=True,
                    observation_valid=True,
                )
            },
            topology_hint={
                "ordered_modules": ["tip", "joint1", "joint2"],
                "active_frontier": ["joint2", "joint1"],
                "blocked_edges": [["tip", "joint1"]],
            },
        )
        relation_states = {
            "joint2->joint1": RelationState(
                active_module="joint2",
                passive_module="joint1",
                relation_type="joint_joint",
                distance_mm=0.0,
                orientation_error_deg=0.0,
                coupled=True,
                observation_valid=True,
                diagnostics={"runtime_observation_valid": True},
            )
        }

        topology_estimate = TopologyEstimator().estimate(
            frame,
            relation_states=relation_states,
            ordered_modules=("tip", "joint1", "joint2"),
        )

        self.assertEqual(topology_estimate.ordered_modules, ("tip", "joint1", "joint2"))
        self.assertEqual(topology_estimate.coupled_edges, (("joint1", "joint2"),))
        self.assertEqual(topology_estimate.blocked_edges, (("joint1", "tip"),))
        self.assertEqual(topology_estimate.active_frontier, ("joint2", "joint1"))
        self.assertTrue(topology_estimate.topology_valid)


if __name__ == "__main__":
    unittest.main()

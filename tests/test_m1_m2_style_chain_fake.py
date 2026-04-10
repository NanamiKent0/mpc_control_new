"""Fake integration test for a joint1-to-tip coarse-then-dock chain."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mpc_control_new.control_core.orchestration.graph_factories import build_pair_approach_then_dock_graph
from mpc_control_new.control_core.orchestration.skill_scheduler import SkillScheduler


def _estimate(
    *,
    distance_mm: float,
    orientation_error_deg: float,
    coupled: bool = False,
) -> SimpleNamespace:
    """Build a fake legacy-like estimate for the joint1-tip chain."""
    geometry = SimpleNamespace(
        tip_joint1_distance_mm=distance_mm,
        tip_joint1_orientation_error_deg=abs(orientation_error_deg),
        tip_joint1_orientation_error_signed_deg=orientation_error_deg,
        tip_joint1_coupled=coupled,
    )
    control = SimpleNamespace(
        c1=0.0,
        psi1=0.0,
        theta1=0.0,
        d_t1=coupled,
    )
    return SimpleNamespace(geometry=geometry, control=control)


class M1M2StyleChainFakeTest(unittest.TestCase):
    """Verify the reusable scheduler chain for a joint1-tip relation."""

    def test_joint1_tip_chain_advances_from_coarse_to_dock_and_finishes(self) -> None:
        """The scheduler should hold, advance, then finish for joint1-tip."""
        scheduler = SkillScheduler()
        scheduler.load_graph(
            build_pair_approach_then_dock_graph(
                graph_id="m1_m2_style",
                active_module="joint1",
                passive_module="tip",
                relation_type="tip_joint",
                coarse_distance_threshold_mm=12.0,
                dock_distance_done_mm=2.0,
                dock_orientation_done_deg=5.0,
            )
        )

        first = scheduler.step(_estimate(distance_mm=30.0, orientation_error_deg=-10.0, coupled=False))
        self.assertEqual(first.node.node_id, "coarse_approach")
        self.assertFalse(first.transitioned)
        self.assertEqual(first.scheduler_state.current_node_id, "coarse_approach")

        second = scheduler.step(_estimate(distance_mm=10.0, orientation_error_deg=-9.0, coupled=False))
        self.assertTrue(second.transitioned)
        self.assertEqual(second.scheduler_state.current_node_id, "fine_dock")
        self.assertEqual(second.scheduler_state.completed_node_ids, ["coarse_approach"])

        third = scheduler.step(_estimate(distance_mm=1.0, orientation_error_deg=2.0, coupled=False))
        self.assertTrue(third.scheduler_state.is_finished)
        self.assertEqual(third.scheduler_state.completed_node_ids, ["coarse_approach", "fine_dock"])
        self.assertEqual(third.transition_reason, "graph_finished:fine_dock")


if __name__ == "__main__":
    unittest.main()

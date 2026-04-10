"""Fake integration test for a joint2-to-joint1 coarse-then-dock chain."""

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
    """Build a fake legacy-like estimate for the joint2-joint1 chain."""
    geometry = SimpleNamespace(
        joint1_joint2_distance_mm=distance_mm,
        joint1_joint2_orientation_error_deg=abs(orientation_error_deg),
        joint1_joint2_orientation_error_signed_deg=orientation_error_deg,
        joint1_joint2_coupled=coupled,
    )
    control = SimpleNamespace(
        c1=0.0,
        psi1=0.0,
        theta1=0.0,
        c2=0.0,
        psi2=0.0,
        theta2=0.0,
        d_12=coupled,
    )
    return SimpleNamespace(geometry=geometry, control=control)


class M5M6StyleChainFakeTest(unittest.TestCase):
    """Verify the reusable scheduler chain for a joint2-joint1 relation."""

    def test_joint2_joint1_chain_reuses_same_template(self) -> None:
        """The same graph template should drive a joint-joint approach+dock flow."""
        scheduler = SkillScheduler()
        scheduler.load_graph(
            build_pair_approach_then_dock_graph(
                graph_id="m5_m6_style",
                active_module="joint2",
                passive_module="joint1",
                relation_type="joint_joint",
                coarse_distance_threshold_mm=15.0,
                dock_distance_done_mm=2.0,
                dock_orientation_done_deg=4.0,
            )
        )

        first = scheduler.step(_estimate(distance_mm=26.0, orientation_error_deg=8.0, coupled=False))
        self.assertEqual(first.node.node_id, "coarse_approach")
        self.assertFalse(first.transitioned)
        self.assertEqual(first.scheduler_state.current_node_id, "coarse_approach")

        second = scheduler.step(_estimate(distance_mm=14.0, orientation_error_deg=7.0, coupled=False))
        self.assertTrue(second.transitioned)
        self.assertEqual(second.scheduler_state.current_node_id, "fine_dock")
        self.assertEqual(second.diagnostics["pair"], "joint2->joint1")

        third = scheduler.step(_estimate(distance_mm=1.0, orientation_error_deg=1.0, coupled=False))
        self.assertTrue(third.scheduler_state.is_finished)
        self.assertEqual(third.scheduler_state.completed_node_ids, ["coarse_approach", "fine_dock"])


if __name__ == "__main__":
    unittest.main()

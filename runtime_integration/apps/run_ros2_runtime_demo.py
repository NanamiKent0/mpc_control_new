"""Internal ROS2 runtime demo/diagnostic entry."""

from __future__ import annotations

if __package__ in {None, ""}:
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

import argparse
import json

from ...runtime_integration.ros2_runtime_node import Ros2RuntimeNode
from ...runtime_integration.runtime_session import build_ros2_runtime_session


class _LoopbackPublisher:
    """Fake publisher used by the ROS2 demo shim."""

    def __init__(self) -> None:
        self.payloads: list[object] = []

    def publish(self, payload: object) -> None:
        """Record one published payload."""
        self.payloads.append(payload)


class _LoopbackNodeHandle:
    """Small ROS2-like node handle for demo and test wiring."""

    def __init__(self) -> None:
        self.publisher = _LoopbackPublisher()
        self.subscriptions: list[object] = []

    def create_subscription(self, msg_type: object, topic: str, callback: object, qos: int) -> object:
        """Record one subscription callback."""
        del msg_type, topic, qos
        self.subscriptions.append(callback)
        return callback

    def create_publisher(self, msg_type: object, topic: str, qos: int) -> _LoopbackPublisher:
        """Return the demo publisher."""
        del msg_type, topic, qos
        return self.publisher

    def destroy_node(self) -> None:
        """Compatibility no-op."""
        return None


def run_ros2_runtime_demo(
    *,
    enable_rclpy: bool = False,
    use_shim: bool = False,
) -> dict[str, object]:
    """Run the internal ROS2 runtime demo and return a compact summary."""
    runtime_node = Ros2RuntimeNode(node_handle=_LoopbackNodeHandle()) if use_shim else None
    session = build_ros2_runtime_session(
        runtime_node=runtime_node,
        enable_rclpy=enable_rclpy,
    )
    if runtime_node is not None:
        session.start()
        runtime_node.push_observation(
            {
                "timestamp_ns": 1_000,
                "modules": [
                    {"module_id": "joint1", "module_type": "joint", "dofs": {"crawl_mm": 0.0}},
                    {"module_id": "tip", "module_type": "tip", "dofs": {"growth_mm": 0.0}},
                    {"module_id": "joint2", "module_type": "joint", "dofs": {"crawl_mm": 0.0}},
                ],
                "pairs": [
                    {
                        "active_module": "joint1",
                        "passive_module": "tip",
                        "relation_type": "tip_joint",
                        "distance_mm": 18.0,
                        "orientation_error_deg": -5.0,
                        "coupled": False,
                        "observation_valid": True,
                    }
                ],
            }
        )
    result = session.step()
    session.stop()
    return {
        "accepted": result.accepted,
        "reason": result.reason,
        "provider_kind": result.provider_kind,
        "dispatcher_kind": result.dispatcher_kind,
        "ros2_available": result.provider_status.get("ros2_available"),
        "dispatch_target": result.dispatch_target,
        "graph_id": result.graph_id,
        "current_node_id": result.current_node_id,
        "active_skill_key": result.active_skill_key,
    }


def main(argv: list[str] | None = None) -> int:
    """CLI entry point for the internal ROS2 runtime demo."""
    parser = argparse.ArgumentParser(description="Run the internal minimal ROS2 runtime demo.")
    parser.add_argument("--enable-rclpy", action="store_true")
    parser.add_argument("--use-shim", action="store_true")
    args = parser.parse_args(argv)
    print(
        json.dumps(
            run_ros2_runtime_demo(enable_rclpy=args.enable_rclpy, use_shim=args.use_shim),
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

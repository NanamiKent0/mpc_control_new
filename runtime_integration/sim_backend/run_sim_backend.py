"""Standalone entry point for the refactored simulator backend."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import sys
import time

if __package__ in {None, ""}:
    project_parent = Path(__file__).resolve().parents[3]
    if str(project_parent) not in sys.path:
        sys.path.insert(0, str(project_parent))

from mpc_control_new.runtime_integration.sim_backend.ros2_backend import (
    SimBackendRunner,
    SimRos2RuntimeDependencies,
)
from mpc_control_new.runtime_integration.sim_backend.types import SIM_MODULE_IDS, SimConfig


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run the standalone simulator backend for gui_ros2.py."
    )
    parser.add_argument("--dt", type=float, default=0.05, help="Simulation step size in seconds.")
    parser.add_argument(
        "--state-topic",
        type=str,
        default="sim/state",
        help="ROS2 topic used by the standalone visualizer.",
    )
    parser.add_argument(
        "--check-runtime",
        action="store_true",
        help="Only validate ROS2 Python availability and exit.",
    )
    return parser


def _runtime_ready() -> tuple[bool, str]:
    runtime = SimRos2RuntimeDependencies()
    if runtime.rclpy_module is None or runtime.node_factory is None:
        return False, "rclpy or rclpy.node.Node is unavailable"
    if runtime.int32_message_cls is None or runtime.int64_message_cls is None or runtime.string_message_cls is None:
        return False, "std_msgs Python message classes are unavailable"
    return True, "ROS2 Python runtime is available"


def main(argv: list[str] | None = None) -> int:
    args = build_arg_parser().parse_args(argv)
    ready, message = _runtime_ready()
    if args.check_runtime:
        print(message)
        return 0 if ready else 1
    if not ready:
        print(f"sim backend cannot start: {message}")
        print("please source your ROS2 environment first, then rerun this command")
        return 1

    os.environ.setdefault("ROS_LOG_DIR", "/tmp/mpc_control_new_ros_logs")
    os.environ.setdefault("ROS_HOME", "/tmp/mpc_control_new_ros_home")

    config = SimConfig(
        dt=float(args.dt),
        state_topic=str(args.state_topic),
    )
    runner = SimBackendRunner(config=config)
    try:
        runner.start()
    except Exception as exc:
        print(f"sim backend failed to start: {exc}")
        return 1

    print("sim backend started")
    namespace_text = "|".join(SIM_MODULE_IDS)
    print(f"command topics: {namespace_text} -> <namespace>/motor_command")
    print(f"feedback topics: {namespace_text} -> <namespace>/motor_feedback")
    print(f"state topic: {config.state_topic}")
    print("press Ctrl+C to stop")
    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        runner.stop()
        print("sim backend stopped")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

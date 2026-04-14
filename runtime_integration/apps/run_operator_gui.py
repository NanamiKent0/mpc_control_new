from __future__ import annotations

if __package__ in {None, ""}:
    import sys
    from pathlib import Path

    # 让直接执行 python3 run_operator_gui.py 也能找到 mpc_control_new 包
    sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

    from mpc_control_new.runtime_integration.operator_gui import OperatorGUIApp
    from mpc_control_new.runtime_integration.runtime_session import (
        build_ros2_runtime_session,
        build_sim_runtime_session,
    )
else:
    from ..operator_gui import OperatorGUIApp
    from ..runtime_session import build_ros2_runtime_session, build_sim_runtime_session

import argparse


def _build_session(backend: str, *, dt: float, enable_rclpy: bool):
    if backend == "ros2":
        return build_ros2_runtime_session(enable_rclpy=enable_rclpy)
    return build_sim_runtime_session(dt=dt)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Launch the high-level operator GUI for TIP_FREE_GROWTH / TIP_TURN control."
    )
    parser.add_argument(
        "--backend",
        choices=("sim", "ros2"),
        default="sim",
        help="Select runtime backend. Default: sim",
    )
    parser.add_argument(
        "--dt",
        type=float,
        default=0.1,
        help="Sim step dt in seconds (used only when backend=sim).",
    )
    parser.add_argument(
        "--enable-rclpy",
        action="store_true",
        help="Enable real rclpy startup when backend=ros2.",
    )
    args = parser.parse_args(argv)

    session = _build_session(
        args.backend,
        dt=args.dt,
        enable_rclpy=args.enable_rclpy,
    )

    app = OperatorGUIApp(runtime_session=session, refresh_ms=150)
    app.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
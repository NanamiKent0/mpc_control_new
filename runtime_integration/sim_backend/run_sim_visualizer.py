"""Standalone entry point for the refactored simulator visualizer."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import sys

if __package__ in {None, ""}:
    project_parent = Path(__file__).resolve().parents[3]
    if str(project_parent) not in sys.path:
        sys.path.insert(0, str(project_parent))


def _configure_matplotlib_backend(force_headless: bool) -> bool:
    import matplotlib

    if force_headless or not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        matplotlib.use("Agg")
        return True
    return False


from mpc_control_new.runtime_integration.sim_backend.ros2_backend import SimRos2RuntimeDependencies


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run the standalone simulator visualizer for gui_ros2.py."
    )
    parser.add_argument("--topic", type=str, default="sim/state", help="State topic to subscribe.")
    parser.add_argument("--fps", type=float, default=30.0, help="Target refresh rate.")
    parser.add_argument(
        "--view-mode",
        type=str,
        choices=("fixed", "follow_with_hysteresis", "fit_once"),
        default="follow_with_hysteresis",
        help="Viewport strategy.",
    )
    parser.add_argument("--follow-margin-ratio", type=float, default=0.12)
    parser.add_argument("--follow-cooldown", type=float, default=0.5)
    parser.add_argument("--frame-axis-length", type=float, default=40.0)
    parser.add_argument("--xlim", nargs=2, type=float, default=(-650.0, 250.0))
    parser.add_argument("--ylim", nargs=2, type=float, default=(-300.0, 300.0))
    parser.add_argument("--zlim", nargs=2, type=float, default=(-300.0, 300.0))
    parser.add_argument("--hide-frames", action="store_true")
    parser.add_argument("--headless", action="store_true", help="Run without opening a window.")
    parser.add_argument("--headless-duration", type=float, default=3.0)
    parser.add_argument("--save-path", type=str, default=None)
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
    if runtime.string_message_cls is None:
        return False, "std_msgs.msg.String is unavailable"
    return True, "ROS2 Python runtime is available"


def main(argv: list[str] | None = None) -> int:
    args = build_arg_parser().parse_args(argv)
    ready, message = _runtime_ready()
    if args.check_runtime:
        print(message)
        return 0 if ready else 1
    if not ready:
        print(f"sim visualizer cannot start: {message}")
        print("please source your ROS2 environment first, then rerun this command")
        return 1

    os.environ.setdefault("MPLCONFIGDIR", "/tmp/mpc_control_new_mplconfig")
    running_headless = _configure_matplotlib_backend(force_headless=bool(args.headless))
    from mpc_control_new.runtime_integration.sim_backend.visualizer import (
        SimVisualizer,
        VisualizerConfig,
    )

    visualizer = SimVisualizer(
        topic_name=str(args.topic),
        config=VisualizerConfig(
            fps=float(args.fps),
            view_mode=str(args.view_mode),
            follow_margin_ratio=float(args.follow_margin_ratio),
            follow_cooldown_s=float(args.follow_cooldown),
            frame_axis_length_mm=float(args.frame_axis_length),
            xlim=(float(args.xlim[0]), float(args.xlim[1])),
            ylim=(float(args.ylim[0]), float(args.ylim[1])),
            zlim=(float(args.zlim[0]), float(args.zlim[1])),
            show_frames=not bool(args.hide_frames),
        ),
    )
    if running_headless:
        visualizer.run_headless(
            duration_s=float(args.headless_duration),
            save_path=args.save_path,
        )
        print("sim visualizer headless run completed")
        return 0

    print("opening sim visualizer window")
    visualizer.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

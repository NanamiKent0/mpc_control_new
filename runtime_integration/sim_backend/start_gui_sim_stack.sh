#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
cd "${PROJECT_ROOT}"

for ros_setup in \
  "${ROS_SETUP_BASH:-}" \
  /opt/ros/jazzy/setup.bash \
  /opt/ros/humble/setup.bash \
  /opt/ros/iron/setup.bash
do
  if [[ -n "${ros_setup}" && -f "${ros_setup}" ]]; then
    # shellcheck disable=SC1090
    set +u
    source "${ros_setup}"
    set -u
    break
  fi
done

export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/mpc_control_new_ros_logs}"
export ROS_HOME="${ROS_HOME:-/tmp/mpc_control_new_ros_home}"
export MPLCONFIGDIR="${MPLCONFIGDIR:-/tmp/mpc_control_new_mplconfig}"

if ! python3 "${PROJECT_ROOT}/runtime_integration/sim_backend/run_sim_backend.py" --check-runtime >/dev/null 2>&1; then
  echo "sim backend 未启动: 当前环境缺少 ROS2 Python 运行时"
  echo "请先 source 对应 ROS2 setup.bash，再重新运行本脚本"
  exit 1
fi

if ! python3 "${PROJECT_ROOT}/runtime_integration/sim_backend/run_sim_visualizer.py" --check-runtime >/dev/null 2>&1; then
  echo "sim visualizer 未启动: 当前环境缺少 ROS2 Python 运行时"
  echo "请先 source 对应 ROS2 setup.bash，再重新运行本脚本"
  exit 1
fi

BACKEND_PID=""
VISUALIZER_PID=""
_cleaned_up=0

cleanup() {
  if [[ "${_cleaned_up}" -eq 1 ]]; then
    return
  fi
  _cleaned_up=1
  echo ""
  echo "stopping sim stack ..."
  for pid in "${VISUALIZER_PID}" "${BACKEND_PID}"; do
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      kill "${pid}" 2>/dev/null || true
    fi
  done
  for pid in "${VISUALIZER_PID}" "${BACKEND_PID}"; do
    if [[ -n "${pid}" ]]; then
      wait "${pid}" 2>/dev/null || true
    fi
  done
  echo "sim stack stopped"
}

trap cleanup EXIT INT TERM

python3 "${PROJECT_ROOT}/runtime_integration/sim_backend/run_sim_backend.py" &
BACKEND_PID=$!
echo "sim backend 已启动 (pid=${BACKEND_PID})"
sleep 1

visualizer_headless=0
if [[ "${SIM_VISUALIZER_HEADLESS:-0}" == "1" ]]; then
  visualizer_headless=1
elif [[ -n "${WAYLAND_DISPLAY:-}" ]]; then
  visualizer_headless=0
elif [[ -n "${DISPLAY:-}" ]]; then
  if command -v xdpyinfo >/dev/null 2>&1 && ! xdpyinfo >/dev/null 2>&1; then
    visualizer_headless=1
  fi
else
  visualizer_headless=1
fi

if [[ "${visualizer_headless}" -eq 0 ]]; then
  python3 "${PROJECT_ROOT}/runtime_integration/sim_backend/run_sim_visualizer.py" &
else
  echo "未检测到可用显示环境，sim visualizer 将以 headless 模式运行"
  python3 "${PROJECT_ROOT}/runtime_integration/sim_backend/run_sim_visualizer.py" --headless --headless-duration 3600 &
fi
VISUALIZER_PID=$!
echo "sim visualizer 已启动 (pid=${VISUALIZER_PID})"
echo ""
echo "下一步请在另一个终端运行："
echo "python3 ${PROJECT_ROOT}/runtime_integration/gui/gui_ros2.py"
echo ""
echo "当前脚本保持运行以托管 sim backend + sim visualizer"
echo "按 Ctrl+C 可停止整个 sim 栈"

wait -n "${BACKEND_PID}" "${VISUALIZER_PID}"

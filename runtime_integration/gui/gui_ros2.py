"""Operator GUI entry kept from the user workflow and made self-contained for the new runtime."""

from __future__ import annotations

import os
import math
import sys
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
import traceback

try:
    import pygame
except Exception:  # pragma: no cover - dependency guard
    pygame = None

from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                             QLabel, QPushButton, QLineEdit, QSpinBox, QDoubleSpinBox, 
                             QTextEdit, QGroupBox, QTabWidget, QComboBox, QScrollArea, QGridLayout, QSizePolicy)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject, QThread
from PyQt5.QtGui import QColor, QTextCursor

try:
    import rclpy
    from rclpy.node import Node
except Exception:  # pragma: no cover - dependency guard
    rclpy = None
    Node = None

try:
    from std_msgs.msg import Int32MultiArray, Int64MultiArray
except Exception:  # pragma: no cover - dependency guard
    Int32MultiArray = None
    Int64MultiArray = None

if __package__ in {None, ""}:
    project_root = Path(__file__).resolve().parents[3]
    if str(project_root) not in sys.path:
        sys.path.insert(0, str(project_root))

try:
    from ..common.encoder_protocol import (
        MOTOR_CONVERSION_PARAMS,
        STATUS_STALE_SOURCE,
        counts_to_physical,
        motor_ids_for_namespace_or_joint_name,
        parse_feedback_array,
        physical_to_counts,
    )
    from .backend_manager import (
        BACKEND_SOURCE_LIVE,
        BACKEND_SOURCE_SIM,
        BACKEND_SOURCES,
        EmbeddedSimBackend,
        GuiBackendManager,
        Ros2TopicBackend,
    )
except ImportError:  # pragma: no cover - direct-script fallback
    from mpc_control_new.runtime_integration.common.encoder_protocol import (
        MOTOR_CONVERSION_PARAMS,
        STATUS_STALE_SOURCE,
        counts_to_physical,
        motor_ids_for_namespace_or_joint_name,
        parse_feedback_array,
        physical_to_counts,
    )
    from mpc_control_new.runtime_integration.gui.backend_manager import (
        BACKEND_SOURCE_LIVE,
        BACKEND_SOURCE_SIM,
        BACKEND_SOURCES,
        EmbeddedSimBackend,
        GuiBackendManager,
        Ros2TopicBackend,
    )

# ===== 配置参数 =====

EXTENDED_FEEDBACK_LOG_INTERVAL_SEC = 2.0
STALE_WARNING_LOG_INTERVAL_SEC = 2.0
GUI_ROS_SPIN_INTERVAL_MS = 10
GUI_ROS_SPIN_BURST_PER_NODE = 8


@dataclass(slots=True)
class Ros2GuiRuntimeDependencies:
    """Injectable ROS2 runtime hooks used to run the GUI against real ROS2 or a shim."""

    rclpy_module: object | None = rclpy
    node_factory: object | None = Node
    int32_message_cls: object | None = Int32MultiArray
    int64_message_cls: object | None = Int64MultiArray

    def ok(self) -> bool:
        module = self.rclpy_module
        if module is None:
            return False
        ok = getattr(module, "ok", None)
        if callable(ok):
            try:
                return bool(ok())
            except Exception:
                return False
        return False

    def init(self) -> None:
        module = self.rclpy_module
        if module is None:
            raise RuntimeError("rclpy_unavailable")
        init = getattr(module, "init", None)
        if callable(init):
            init(args=None)

    def shutdown(self) -> None:
        module = self.rclpy_module
        if module is None:
            return
        shutdown = getattr(module, "shutdown", None)
        if callable(shutdown):
            try:
                shutdown()
            except Exception:
                return

    def spin_once(self, node: object, timeout_sec: float) -> None:
        module = self.rclpy_module
        if module is None:
            return
        spin_once = getattr(module, "spin_once", None)
        if callable(spin_once):
            spin_once(node, timeout_sec=timeout_sec)

    def create_node(self, node_name: str) -> object:
        if self.node_factory is None:
            raise RuntimeError("ros2_node_factory_unavailable")
        return self.node_factory(node_name)

    def make_command_message(self) -> object:
        if self.int32_message_cls is None:
            return _ArrayMessage()
        return self.int32_message_cls()

    def make_feedback_message(self) -> object:
        if self.int64_message_cls is None:
            return _ArrayMessage()
        return self.int64_message_cls()


class _ArrayMessage:
    """Tiny fallback message object used when std_msgs is unavailable."""

    def __init__(self) -> None:
        self.data = []


def _env_flag(name: str, *, default: bool = False) -> bool:
    """Read a boolean environment flag."""
    raw_value = os.environ.get(name)
    if raw_value is None:
        return default
    return raw_value.strip().lower() in {"1", "true", "yes", "on"}


def _prepare_qt_environment(*, headless: bool) -> None:
    """Prepare a deterministic Qt environment for headless launches."""
    if headless or not os.environ.get("DISPLAY"):
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")


def build_gui_backend_manager(
    *,
    ros_runtime: Ros2GuiRuntimeDependencies | None = None,
    enable_live_backend: bool = True,
    enable_embedded_sim: bool | None = None,
    sim_auto_step_ms: int = 50,
) -> GuiBackendManager:
    """Build the shared backend manager used by the canonical GUI entry."""
    if enable_embedded_sim is None:
        enable_embedded_sim = _env_flag("MPC_CONTROL_NEW_GUI_ENABLE_SIM", default=False)
    live_backend = Ros2TopicBackend(
        source=BACKEND_SOURCE_LIVE,
        ros_runtime=ros_runtime or Ros2GuiRuntimeDependencies(),
        enabled=enable_live_backend,
        node_name_prefix="gui_live_backend",
        assume_online_if_started=False,
    )
    sim_backend = EmbeddedSimBackend(
        enabled=bool(enable_embedded_sim),
        auto_step_ms=sim_auto_step_ms,
    )
    return GuiBackendManager(
        sim_backend=sim_backend,
        live_backend=live_backend,
    )

# 关节对配置 (保持不变)
JOINT_PAIRS = {
    "大臂关节对": {
        "joints": ["关节1", "关节2"],
        "motors": {
            "关节1": [1, 3],
            "关节2": [1, 3]
        }
    },
    "小臂关节对": {
        "joints": ["关节3", "关节4"],
        "motors": {
            "关节3": [1, 3],
            "关节4": [1, 3]
        }
    }
}

# 关节配置 (修改为 ROS2 Topic 配置)
# 在ROS2中，区分不同机器人关节通常使用Namespace
# 请确保esp32 micro-ros固件或agent映射到了对应的Namespace
# 例如：关节1的esp32应该订阅‘joint1/motor_command’并发布'joint1/motor_feedback'
JOINT_CONFIGS = [
    {"name": "尖端", "topic_ns": "tip"} ,
    {"name": "关节1", "topic_ns": "joint1"} ,
    {"name": "关节2", "topic_ns": "joint2"} ,
    {"name": "关节3", "topic_ns": "joint3"} ,
    {"name": "关节4", "topic_ns": "joint4"} ,
    {"name": "关节5", "topic_ns": "joint5"} ,
]

# ===== ROS2 工作线程 =====
class ROS2Worker(QThread):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self._running = True

    def run(self):
        if rclpy is None:
            return
        while self._running and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.01)
    
    def stop(self):
        self._running = False
        self.wait()

# ===== ROS2 控制系统类 =====
class ROS2ControlSystem(QObject):
    """
    负责与 ESP32 Micro-ROS Agent 通信的类
    """
    # 信号定义
    log_signal = pyqtSignal(str, str)  # message, level
    connected_signal = pyqtSignal()
    disconnected_signal = pyqtSignal()
    feedback_signal = pyqtSignal(int, int) # motor_id, raw_pulses

    def __init__(self, topic_ns="", *, ros_runtime: Ros2GuiRuntimeDependencies | None = None):
        super().__init__()
        self.topic_ns = topic_ns
        self.ros_runtime = ros_runtime or Ros2GuiRuntimeDependencies()
        self.node = None
        self.publisher = None
        self.subscription = None
        self.connected = False
        self.latest_feedback_by_motor = {}
        self._last_extended_feedback_log_ts = {}
        self._last_stale_warning_ts = {}

    def start_ros(self):
        """启动 ROS2 节点"""
        try:
            if not self.ros_runtime.ok():
                self.ros_runtime.init()
            
            node_name = f'gui_node_{int(time.time())}'
            self.node = self.ros_runtime.create_node(node_name)
            
            # 创建发布者 (Command)
            topic_cmd = 'motor_command'
            if self.topic_ns:
                topic_cmd = f'{self.topic_ns}/{topic_cmd}'
            
            self.publisher = self.node.create_publisher(
                object if self.ros_runtime.int32_message_cls is None else self.ros_runtime.int32_message_cls, 
                topic_cmd, 
                10
            )
            
            # 创建订阅者 (Feedback)
            topic_fb = 'motor_feedback'
            if self.topic_ns:
                topic_fb = f'{self.topic_ns}/{topic_fb}'
                
            self.subscription = self.node.create_subscription(
                object if self.ros_runtime.int64_message_cls is None else self.ros_runtime.int64_message_cls,
                topic_fb,
                self._feedback_callback,
                10
            )
            
            self.connected = True
            self.connected_signal.emit()
            self.log_signal.emit(f"ROS2 节点已启动: {node_name}", "info")
            
            return self.node
            
        except Exception as e:
            self.log_signal.emit(f"启动 ROS2 失败: {str(e)}", "error")
            return None

    def stop_ros(self):
        """停止 ROS2 节点"""
        if self.node:
            destroy = getattr(self.node, "destroy_node", None)
            if callable(destroy):
                destroy()
            self.node = None
        self.connected = False
        self.disconnected_signal.emit()
        self.log_signal.emit("ROS2 节点已停止", "info")

    def _feedback_callback(self, msg):
        """处理反馈消息"""
        try:
            parsed_feedback = parse_feedback_array(msg.data)
            self.latest_feedback_by_motor[parsed_feedback.motor_id] = parsed_feedback

            if parsed_feedback.is_extended:
                now_ts = time.time()
                motor_id = parsed_feedback.motor_id

                last_log_ts = self._last_extended_feedback_log_ts.get(motor_id, 0.0)
                if now_ts - last_log_ts >= EXTENDED_FEEDBACK_LOG_INTERVAL_SEC:
                    self._last_extended_feedback_log_ts[motor_id] = now_ts
                    self.log_signal.emit(
                        f"收到新版反馈: 电机{motor_id}, seq={parsed_feedback.seq}, "
                        f"flags=0x{parsed_feedback.status_flags:X}, source={parsed_feedback.source_id}",
                        "recv"
                    )

                if parsed_feedback.status_flags & STATUS_STALE_SOURCE:
                    last_warn_ts = self._last_stale_warning_ts.get(motor_id, 0.0)
                    if now_ts - last_warn_ts >= STALE_WARNING_LOG_INTERVAL_SEC:
                        self._last_stale_warning_ts[motor_id] = now_ts
                        self.log_signal.emit(
                            f"电机{motor_id} 反馈源陈旧(stale_source)，GUI继续显示最近一次反馈值",
                            "warning"
                        )

            self.feedback_signal.emit(parsed_feedback.motor_id, parsed_feedback.raw_pulses)
        except Exception as e:
            self.log_signal.emit(f"处理反馈消息错误: {str(e)}", "error")

    def send_compact_command(self, motor_id, cmd_type, param1=0, param2=0, param3=0):
        """发送紧凑命令"""
        if not self.connected or not self.publisher:
            return False
        
        try:
            
            msg = self.ros_runtime.make_command_message()
            # 格式: [motor_id, cmd_type, param1, param2, param3]
            msg.data = [int(motor_id), int(cmd_type), int(param1), int(param2), int(param3)]
            self.publisher.publish(msg)
            return True
            
        except Exception as e:
            self.log_signal.emit(f"发送命令失败: {str(e)}", "error")
            return False

    def set_motor_velocity(self, motor_id, velocity):
        """设置电机速度"""
        # 参数1: 速度
        # 参数2: 时间戳低16位 * 100 (用于同步，这里简化处理)
        # 参数3: 保留
        
        p1 = int(velocity)
        p2_x100 = int((time.time() % 10000) * 100) # 简单模拟时间戳
        
        return self.send_compact_command(motor_id, 0x02, p1, p2_x100, 0)

    def emergency_stop(self):
        """急停"""
        # 对所有可能的电机ID发送停止
        for i in range(1, 6):
            self.send_compact_command(i, 0x01)
        self.log_signal.emit("发送急停命令", "warning")

    def get_zero_target_motor_ids(self):
        return motor_ids_for_namespace_or_joint_name(namespace=self.topic_ns)

    def set_zero_position(self):
        """设置零位"""
        # tip 页面只对 motor4 发 0x03；普通关节继续对 motor1/2/3 发 0x03。
        success = True
        for motor_id in self.get_zero_target_motor_ids():
            if not self.send_compact_command(motor_id, 0x03):
                success = False
        return success


# ===== 关节控制组件 =====
class JointControlWidget(QWidget):
    """单个关节（或单个 ESP32 节点）的控制组件"""
    
    log_signal = pyqtSignal(str, str, str) # message, level, joint_name

    def __init__(self, joint_config, *, dispatch_manager: GuiBackendManager):
        super().__init__()
        self.joint_config = joint_config
        self.joint_name = joint_config["name"]
        self.topic_ns = joint_config.get("topic_ns", "")
        self.dispatch_manager = dispatch_manager
        
        # 状态变量
        self.feedback_state = {
            source: {
                "motor_positions": [0, 0, 0],
                "motor_offsets": [0, 0, 0],
                "motor_last_raw": [0, 0, 0],
                "motor4_offset": 0,
                "motor4_last_raw": 0,
                "position_status_raw_4": 0,
            }
            for source in BACKEND_SOURCES
        }
        self.is_setting_zero = False        # [新增] 正在设置零位标志
        
        self.data_lock = threading.Lock()

        # 初始化 backend 分发代理
        self.ros_system = self.dispatch_manager.create_namespace_proxy(self.topic_ns)
        self.ros_system.log_signal.connect(self.on_log_internal)
        self.ros_system.connected_signal.connect(self.on_connected)
        self.ros_system.disconnected_signal.connect(self.on_disconnected)
        self.ros_system.sourced_feedback_signal.connect(self.on_compact_feedback)
        self.dispatch_manager.status_signal.connect(self.on_backend_status)
        
        # 初始化UI
        self.init_ui()
        self.on_backend_status(self.dispatch_manager.status_summary())
        if self.ros_system.connected:
            self.on_connected()
        else:
            self.on_disconnected()

    def init_ui(self):
        layout = QVBoxLayout(self)
        
        # 1. backend 状态区域
        conn_group = QGroupBox(f"{self.joint_name} Backend 状态")
        conn_layout = QGridLayout()
        self.backend_status_labels = {}
        for row, source in enumerate(BACKEND_SOURCES):
            source_label = QLabel(f"{source.upper()}:")
            conn_layout.addWidget(source_label, row, 0)
            value_label = QLabel("offline")
            value_label.setStyleSheet("color: red; font-weight: bold;")
            conn_layout.addWidget(value_label, row, 1)
            self.backend_status_labels[source] = value_label
        conn_group.setLayout(conn_layout)
        layout.addWidget(conn_group)
        
        # 2. 状态显示区域（按来源拆分，避免 sim/live 互相覆盖）
        status_group = QGroupBox("状态监视")
        status_layout = QGridLayout()
        self.position_status = {source: {} for source in BACKEND_SOURCES}
        motor_ids = [4] if self.joint_name == "尖端" else [1, 2, 3]
        if self.joint_name == "尖端":
            motor_headers = {4: "尖端电机(电机4)"}
        else:
            motor_headers = {
                1: "电机1(直线)",
                2: "电机2(旋转)",
                3: "电机3(弯折)",
            }
        status_layout.addWidget(QLabel("来源"), 0, 0)
        for column, motor_id in enumerate(motor_ids, start=1):
            status_layout.addWidget(QLabel(motor_headers[motor_id]), 0, column)
        for row, source in enumerate(BACKEND_SOURCES, start=1):
            row_label = QLabel(source.upper())
            row_label.setStyleSheet("font-weight: bold;")
            status_layout.addWidget(row_label, row, 0)
            for column, motor_id in enumerate(motor_ids, start=1):
                value_label = QLabel(self._default_position_text(motor_id))
                value_label.setStyleSheet("color: gray; font-weight: bold; font-size: 16px;")
                status_layout.addWidget(value_label, row, column)
                self.position_status[source][motor_id] = value_label
        status_group.setLayout(status_layout)
        layout.addWidget(status_group)
        
        # 3. 速度控制区域
        vel_group = QGroupBox("速度控制")
        vel_layout = QVBoxLayout()
        
        self.velocity_inputs = []
        self.velocity_send_buttons = []
        self.velocity_stop_buttons = []
        
        # 创建3个电机的控制行 (虽然主要用1和3)
        motor_ids = [4] if self.joint_name == "尖端" else [1,2,3]
        for motor_id in motor_ids:
            h_layout = QHBoxLayout()
            
            h_layout.addWidget(QLabel(f"电机{motor_id}:"))
            
            # 输入框
            spin = QDoubleSpinBox()
            spin.setRange(-2000, 2000)
            spin.setValue(10.0)
            spin.setSingleStep(10)
            
            #根据配置设置单位后缀
            params = MOTOR_CONVERSION_PARAMS.get(motor_id)
            if params and 'unit_suffix' in params:
                spin.setSuffix(f"{params['unit_suffix']}/s")
            else:
                spin.setSuffix(" cps")

            self.velocity_inputs.append(spin)
            h_layout.addWidget(spin)
            
            # 发送按钮
            btn_send = QPushButton("设置速度")
            btn_send.clicked.connect(lambda checked, mid=motor_id: self.send_single_velocity_command(mid))
            btn_send.setEnabled(False)
            self.velocity_send_buttons.append(btn_send)
            h_layout.addWidget(btn_send)
            
            # 停止按钮
            btn_stop = QPushButton("停止")
            btn_stop.clicked.connect(lambda checked, mid=motor_id: self.stop_single_velocity_command(mid))
            btn_stop.setStyleSheet("background-color: #FF851B; color: white; font-weight: bold;")
            btn_stop.setEnabled(False)
            self.velocity_stop_buttons.append(btn_stop)
            h_layout.addWidget(btn_stop)
            
            vel_layout.addLayout(h_layout)
            
        vel_group.setLayout(vel_layout)
        layout.addWidget(vel_group)
        
        # 4. 功能按钮区域
        func_layout = QHBoxLayout()
        
        btn_zero = QPushButton("设置零位")
        btn_zero.clicked.connect(self.zero_position_clicked)
        func_layout.addWidget(btn_zero)

        magnet_layout = QHBoxLayout()
        magnet_layout.addWidget(QLabel("电磁铁："))

        btn_mag_on = QPushButton("开")
        btn_mag_on.clicked.connect(lambda:self.get_electromagnet(True))
        magnet_layout.addWidget(btn_mag_on)

        btn_mag_on = QPushButton("关")
        btn_mag_on.clicked.connect(lambda:self.get_electromagnet(False))
        magnet_layout.addWidget(btn_mag_on)

        func_layout.addLayout(magnet_layout)
        func_layout.addSpacing(20)
        
        btn_estop = QPushButton("急停")
        btn_estop.setStyleSheet("background-color: red; color: white; font-weight: bold;")
        btn_estop.clicked.connect(self.stop_clicked)
        func_layout.addWidget(btn_estop)
        
        layout.addLayout(func_layout)
        layout.addStretch()

    # ===== 事件处理 =====
    def connect_clicked(self):
        self.ros_system.start_ros()

    def on_connected(self):
        self.reset_velocity_button_states()
        self.on_backend_status(self.dispatch_manager.status_summary())

    def on_disconnected(self):
        # 禁用按钮
        for btn in self.velocity_send_buttons + self.velocity_stop_buttons:
            btn.setEnabled(False)
        self.on_backend_status(self.dispatch_manager.status_summary())

    def on_log_internal(self, msg, level):
        self.log_signal.emit(msg, level, self.joint_name)

    def on_backend_status(self, summary):
        backends = dict(summary.get("backends", {}))
        for source in BACKEND_SOURCES:
            label = self.backend_status_labels[source]
            online = bool(backends.get(source, {}).get("online"))
            reason = str(backends.get(source, {}).get("reason", "offline"))
            label.setText("online" if online else f"offline ({reason})")
            label.setStyleSheet(
                "color: green; font-weight: bold;"
                if online
                else "color: red; font-weight: bold;"
            )

    def on_compact_feedback(self, source, motor_id, raw_pulses, parsed_feedback):
        """处理来自 dispatch manager 的带来源反馈。"""
        del parsed_feedback
        if source not in self.feedback_state:
            return
        if self.joint_name == "尖端":
            self._update_tip_feedback(source, motor_id, raw_pulses)
            return
        self._update_joint_feedback(source, motor_id, raw_pulses)

    def update_status_display(self):
        pass

    def _update_tip_feedback(self, source, motor_id, raw_pulses):
        if motor_id != 4:
            return
        state = self.feedback_state[source]
        diff = raw_pulses - state["motor4_last_raw"]
        if not self.is_setting_zero and abs(raw_pulses) < 100 and diff < -2000:
            state["motor4_offset"] += state["motor4_last_raw"]
            self.log_signal.emit(
                f"[{source}] 检测到尖端电机重启，应用偏移: {state['motor4_offset']}",
                "warning",
                self.joint_name,
            )
        state["motor4_last_raw"] = raw_pulses
        adjusted_pulses = raw_pulses + state["motor4_offset"]
        state["position_status_raw_4"] = adjusted_pulses
        display_val, suffix = counts_to_physical(4, adjusted_pulses)
        label = self.position_status[source].get(4)
        if label is not None:
            label.setText(f"位置: {display_val:.2f}{suffix}")
            label.setStyleSheet("color: blue; font-weight: bold; font-size: 16px;")

    def _update_joint_feedback(self, source, motor_id, raw_pulses):
        idx = motor_id - 1
        if idx < 0 or idx >= 3:
            return
        state = self.feedback_state[source]
        diff = raw_pulses - state["motor_last_raw"][idx]
        if not self.is_setting_zero and abs(raw_pulses) < 100 and diff < -2000:
            state["motor_offsets"][idx] += state["motor_last_raw"][idx]
            self.log_signal.emit(
                f"[{source}] 检测到电机{motor_id}重启，应用偏移: {state['motor_offsets'][idx]}",
                "warning",
                self.joint_name,
            )
        state["motor_last_raw"][idx] = raw_pulses
        adjusted_pulses = raw_pulses + state["motor_offsets"][idx]
        state["motor_positions"][idx] = adjusted_pulses
        params = MOTOR_CONVERSION_PARAMS.get(motor_id)
        if params is None:
            return
        display_val, suffix = counts_to_physical(motor_id, adjusted_pulses)
        label = self.position_status[source].get(motor_id)
        if label is not None:
            label.setText(f"位置: {display_val:.2f}{suffix}")
            label.setStyleSheet("color: blue; font-weight: bold; font-size: 16px;")

    def _default_position_text(self, motor_id):
        if motor_id == 1:
            return "位置: 0.00 mm"
        if motor_id == 4:
            return "位置: 0.00 mm"
        return "位置: 0.00 °"

    def send_single_velocity_command(self, motor_id):
        try:
            if self.joint_name == "尖端":
                idx = 0
            else:
                idx = motor_id - 1
            val = self.velocity_inputs[idx].value()
            
            # 转换单位到 pulses/s
            raw_val = val
            params = MOTOR_CONVERSION_PARAMS.get(motor_id)
            if params:
                val = physical_to_counts(motor_id, val)
            
            if self.ros_system.set_motor_velocity(motor_id, val):
                self.log_signal.emit(f"电机{motor_id} 速度设为 {raw_val} ({val:.1f} cps)", "send", self.joint_name)
                self.update_velocity_button_state(motor_id, True)
            else:
                self.log_signal.emit(f"电机{motor_id} 速度设置失败", "error", self.joint_name)
                
        except Exception as e:
            self.log_signal.emit(f"发送速度错误: {e}", "error", self.joint_name)
        
    
    def get_physical_positions(self, *, source=None):
        """
        返回当前关节的 3 个监测通道的物理位置。
        - 对于普通关节，motor1/2/3 分别对应直线/旋转2/旋转3
        - 对于“尖端”，仅 motor4 有意义，其余填 0
        """
        selected_source = source or self.dispatch_manager.preferred_feedback_source() or BACKEND_SOURCE_SIM
        state = self.feedback_state.get(selected_source, {})

        if self.joint_name == "尖端":
            raw = state.get("position_status_raw_4", 0)
            mm, _ = counts_to_physical(4, raw)
            return [mm, 0.0, 0.0]

        phys_pos = [0.0, 0.0, 0.0]
        for i in range(3):
            motor_id = i + 1
            raw_positions = state.get("motor_positions", [0, 0, 0])
            raw = raw_positions[i]
            params = MOTOR_CONVERSION_PARAMS.get(motor_id)
            if params is None:
                continue
            phys_val, _ = counts_to_physical(motor_id, raw)
            phys_pos[i] = phys_val
        return phys_pos


    def get_electromagnet(self, is_on):
        """设置电磁铁状态(0x05)"""
        val = 1 if is_on else 0
        if self.ros_system.send_compact_command(3, 0x05, val, 0, 0):
            state_str = "开启" if is_on else "关闭"
            self.log_signal.emit(f"电磁铁{state_str} (3s超时)", "send", self.joint_name)
        else:
            self.log_signal.emit(f"电磁铁指令发送失败", "error", self.joint_name)

    def stop_single_velocity_command(self, motor_id):
        if self.ros_system.send_compact_command(motor_id, 0x01):
            self.log_signal.emit(f"电机{motor_id} 停止", "send", self.joint_name)
            self.update_velocity_button_state(motor_id, False)

    def zero_position_clicked(self):
        target_motor_ids = self.ros_system.get_zero_target_motor_ids()
        if self.ros_system.set_zero_position():
            motors_text = ",".join(str(mid) for mid in target_motor_ids)
            self.log_signal.emit(f"已发送零位设置到电机{motors_text}", "send", self.joint_name)
            # 标记正在设置零位，防止触发断电重启保护
            self.is_setting_zero = True
            for state in self.feedback_state.values():
                state["motor_offsets"] = [0, 0, 0]
                state["motor4_offset"] = 0
            # 1秒后清除标志
            QTimer.singleShot(1000, lambda: setattr(self, 'is_setting_zero', False))
        else:
            self.log_signal.emit("零位设置发送失败", "error", self.joint_name)

    def stop_clicked(self):
        self.ros_system.emergency_stop()

    def reset_velocity_button_states(self):
        for btn in self.velocity_send_buttons:
            btn.setEnabled(True)
            btn.setStyleSheet("")
        for btn in self.velocity_stop_buttons:
            btn.setEnabled(True)
            btn.setStyleSheet("background-color: #FF851B; color: white; font-weight: bold;")

    def update_velocity_button_state(self, motor_id, is_active):
        if self.joint_name == "尖端":
            idx = 0
        else:
            idx = motor_id - 1
        if 0 <= idx < len(self.velocity_send_buttons):
            if is_active:
                self.velocity_send_buttons[idx].setStyleSheet("background-color: #2ECC40; color: white; font-weight: bold;")
            else:
                self.velocity_send_buttons[idx].setStyleSheet("")

    def is_connected(self):
        return self.ros_system.connected

    def cleanup(self):
        self.ros_system.stop_ros()

    def set_velocity_physical(self, motor_id, v_phys):
        cps = physical_to_counts(motor_id, v_phys)
        self.ros_system.set_motor_velocity(motor_id, cps)


def _resolve_record_base_dir(requested_base_dir: str) -> str:
    """Resolve a writable recording directory for GUI data capture."""
    requested_path = Path(requested_base_dir)
    if not requested_path.is_absolute():
        requested_path = Path.cwd() / requested_path
    try:
        requested_path.mkdir(parents=True, exist_ok=True)
        return str(requested_path)
    except OSError:
        fallback_path = Path("/tmp/mpc_control_new_records")
        fallback_path.mkdir(parents=True, exist_ok=True)
        return str(fallback_path)


# ===== 数据记录器 =====
class DataRecorder(QObject):
    log_signal = pyqtSignal(str, str)

    def __init__(self, joint_map):
        super().__init__()
        self.joint_map = joint_map
        self.recording = False
        self.timer = QTimer()
        self.timer.timeout.connect(self.sample_data)
        self.start_time = 0
        self.files = {} # joint_name -> file_handle
        requested_base_dir = os.environ.get("MPC_CONTROL_NEW_RECORD_DIR", "records")
        self.base_dir = _resolve_record_base_dir(requested_base_dir)
        os.makedirs(self.base_dir, exist_ok=True)

    def start_recording(self):
        if self.recording:
            return
        
        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.current_dir = os.path.join(self.base_dir, timestamp)
            os.makedirs(self.current_dir)
            
            self.files = {}
            # Create a CSV for each joint
            for name in self.joint_map.keys():
                filepath = os.path.join(self.current_dir, f"{name}.csv")
                f = open(filepath, 'w', encoding='utf-8-sig') # utf-8-sig for Excel compatibility
                # Header: Time, Motor1, Motor2, Motor3
                # Motor 1 is Linear, 2 and 3 are Rotary usually
                f.write("Time(s),Motor1(Linear),Motor2(Rotary),Motor3(Rotary)\n")
                self.files[name] = f
                
            self.start_time = time.time()
            self.timer.start(100) # 10Hz 数据记录频率
            self.recording = True
            self.log_signal.emit(f"开始记录数据: {self.current_dir}", "info")
            return True
        except Exception as e:
            self.log_signal.emit(f"启动记录失败: {str(e)}", "error")
            self.stop_recording()
            return False

    def stop_recording(self):
        if not self.recording:
            return
            
        self.timer.stop()
        for f in self.files.values():
            try:
                f.close()
            except:
                pass
        self.files = {}
        self.recording = False
        self.log_signal.emit("数据记录已停止并保存", "info")

    def sample_data(self):
        try:
            t_now = time.time() # 使用绝对 Unix 时间戳 
            
            for name, widget in self.joint_map.items():
                if name in self.files:
                    f = self.files[name]
                    # Get physical positions
                    pos = widget.get_physical_positions()
                    # Write: Time, M1, M2, M3
                    # Note: pos index 0->M1, 1->M2, 2->M3
                    line = f"{t_now:.3f},{pos[0]:.3f},{pos[1]:.3f},{pos[2]:.3f}\n"
                    f.write(line)
                    f.flush() # Ensure data is written to disk
        except Exception as e:
            # Prevent flooding logs if error occurs repeatedly
            pass

# ==== GamepadWorker线程 ====
class GamepadState:
    def __init__(self, lx=0.0, ly=0.0, rx=0.0, lt=False, rt=False):
        self.lx = lx
        self.ly = ly
        self.rx = rx
        self.lt = lt
        self.rt = rt


class GamepadWorker(QThread):
    state_updated = pyqtSignal('PyQt_PyObject')

    def __init__(self):
        super().__init__()
        self._pygame_available = pygame is not None

        self.running = True
        self.connected = False
        self.prev_state = None
        self.joy = None

        if not self._pygame_available:
            return

        pygame.display.init()
        pygame.joystick.init()

        pygame.event.set_allowed(None)      # 禁用除 joystick 外所有事件
        pygame.event.set_allowed(pygame.JOYAXISMOTION)
        pygame.event.set_allowed(pygame.JOYBUTTONDOWN)
        pygame.event.set_allowed(pygame.JOYBUTTONUP)

        if pygame.joystick.get_count() > 0:
            self.joy = pygame.joystick.Joystick(0)
            self.joy.init()
            self.connected = True

    def run(self):
        if not self._pygame_available:
            while self.running:
                time.sleep(0.05)
            return
        CLOCK = pygame.time.Clock()

        while self.running:
            # 自动重连
            if not self.connected:
                pygame.joystick.quit()
                pygame.joystick.init()
                if pygame.joystick.get_count() > 0:
                    self.joy = pygame.joystick.Joystick(0)
                    self.joy.init()
                    self.connected = True
                CLOCK.tick(30)
                continue

            pygame.event.pump()   # 非阻塞

            lx = round(self.joy.get_axis(0), 3)
            ly = round(-self.joy.get_axis(1), 3)
            rx = round(self.joy.get_axis(3), 3)

            # LT / RT 作为触发轴（常见模式）
            lt_val = self.joy.get_axis(2) if self.joy.get_numaxes() > 2 else 0.0
            rt_val = self.joy.get_axis(5) if self.joy.get_numaxes() > 5 else 0.0

            lb = self.joy.get_button(4)   # LB
            rb = self.joy.get_button(5)   # RB
            x_btn = self.joy.get_button(2)   # X 按钮（全局急停）

            lt = lt_val > 0.5
            rt = rt_val > 0.5

            st = GamepadState(lx, ly, rx, lt, rt)
            st.lb = lb
            st.rb = rb
            st.x = x_btn

            current_state = (lx, ly, rx, lt, rt, lb, rb, x_btn)
            if self.prev_state != current_state:
                self.prev_state = current_state
                self.state_updated.emit(st)

            CLOCK.tick(240)
    
    def stop(self):
        self.running = False


class GamepadController(QObject):
    def __init__(self, window):
        super().__init__()
        self.window = window     # ★ 保存 GUI 主窗口对象
        self.enabled = False
        self.deadzone = 0.25
        self.margin = 0.15
        self.last_motor = None
        self.tab_switch_lock = False    # RT / LT 专用
        self.pair_switch_lock = False   # LB / RB 专用
        self.estop_lock = False   # X 键急停防抖锁
        # ===== 协同控制相关状态 =====
        self.control_mode = "SINGLE"   # 可选: SINGLE / TIP_JOINT1 / JOINT_PAIR
        self.joint_pair_index = 0      # 仅在 JOINT_PAIR 模式下使用 (0~3)

        # ===== 速度模式发包收敛（关键）=====
        self._vel_active = {}        # key: (joint_name, motor_id) -> bool
        self._last_vel_sent = {}     # key -> last sent physical velocity (user unit)
        self._last_send_ts = {}      # key -> time.time()
        self._send_hz = 30.0         # 速度更新频率上限（Hz）
        self._vel_eps = 1e-3         # 速度变化阈值（物理单位），可按需要调大

    def enable(self, flag):
        self.enabled = flag

    def handle_state(self, st: GamepadState):
        if not self.enabled:
            return
        
        # 手柄 X 键 = 全局急停
        if hasattr(st, "x"):
            if st.x and not self.estop_lock:
                self.estop_lock = True
                # 调用 GUI 的全局急停（与按钮完全一致）
                self.window.global_emergency_stop()
            elif not st.x:
                # 松开后解除锁
                self.estop_lock = False
        
        # === 摇杆噪声死区（消除 ±0.05 抖动） ===
        lx = 0 if abs(st.lx) < 0.05 else st.lx
        ly = 0 if abs(st.ly) < 0.05 else st.ly
        rx = 0 if abs(st.rx) < 0.05 else st.rx

        # ① 处理 LT/RT 切换标签页
        # ===== RT / LT 标签页切换改为“按下触发一次” =====
        if st.rt and not self.tab_switch_lock:
            self.tab_switch_lock = True
            idx = self.window.tab_widget.currentIndex()
            self.window.tab_widget.setCurrentIndex((idx + 1) % self.window.tab_widget.count())

        elif st.lt and not self.tab_switch_lock:
            self.tab_switch_lock = True
            idx = self.window.tab_widget.currentIndex()
            self.window.tab_widget.setCurrentIndex((idx - 1) % self.window.tab_widget.count())

        # 当按键松开时重置锁
        if not st.rt and not st.lt:
            self.tab_switch_lock = False

        # ② 获取当前 joint widget
        joint_widget = self.window.get_current_joint_widget()
        if joint_widget is None:
            return
        
        # ④ 控制模式分发（目前只接 SINGLE）
        if self.control_mode == "SINGLE":
            self._handle_single_control(joint_widget, lx, ly, rx)
        elif self.control_mode == "TIP_JOINT1":
            self._handle_tip_joint1_coupling(lx, ly, rx)
        elif self.control_mode == "JOINT_PAIR":
            self._handle_joint_pair_coupling(lx, ly, rx, st)
        elif self.control_mode == "PIPE_FEED":
            self._handle_pipe_feed_control(ly)

    def _key(self, joint_widget, motor_id):
        return (joint_widget.joint_name, motor_id)
    
    def _enter_velocity_mode(self, joint_widget, motor_id, vel_physical):
        """边沿触发：进入速度模式（发送一次 0x02）"""
        k = self._key(joint_widget, motor_id)
        self._vel_active[k] = True
        self._last_vel_sent[k] = None
        self._last_send_ts[k] = 0.0
        joint_widget.set_velocity_physical(motor_id, vel_physical)  # 会发 0x02（enter + set）

    def _update_velocity_mode(self, joint_widget, motor_id, vel_physical):
            """模式内更新：限频 + 阈值"""
            k = self._key(joint_widget, motor_id)
            now = time.time()
            min_dt = 1.0 / self._send_hz

            last_t = self._last_send_ts.get(k, 0.0)
            if (now - last_t) < min_dt:
                return

            last_v = self._last_vel_sent.get(k, None)
            if last_v is not None and abs(vel_physical - last_v) < self._vel_eps:
                return

            self._last_send_ts[k] = now
            self._last_vel_sent[k] = vel_physical
            joint_widget.set_velocity_physical(motor_id, vel_physical)  # 仅更新目标速度（0x02）

    def _exit_velocity_mode(self, joint_widget, motor_id):
        """边沿触发：退出速度模式（发送一次 0x01）"""
        k = self._key(joint_widget, motor_id)
        if self._vel_active.get(k, False):
            self._vel_active[k] = False
            self._last_vel_sent[k] = None
            self._last_send_ts[k] = 0.0
            joint_widget.stop_single_velocity_command(motor_id)     # 发 0x01（stop）

    def _handle_single_control(self, joint_widget, lx, ly, rx):
        """
        单模块速度控制（原有逻辑，行为保持不变）
        """
        joint_name = joint_widget.joint_name

        # ③ 尖端特殊逻辑
        if joint_name == "尖端":
            if abs(ly) < self.deadzone:
                self._exit_velocity_mode(joint_widget, 4)
                return
            v4 = joint_widget.velocity_inputs[0].value()
            direction = +1 if ly > 0 else -1
            v_cmd = direction * v4
            k = self._key(joint_widget, 4)
            if not self._vel_active.get(k, False):
                self._enter_velocity_mode(joint_widget, 4, v_cmd)
            else:
                self._update_velocity_mode(joint_widget, 4, v_cmd)
            return

        # ④ 普通关节 motor1/2/3 互斥策略（方案 B+）
        a1 = abs(ly)
        a2 = abs(lx)
        a3 = abs(rx)

        a1 = abs(ly); a2 = abs(lx); a3 = abs(rx)

        # 1) 全回中：只在边沿触发 stop（对三个电机各发一次 stop）
        if a1 < self.deadzone and a2 < self.deadzone and a3 < self.deadzone:
            for m in [1, 2, 3]:
                self._exit_velocity_mode(joint_widget, m)
            self.last_motor = None
            return

        # 2) 选电机（沿用你 B+ 互斥策略）
        axes = {1: a1, 2: a2, 3: a3}
        motor_candidate = max(axes, key=axes.get)
        Amax = axes[motor_candidate]
        A2v = sorted(axes.values())[-2]
        motor_to_use = self.last_motor if (self.last_motor is not None and (Amax - A2v) < self.margin) else motor_candidate

        # 3) 如果电机发生切换：只 stop “上一个电机”一次（边沿）
        if self.last_motor is not None and self.last_motor != motor_to_use:
            self._exit_velocity_mode(joint_widget, self.last_motor)

        # 4) 计算目标速度（物理单位）
        v1 = joint_widget.velocity_inputs[0].value()
        v2 = joint_widget.velocity_inputs[1].value()
        v3 = joint_widget.velocity_inputs[2].value()

        if motor_to_use == 1:
            direction = +1 if ly > 0 else -1
            v_cmd = direction * v1
        elif motor_to_use == 2:
            direction = -1 if lx > 0 else +1
            v_cmd = direction * v2
        else:
            direction = -1 if rx > 0 else +1
            v_cmd = direction * v3

        # 5) 进入/更新速度模式（边沿 enter + 模式内限频）
        k = self._key(joint_widget, motor_to_use)
        if not self._vel_active.get(k, False):
            self._enter_velocity_mode(joint_widget, motor_to_use, v_cmd)
        else:
            self._update_velocity_mode(joint_widget, motor_to_use, v_cmd)

        self.last_motor = motor_to_use


    def _handle_tip_joint1_coupling(self, lx, ly, rx):
        """
        尖端 + 关节1 协同速度控制

        左摇杆 上/下：
            - 尖端 motor4
            - 关节1 motor1
            同向运动

        右摇杆 左/右：
            - 仅控制 关节1 motor3
            与 SINGLE 模式一致的方向逻辑

        说明：
        - 不使用 B+ 互斥
        - 允许左右摇杆同时生效
        """

        # ---------- 获取控件 ----------
        tip_widget = self.window.joint_map.get("尖端", None)
        joint1_widget = self.window.joint_map.get("关节1", None)

        if tip_widget is None or joint1_widget is None:
            return

        # =====================================================
        # 左摇杆：控制 尖端 motor4 + 关节1 motor1（同向）
        # =====================================================
        if abs(ly) < self.deadzone:
            self._exit_velocity_mode(tip_widget, 4)
            self._exit_velocity_mode(joint1_widget, 1)
        else:
            # 速度取 GUI 中的设定值
            v_tip = tip_widget.velocity_inputs[0].value()     # 尖端只有一个电机
            v_joint1_m1 = joint1_widget.velocity_inputs[0].value()  # 关节1 motor1

            direction = +1 if ly > 0 else -1

            # 尖端 motor4
            v_cmd_tip = direction * v_tip
            k_tip = self._key(tip_widget, 4)
            if not self._vel_active.get(k_tip, False):
                self._enter_velocity_mode(tip_widget, 4, v_cmd_tip)
            else:
                self._update_velocity_mode(tip_widget, 4, v_cmd_tip)

            # 关节1 motor1
            v_cmd_j1 = direction * v_joint1_m1
            k_j1 = self._key(joint1_widget, 1)
            if not self._vel_active.get(k_j1, False):
                self._enter_velocity_mode(joint1_widget, 1, v_cmd_j1)
            else:
                self._update_velocity_mode(joint1_widget, 1, v_cmd_j1)

        # =====================================================
        # 右摇杆：控制 关节1 motor3（允许与左摇杆同时）
        # =====================================================
        if abs(rx) < self.deadzone:
            self._exit_velocity_mode(joint1_widget, 3)
        else:
            v_joint1_m3 = joint1_widget.velocity_inputs[2].value()  # motor3

            # ⚠ 注意：这里沿用你当前 SINGLE 模式中
            # motor3 已经修正后的方向逻辑
            direction = -1 if rx > 0 else +1

            v_cmd_m3 = direction * v_joint1_m3
            k_m3 = self._key(joint1_widget, 3)
            if not self._vel_active.get(k_m3, False):
                self._enter_velocity_mode(joint1_widget, 3, v_cmd_m3)
            else:
                self._update_velocity_mode(joint1_widget, 3, v_cmd_m3)

    def _handle_joint_pair_coupling(self, lx, ly, rx, st):
        """
        关节对协同控制（JOINT_PAIR 模式）

        - 左摇杆：两个关节的 motor1 同向
        - 右摇杆：两个关节的 motor3 反向
        - LB / RB：切换关节对
        """

        # ========= ① 关节对定义（顺序很重要） =========
        joint_pairs = [
            ("关节1", "关节2"),
            ("关节2", "关节3"),
            ("关节3", "关节4"),
            ("关节4", "关节5"),
        ]

        # ========= ② LB / RB 切换关节对（沿用 switch_lock） =========
        if hasattr(st, "lb") and st.lb and not self.pair_switch_lock:
            self.pair_switch_lock = True
            self.joint_pair_index = (self.joint_pair_index - 1) % len(joint_pairs)
            self.window.update_coupling_status_label()

        elif hasattr(st, "rb") and st.rb and not self.pair_switch_lock:
            self.pair_switch_lock = True
            self.joint_pair_index = (self.joint_pair_index + 1) % len(joint_pairs)
            self.window.update_coupling_status_label()

        if not getattr(st, "lb", False) and not getattr(st, "rb", False):
            self.pair_switch_lock = False

        # ========= ③ 获取当前关节对 =========
        j1_name, j2_name = joint_pairs[self.joint_pair_index]

        j1 = self.window.joint_map.get(j1_name)
        j2 = self.window.joint_map.get(j2_name)

        if j1 is None or j2 is None:
            return

        # ========= ④ 左摇杆：motor1 同向 =========
        if abs(ly) < self.deadzone:
            self._exit_velocity_mode(j1, 1)
            self._exit_velocity_mode(j2, 1)
        else:
            v1_j1 = j1.velocity_inputs[0].value()
            v1_j2 = j2.velocity_inputs[0].value()
            direction = +1 if ly > 0 else -1

            v_cmd1 = direction * v1_j1
            k1 = self._key(j1, 1)
            if not self._vel_active.get(k1, False):
                self._enter_velocity_mode(j1, 1, v_cmd1)
            else:
                self._update_velocity_mode(j1, 1, v_cmd1)

            v_cmd2 = direction * v1_j2
            k2 = self._key(j2, 1)
            if not self._vel_active.get(k2, False):
                self._enter_velocity_mode(j2, 1, v_cmd2)
            else:
                self._update_velocity_mode(j2, 1, v_cmd2)

        # ========= ⑤ 右摇杆：motor3 反向 =========
        if abs(rx) < self.deadzone:
            self._exit_velocity_mode(j1, 3)
            self._exit_velocity_mode(j2, 3)
        else:
            v3_j1 = j1.velocity_inputs[2].value()
            v3_j2 = j2.velocity_inputs[2].value()

            if rx > 0:
                dir_j1, dir_j2 = -1, +1
            else:
                dir_j1, dir_j2 = +1, -1

            v_cmd1 = dir_j1 * v3_j1
            k1 = self._key(j1, 3)
            if not self._vel_active.get(k1, False):
                self._enter_velocity_mode(j1, 3, v_cmd1)
            else:
                self._update_velocity_mode(j1, 3, v_cmd1)

            v_cmd2 = dir_j2 * v3_j2
            k2 = self._key(j2, 3)
            if not self._vel_active.get(k2, False):
                self._enter_velocity_mode(j2, 3, v_cmd2)
            else:
                self._update_velocity_mode(j2, 3, v_cmd2)

    def _handle_pipe_feed_control(self, ly):
        # 所有关节并行，不看当前 tab

        for joint_name, widget in self.window.joint_map.items():

            # ① 跳过尖端
            if joint_name == "尖端":
                continue

            # ② 跳过未连接节点
            if not widget.is_connected():
                continue

            # ③ deadzone：持续 stop motor1
            if abs(ly) < self.deadzone:
                widget.stop_single_velocity_command(1)
                continue

            # ④ 方向约定
            # 上推（ly > 0） → 反向
            # 下推（ly < 0） → 正向
            direction = -1 if ly > 0 else +1

            # ⑤ 速度幅值取各自 GUI 中的 motor1 设置
            v = widget.velocity_inputs[0].value()

            widget.set_velocity_physical(1, direction * v)


# ===== 主窗口 =====
class MultiJointControlGUI(QMainWindow):
    def __init__(
        self,
        *,
        ros_runtime: Ros2GuiRuntimeDependencies | None = None,
        backend_manager: GuiBackendManager | None = None,
        enable_gamepad_thread: bool = True,
        enable_embedded_sim: bool | None = None,
        enable_live_backend: bool = True,
        sim_auto_step_ms: int = 50,
    ):
        super().__init__()
        self.setWindowTitle("ESP32 ROS2 机器人控制系统")
        self.resize(1200, 900)
        self.backend_manager = backend_manager or build_gui_backend_manager(
            ros_runtime=ros_runtime,
            enable_live_backend=enable_live_backend,
            enable_embedded_sim=enable_embedded_sim,
            sim_auto_step_ms=sim_auto_step_ms,
        )
        self.enable_gamepad_thread = enable_gamepad_thread
        
        self.gamepad_worker = None
        self.gamepad_controller = None
        
        self.init_ui()
        self.backend_manager.log_signal.connect(self.add_log)
        self.backend_manager.status_signal.connect(self.on_backend_status_summary)

        # 初始化数据记录器
        self.recorder = DataRecorder(self.joint_map)
        self.recorder.log_signal.connect(lambda msg, level: self.add_log(msg, level))
        
        self.btn_record_start.clicked.connect(self.start_record_clicked)
        self.btn_record_stop.clicked.connect(self.stop_record_clicked)

        self.backend_manager.start()
        self.on_backend_status_summary(self.backend_manager.status_summary())
        self.add_log("系统启动", "system")

        # 初始化手柄系统
        self.gamepad_controller = GamepadController(self)
        if self.enable_gamepad_thread:
            self.gamepad_worker = GamepadWorker()
            self.gamepad_worker.state_updated.connect(self.gamepad_controller.handle_state)
            self.gamepad_worker.start()
        self.update_coupling_status_label()

        # ===== GUI 控制模式 → GamepadController 绑定 =====
        def on_mode_changed(text):
            if text == "单关节控制":
                self.gamepad_controller.control_mode = "SINGLE"
            elif text == "尖端 + 关节1 协同":
                self.gamepad_controller.control_mode = "TIP_JOINT1"
            elif text == "关节协同":
                self.gamepad_controller.control_mode = "JOINT_PAIR"
            elif text == "送管模式":
                self.gamepad_controller.control_mode = "PIPE_FEED"

            # 切换模式时清理状态，避免残留互斥逻辑
            self.gamepad_controller.last_motor = None

            # 同步更新状态显示
            self.update_coupling_status_label()

        self.combo_control_mode.currentTextChanged.connect(on_mode_changed)

        # 初始化一次状态显示
        self.update_coupling_status_label()

    def init_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        
        # 1. 全局控制
        global_grp = QGroupBox("全局控制")
        global_layout = QHBoxLayout()

        self.combo_control_mode = QComboBox()
        self.combo_control_mode.addItems([
            "单关节控制",
            "尖端 + 关节1 协同",
            "关节协同",
            "送管模式"
        ])
        global_layout.addWidget(QLabel("控制模式："))
        global_layout.addWidget(self.combo_control_mode)

        # --- 当前协同状态显示 ---
        self.label_coupling_status = QLabel("当前模式：单关节控制")
        self.label_coupling_status.setStyleSheet(
            "color: #0074D9; font-weight: bold;"
        )
        global_layout.addWidget(self.label_coupling_status)

        self.label_sim_status = QLabel("SIM: offline")
        self.label_sim_status.setStyleSheet("color: red; font-weight: bold;")
        global_layout.addWidget(self.label_sim_status)

        self.label_live_status = QLabel("LIVE: offline")
        self.label_live_status.setStyleSheet("color: red; font-weight: bold;")
        global_layout.addWidget(self.label_live_status)

        self.label_dispatch_status = QLabel("Dispatch: none")
        self.label_dispatch_status.setStyleSheet("color: #85144b; font-weight: bold;")
        global_layout.addWidget(self.label_dispatch_status)

        global_grp.setLayout(global_layout)
        main_layout.addWidget(global_grp)
        
        btn_estop = QPushButton("全局急停")
        btn_estop.setStyleSheet("background-color: red; color: white; font-weight: bold; padding: 10px;")
        btn_estop.clicked.connect(self.global_emergency_stop)
        global_layout.addWidget(btn_estop)

        global_layout.addSpacing(20)
        
        self.btn_record_start = QPushButton("开始记录")
        self.btn_record_start.setStyleSheet("background-color: #2ECC40; color: white; font-weight: bold; padding: 10px;")
        global_layout.addWidget(self.btn_record_start)
        
        self.btn_record_stop = QPushButton("停止并保存")
        self.btn_record_stop.setStyleSheet("background-color: #FF851B; color: white; font-weight: bold; padding: 10px;")
        self.btn_record_stop.setEnabled(False)
        global_layout.addWidget(self.btn_record_stop)
        
        global_grp.setLayout(global_layout)
        main_layout.addWidget(global_grp)

        # 添加“启用手柄控制”按钮
        self.btn_gamepad = QPushButton("启用手柄控制")
        self.btn_gamepad.setCheckable(True)

        def toggle_gamepad():
            enabled = self.btn_gamepad.isChecked()
            self.gamepad_controller.enable(enabled)
            if enabled:
                self.btn_gamepad.setText("手柄控制：开启")
            else:
                self.btn_gamepad.setText("手柄控制：关闭")

        self.btn_gamepad.toggled.connect(toggle_gamepad)
        global_layout.addWidget(self.btn_gamepad)
        
        # 2. 关节标签页
        self.tab_widget = QTabWidget()
        self.joint_widgets = []
        self.joint_map = {}
        
        for config in JOINT_CONFIGS:
            w = JointControlWidget(config, dispatch_manager=self.backend_manager)
            w.log_signal.connect(self.on_joint_log)
            self.joint_widgets.append(w)
            self.joint_map[config["name"]] = w
            self.tab_widget.addTab(w, config["name"])
            
        main_layout.addWidget(self.tab_widget)
        
        # # 3.替换运动控制
        # self.replace_motion_widget = ReplacementMotionWidget(self.joint_map)
        # main_layout.addWidget(self.replace_motion_widget)
        
        # 4. 日志区域
        log_grp = QGroupBox("系统日志")
        log_layout = QVBoxLayout()
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMinimumHeight(200)
        log_layout.addWidget(self.log_text)
        log_grp.setLayout(log_layout)
        main_layout.addWidget(log_grp)

    def on_backend_status_summary(self, summary):
        backends = dict(summary.get("backends", {}))
        self._set_backend_label(self.label_sim_status, "SIM", backends.get(BACKEND_SOURCE_SIM, {}))
        self._set_backend_label(self.label_live_status, "LIVE", backends.get(BACKEND_SOURCE_LIVE, {}))
        dispatch_mode = str(summary.get("dispatch_mode", "none"))
        dispatch_color = "#2ECC40" if dispatch_mode in {"sim", "live", "both"} else "#85144b"
        self.label_dispatch_status.setText(f"Dispatch: {dispatch_mode}")
        self.label_dispatch_status.setStyleSheet(f"color: {dispatch_color}; font-weight: bold;")

    def _set_backend_label(self, label, prefix, snapshot):
        online = bool(snapshot.get("online"))
        reason = str(snapshot.get("reason", "offline"))
        label.setText(f"{prefix}: {'online' if online else f'offline ({reason})'}")
        label.setStyleSheet(
            "color: green; font-weight: bold;"
            if online
            else "color: red; font-weight: bold;"
        )

    def start_record_clicked(self):
        if self.recorder.start_recording():
            self.btn_record_start.setEnabled(False)
            self.btn_record_stop.setEnabled(True)
            self.btn_record_start.setStyleSheet("background-color: gray; color: white; font-weight: bold; padding: 10px;")

    def stop_record_clicked(self):
        self.recorder.stop_recording()
        self.btn_record_start.setEnabled(True)
        self.btn_record_stop.setEnabled(False)
        self.btn_record_start.setStyleSheet("background-color: #2ECC40; color: white; font-weight: bold; padding: 10px;")

    def global_emergency_stop(self):
        self.backend_manager.emergency_stop()
        self.add_log("执行全局急停", "system")

    def on_joint_log(self, msg, level, joint_name):
        self.add_log(f"[{joint_name}] {msg}", level)

    def add_log(self, message, level="info"):
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        color = "black"
        if level == "error": color = "red"
        elif level == "warning": color = "orange"
        elif level == "send": color = "blue"
        elif level == "recv": color = "green"
        
        self.log_text.append(f'<span style="color:{color}">[{timestamp}] {message}</span>')
        self.log_text.moveCursor(QTextCursor.End)

    def get_current_joint_widget(self):
        """返回当前 Tab 上对应的 JointControlWidget"""
        idx = self.tab_widget.currentIndex()
        if 0 <= idx < len(self.joint_widgets):
            return self.joint_widgets[idx]
        return None
    
    def update_coupling_status_label(self):
        """
        更新 GUI 上显示的协同控制状态
        """
        mode = self.gamepad_controller.control_mode

        if mode == "SINGLE":
            text = "当前模式：单关节控制"

        elif mode == "TIP_JOINT1":
            text = "当前模式：尖端 + 关节1 协同"

        elif mode == "JOINT_PAIR":
            pairs = [
                "关节1 + 关节2",
                "关节2 + 关节3",
                "关节3 + 关节4",
                "关节4 + 关节5",
            ]
            idx = self.gamepad_controller.joint_pair_index
            text = f"当前模式：关节协同（{pairs[idx]}）"

        elif mode == "PIPE_FEED":
            text = "当前模式：送管模式（全关节直线联动）"
            
        else:
            text = "当前模式：未知"

        self.label_coupling_status.setText(text)

    def closeEvent(self, event):
        self.shutdown()
        event.accept()

    def shutdown(self):
        if getattr(self, "recorder", None) is not None and self.recorder.recording:
            self.recorder.stop_recording()
        if self.gamepad_worker is not None:
            self.gamepad_worker.stop()
            self.gamepad_worker.wait(500)
        for w in self.joint_widgets:
            w.cleanup()
        self.backend_manager.shutdown()


def _window_summary(window: MultiJointControlGUI) -> dict[str, object]:
    """Return a compact GUI summary for tests and launch diagnostics."""
    status = window.backend_manager.status_summary()
    feedback_snapshot = window.backend_manager.feedback_snapshot()
    return {
        "window_title": window.windowTitle(),
        "joint_names": list(window.joint_map),
        "topic_namespaces": [widget.topic_ns for widget in window.joint_widgets],
        "dispatch_mode": status.get("dispatch_mode"),
        "backends": status.get("backends"),
        "feedback_sources": {
            source: sorted(namespaces)
            for source, namespaces in (
                (source, snapshot.keys()) for source, snapshot in feedback_snapshot.items()
            )
            if namespaces
        },
        "gamepad_thread_enabled": bool(window.enable_gamepad_thread),
        "self_contained": True,
    }


def launch_gui_ros2(
    *,
    ros_runtime: Ros2GuiRuntimeDependencies | None = None,
    backend_manager: GuiBackendManager | None = None,
    headless: bool = False,
    auto_close_ms: int = 0,
    enable_gamepad: bool = False,
    check_only: bool = False,
    enable_embedded_sim: bool | None = None,
    enable_live_backend: bool = True,
    sim_auto_step_ms: int = 50,
) -> dict[str, object] | int:
    """Launch the canonical GUI entry or run a headless construction smoke pass."""
    _prepare_qt_environment(headless=headless)
    app = QApplication.instance() or QApplication(sys.argv)
    window = MultiJointControlGUI(
        ros_runtime=ros_runtime,
        backend_manager=backend_manager,
        enable_gamepad_thread=enable_gamepad,
        enable_embedded_sim=enable_embedded_sim,
        enable_live_backend=enable_live_backend,
        sim_auto_step_ms=sim_auto_step_ms,
    )
    if check_only:
        summary = _window_summary(window)
        window.shutdown()
        window.close()
        app.processEvents()
        return summary
    if headless:
        if auto_close_ms > 0:
            QTimer.singleShot(auto_close_ms, window.close)
            app.exec_()
        else:
            app.processEvents()
        summary = _window_summary(window)
        window.shutdown()
        window.close()
        app.processEvents()
        return summary
    if auto_close_ms > 0:
        QTimer.singleShot(auto_close_ms, window.close)
    window.show()
    return app.exec_()


def main(argv: list[str] | None = None) -> int:
    """CLI entry point for the canonical GUI launch path."""
    import argparse
    import json

    parser = argparse.ArgumentParser(description="Run the canonical gui_ros2 entry point.")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--auto-close-ms", type=int, default=0)
    parser.add_argument("--enable-gamepad", action="store_true")
    parser.add_argument("--check-only", action="store_true")
    parser.add_argument("--enable-embedded-sim", action="store_true")
    parser.add_argument("--disable-live-backend", action="store_true")
    args = parser.parse_args(argv)
    result = launch_gui_ros2(
        headless=args.headless,
        auto_close_ms=args.auto_close_ms,
        enable_gamepad=args.enable_gamepad,
        check_only=args.check_only,
        enable_embedded_sim=(True if args.enable_embedded_sim else None),
        enable_live_backend=not args.disable_live_backend,
    )
    if isinstance(result, dict):
        print(json.dumps(result, sort_keys=True))
        return 0
    return int(result)


if __name__ == "__main__":
    raise SystemExit(main())

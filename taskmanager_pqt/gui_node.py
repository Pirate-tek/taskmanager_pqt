#!/usr/bin/env python3

import sys
import math
import csv
import time
import subprocess
from datetime import datetime
import threading
import signal

# PyQt5
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QPushButton, QLabel, QTextEdit,
                             QGroupBox, QGridLayout, QListWidget, QCheckBox,
                             QAbstractItemView, QFrame, QSizePolicy, QStatusBar)
from PyQt5.QtCore import QTimer, pyqtSignal, QObject, Qt, QPointF, QPropertyAnimation, QEasingCurve
from PyQt5.QtGui import QPainter, QPen, QColor, QFont, QBrush, QLinearGradient, QRadialGradient

# ROS2
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped, PoseWithCovarianceStamped
from taskmanager_pqt.action import MissionTask
from std_srvs.srv import Trigger

# =========================================================================
# STYLE CONSTANTS
# =========================================================================
DARK_BG    = "#0D1117"
PANEL_BG   = "#161B22"
BORDER_COL = "#30363D"
NEON_GREEN = "#00FF9C"
SOFT_GREEN = "#2ECC71"
DANGER_RED = "#E74C3C"
WARN_AMB   = "#F39C12"
TEXT_LIGHT = "#E6EDF3"
TEXT_DIM   = "#8B949E"
BLUE_GLOW  = "#4FC3F7"
CYAN_HI    = "#00E5FF"

GLOBAL_STYLE = f"""
QMainWindow, QWidget {{
    background-color: {DARK_BG};
    color: {TEXT_LIGHT};
    font-family: 'Inter', 'Segoe UI', Arial, sans-serif;
    font-size: 13px;
}}

QGroupBox {{
    background-color: {PANEL_BG};
    border: 1px solid {BORDER_COL};
    border-radius: 8px;
    margin-top: 10px;
    padding: 8px 6px 6px 6px;
    font-weight: bold;
    font-size: 12px;
    color: {TEXT_DIM};
}}

QGroupBox::title {{
    subcontrol-origin: margin;
    left: 10px;
    padding: 0 4px;
    color: {NEON_GREEN};
    font-size: 11px;
    letter-spacing: 1px;
    text-transform: uppercase;
}}

QPushButton {{
    background-color: #21262D;
    color: {TEXT_LIGHT};
    border: 1px solid {BORDER_COL};
    border-radius: 8px;
    padding: 7px 12px;
    font-size: 12px;
    font-weight: 600;
}}

QPushButton:hover {{
    background-color: #2D333B;
    border: 1px solid {NEON_GREEN};
    color: {NEON_GREEN};
}}

QPushButton:pressed {{
    background-color: #1C2128;
    border: 1px solid {SOFT_GREEN};
}}

QPushButton:disabled {{
    background-color: #161B22;
    color: #484F58;
    border: 1px solid #21262D;
}}

QPushButton:checked {{
    background-color: #0F3D2A;
    border: 2px solid {NEON_GREEN};
    color: {NEON_GREEN};
}}

QListWidget {{
    background-color: #0D1117;
    border: 1px solid {BORDER_COL};
    border-radius: 6px;
    color: {TEXT_LIGHT};
    font-family: 'Consolas', 'Courier New', monospace;
    font-size: 12px;
    padding: 4px;
}}

QListWidget::item {{
    padding: 5px 8px;
    border-bottom: 1px solid #21262D;
    border-radius: 4px;
}}

QListWidget::item:selected {{
    background-color: #0F3D2A;
    color: {NEON_GREEN};
    border: 1px solid {NEON_GREEN}55;
}}

QListWidget::item:hover {{
    background-color: #21262D;
}}

QTextEdit {{
    background-color: #0D1117;
    border: 1px solid {BORDER_COL};
    border-radius: 6px;
    color: {TEXT_LIGHT};
    font-family: 'Consolas', 'Courier New', monospace;
    font-size: 12px;
    padding: 4px;
    selection-background-color: #0F3D2A;
}}

QCheckBox {{
    color: {TEXT_DIM};
    spacing: 8px;
}}

QCheckBox::indicator {{
    width: 16px;
    height: 16px;
    border: 1px solid {BORDER_COL};
    border-radius: 4px;
    background-color: #21262D;
}}

QCheckBox::indicator:checked {{
    background-color: {NEON_GREEN};
    border: 1px solid {NEON_GREEN};
}}

QScrollBar:vertical {{
    background-color: #161B22;
    width: 8px;
    border-radius: 4px;
}}

QScrollBar::handle:vertical {{
    background-color: {BORDER_COL};
    border-radius: 4px;
    min-height: 20px;
}}

QScrollBar::handle:vertical:hover {{
    background-color: {NEON_GREEN}80;
}}

QLabel {{
    color: {TEXT_LIGHT};
}}

QStatusBar {{
    background-color: #161B22;
    border-top: 1px solid {BORDER_COL};
    color: {TEXT_DIM};
    font-size: 11px;
}}

QFrame[frameShape="4"] {{
    color: {BORDER_COL};
}}
"""


# =========================================================================
# LED Status Indicator
# =========================================================================
class LEDIndicator(QLabel):
    GREEN  = QColor(NEON_GREEN)
    YELLOW = QColor(WARN_AMB)
    RED    = QColor(DANGER_RED)
    GRAY   = QColor("#484F58")

    def __init__(self, label_text="", parent=None):
        super().__init__(parent)
        self._color = self.GRAY
        self._label_text = label_text
        self.setFixedSize(14, 14)
        self.setToolTip(label_text)

    def set_color(self, color: QColor):
        self._color = color
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # Outer glow
        glow = QRadialGradient(7, 7, 7)
        glow.setColorAt(0.0, self._color)
        glow.setColorAt(0.5, QColor(self._color.red(), self._color.green(), self._color.blue(), 150))
        glow.setColorAt(1.0, QColor(self._color.red(), self._color.green(), self._color.blue(), 0))
        painter.setBrush(QBrush(glow))
        painter.setPen(Qt.NoPen)
        painter.drawEllipse(0, 0, 14, 14)

        # Inner dot
        painter.setBrush(QBrush(self._color))
        painter.drawEllipse(3, 3, 8, 8)


# =========================================================================
# Compass Widget
# =========================================================================
class CompassWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(80, 80)
        self._heading_deg = 0.0

    def set_heading(self, deg):
        self._heading_deg = deg
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        cx, cy = self.width() / 2, self.height() / 2
        r = min(cx, cy) - 4

        # Background
        painter.setBrush(QColor("#0D1117"))
        painter.setPen(QPen(QColor(BORDER_COL), 1))
        painter.drawEllipse(int(cx - r), int(cy - r), int(r * 2), int(r * 2))

        # Cardinal marks
        painter.setPen(QPen(QColor(TEXT_DIM), 1))
        painter.setFont(QFont("Arial", 6, QFont.Bold))
        marks = [("N", 0), ("E", 90), ("S", 180), ("W", 270)]
        for label, ang in marks:
            rad = math.radians(ang - 90)
            lx = cx + (r - 10) * math.cos(rad) - 4
            ly = cy + (r - 10) * math.sin(rad) - 5
            painter.drawText(int(lx), int(ly), 8, 10, Qt.AlignCenter, label)

        painter.save()
        painter.translate(cx, cy)
        painter.rotate(self._heading_deg)
        pen = QPen(QColor(NEON_GREEN), 2)
        painter.setPen(pen)
        painter.drawLine(0, 0, 0, -int(r - 16))
        painter.restore()


# =========================================================================
# Virtual Joystick Widget
# =========================================================================
class JoystickWidget(QWidget):
    joystickMoved = pyqtSignal(float, float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(130, 130)
        self.setMaximumSize(160, 160)
        self.moving_offset = QPointF(0, 0)
        self.grab_center = False
        self.__max_distance = 45

    def _center(self):
        return QPointF(self.width() / 2, self.height() / 2)

    def _bound_joystick(self, point):
        limit_line = point - self._center()
        distance = math.sqrt(limit_line.x()**2 + limit_line.y()**2)
        if distance < self.__max_distance:
            return point
        else:
            return QPointF(
                self._center().x() + limit_line.x() / distance * self.__max_distance,
                self._center().y() + limit_line.y() / distance * self.__max_distance
            )

    def joystick_position(self):
        if not self.grab_center:
            return 0.0, 0.0
        norm_x = self.moving_offset.x() / self.__max_distance
        norm_y = self.moving_offset.y() / self.__max_distance
        return -norm_y, -norm_x

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        cx, cy = self.width() / 2, self.height() / 2
        r = self.__max_distance

        color_ring = QColor(BLUE_GLOW) if self.isEnabled() else QColor("#30363D")
        color_handle = QColor(BLUE_GLOW) if self.isEnabled() else QColor("#484F58")

        # Outer ring
        painter.setPen(QPen(color_ring, 2))
        painter.setBrush(QColor("#0D1117"))
        painter.drawEllipse(int(cx - r - 4), int(cy - r - 4), int((r + 4) * 2), int((r + 4) * 2))

        # Crosshair lines
        painter.setPen(QPen(color_ring, 1, Qt.DotLine))
        painter.drawLine(int(cx - r + 4), int(cy), int(cx + r - 4), int(cy))
        painter.drawLine(int(cx), int(cy - r + 4), int(cx), int(cy + r - 4))

        # Handle glow
        hx = cx + self.moving_offset.x()
        hy = cy + self.moving_offset.y()
        glow = QRadialGradient(hx, hy, 22)
        glow.setColorAt(0, color_handle)
        glow.setColorAt(0.5, QColor(color_handle.red(), color_handle.green(), color_handle.blue(), 120))
        glow.setColorAt(1, QColor(color_handle.red(), color_handle.green(), color_handle.blue(), 0))
        painter.setBrush(QBrush(glow))
        painter.setPen(Qt.NoPen)
        painter.drawEllipse(int(hx - 22), int(hy - 22), 44, 44)

        # Handle core
        painter.setBrush(color_handle)
        painter.setPen(QPen(QColor("#FFFFFF") if self.isEnabled() else QColor("#30363D"), 1))
        painter.drawEllipse(int(hx - 12), int(hy - 12), 24, 24)

    def mousePressEvent(self, event):
        if not self.isEnabled(): return
        self.grab_center = True
        self.moving_offset = self._bound_joystick(QPointF(event.pos())) - self._center()
        self.joystickMoved.emit(*self.joystick_position())
        self.update()

    def mouseMoveEvent(self, event):
        if not self.isEnabled(): return
        if self.grab_center:
            self.moving_offset = self._bound_joystick(QPointF(event.pos())) - self._center()
            self.joystickMoved.emit(*self.joystick_position())
            self.update()

    def mouseReleaseEvent(self, event):
        if not self.isEnabled(): return
        self.grab_center = False
        self.moving_offset = QPointF(0, 0)
        self.joystickMoved.emit(0.0, 0.0)
        self.update()


# =========================================================================
# ROS 2 Node
# =========================================================================
class GuiRosNode(Node, QObject):
    log_signal              = pyqtSignal(str)
    command_finished_signal = pyqtSignal(bool, str, float)
    nearest_wp_signal       = pyqtSignal(bool, str)
    amcl_pose_signal        = pyqtSignal(float, float, float) # x, y, yaw

    def __init__(self):
        Node.__init__(self, 'advanced_gui_node')
        QObject.__init__(self)

        self.waypoints = {
            'A': (-2.53, -2.2),
            'B': (1.16,  1.23),
            'C': (-2.38, 0.545),
            'D': (-2.3,  4.49),
        }

        self.mission_logs    = []
        self.subprocess_list = []

        self.action_client       = ActionClient(self, MissionTask, 'mission_task')
        self.stop_client         = self.create_client(Trigger, '/stop_mission')
        self.shutdown_srv_client = self.create_client(Trigger, '/shutdown_taskmanager')
        self.nearest_srv_client  = self.create_client(Trigger, '/get_nearest_waypoint')
        self.joy_pub             = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        # AMCL Subscription
        qos_profile = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._amcl_cb,
            qos_profile
        )

        self._start_time   = 0.0
        self._current_cmd  = ""

    def _amcl_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        # Convert quaternion to yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Convert radians to degrees (-180 to 180) 
        yaw_deg = math.degrees(yaw)
        self.amcl_pose_signal.emit(x, y, yaw_deg)

    # ------------------------------------------------------------------
    # Subprocess launchers
    # ------------------------------------------------------------------
    def launch_simulation(self):
        self.log_signal.emit('[INFO] Launching simulation (Gazebo + Nav2)…')
        try:
            self.sim_log = open('simulation_gui.log', 'w')
            sim = subprocess.Popen(
                ['ros2', 'launch', 'taskmanager_pqt', 'mission_launch.py', 'use_sim_time:=true'],
                stdout=self.sim_log, stderr=subprocess.STDOUT)
            self.subprocess_list.append(sim)
            self.log_signal.emit('[INFO] Process spawned. Waiting ~20 s for Nav2…')
        except Exception as e:
            self.log_signal.emit(f'[ERROR] Failed to launch sim: {e}')

    def launch_executor(self):
        self.log_signal.emit('[INFO] Starting Mission Executor + Nearest Waypoint nodes…')
        try:
            self.exec_log = open('executor_gui.log', 'w')
            executor_proc = subprocess.Popen(
                ['ros2', 'run', 'taskmanager_pqt', 'mission_executor_node'],
                stdout=self.exec_log, stderr=subprocess.STDOUT)
            self.subprocess_list.append(executor_proc)

            self.nearest_log = open('nearest_wp_gui.log', 'w')
            nearest_proc = subprocess.Popen(
                ['ros2', 'run', 'taskmanager_pqt', 'nearest_waypoint_node'],
                stdout=self.nearest_log, stderr=subprocess.STDOUT)
            self.subprocess_list.append(nearest_proc)

            self.log_signal.emit('[INFO] Executor and Nearest Waypoint nodes are online.')
        except Exception as e:
            self.log_signal.emit(f'[ERROR] Failed to launch nodes: {e}')

    # ------------------------------------------------------------------
    # Joystick
    # ------------------------------------------------------------------
    def publish_joystick_cmd(self, linear, angular):
        msg = TwistStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x  = float(linear  * 0.4)
        msg.twist.angular.z = float(angular * 0.7)
        self.joy_pub.publish(msg)

    # ------------------------------------------------------------------
    # Nearest Waypoint
    # ------------------------------------------------------------------
    def trigger_nearest_waypoint(self):
        self.log_signal.emit('[INFO] Requesting nearest waypoint…')
        if not self.nearest_srv_client.wait_for_service(timeout_sec=1.0):
            self.log_signal.emit('[ERROR] /get_nearest_waypoint offline. Is nearest_waypoint_node running?')
            return
        future = self.nearest_srv_client.call_async(Trigger.Request())
        future.add_done_callback(self.nearest_response_callback)

    def nearest_response_callback(self, future):
        res = future.result()
        self.nearest_wp_signal.emit(res.success, res.message)

    # ------------------------------------------------------------------
    # Stop
    # ------------------------------------------------------------------
    def trigger_stop(self):
        self.log_signal.emit('[WARNING] Emergency Stop triggered!')
        if not self.stop_client.wait_for_service(timeout_sec=0.5):
            self.log_signal.emit('[ERROR] Stop service unavailable. Is executor running?')
            return
        self.stop_client.call_async(Trigger.Request())

    # ------------------------------------------------------------------
    # Action execution
    # ------------------------------------------------------------------
    def execute_command(self, cmd_name):
        self._current_cmd = cmd_name
        self._start_time  = time.time()
        self.log_signal.emit(f'[INFO] Executing: {cmd_name}…')

        goal_msg = MissionTask.Goal()

        if cmd_name in self.waypoints:
            coords = self.waypoints[cmd_name]
            goal_msg.waypoint_name = cmd_name
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp    = self.get_clock().now().to_msg()
            pose.pose.position.x = float(coords[0])
            pose.pose.position.y = float(coords[1])
            pose.pose.orientation.w = 1.0
            goal_msg.target_pose = pose
        else:
            goal_msg.waypoint_name = cmd_name

        if not self.action_client.wait_for_server(timeout_sec=2.0):
            self.log_signal.emit('[ERROR] Action server offline. Is Executor running?')
            self.command_finished_signal.emit(False, cmd_name, 0.0)
            return

        future = self.action_client.send_goal_async(goal_msg)
        future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.log_signal.emit(f'[ERROR] Executor rejected: {self._current_cmd}')
            self.command_finished_signal.emit(False, self._current_cmd, time.time() - self._start_time)
            return
        self.log_signal.emit(f'[INFO] {self._current_cmd} accepted, running…')
        goal_handle.get_result_async().add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future):
        result   = future.result().result
        status   = future.result().status
        duration = time.time() - self._start_time
        success  = result.success and (status == 4)

        if success:
            self.log_mission(self._current_cmd, 'SUCCESS', f'{duration:.2f}')
            self.log_signal.emit(f'[SUCCESS] {self._current_cmd} done in {duration:.1f}s: {result.message}')
        else:
            self.log_mission(self._current_cmd, 'FAILED', f'{duration:.2f}')
            self.log_signal.emit(f'[ERROR] {self._current_cmd} failed: {result.message}')

        self.command_finished_signal.emit(success, self._current_cmd, duration)

    def log_mission(self, cmd, result, duration):
        self.mission_logs.append({
            'timestamp': datetime.now().strftime('%H:%M:%S'),
            'waypoint':  cmd,
            'result':    result,
            'duration':  duration,
        })

    def shutdown_system(self):
        report = 'mission_report.csv'
        self.log_signal.emit(f'[INFO] Saving report → {report}')
        try:
            with open(report, 'w', newline='') as f:
                w = csv.DictWriter(f, fieldnames=['timestamp','waypoint','result','duration'])
                w.writeheader()
                w.writerows(self.mission_logs)
            self.log_signal.emit('[INFO] Report saved.')
        except Exception as e:
            self.log_signal.emit(f'[ERROR] Report save failed: {e}')

        for proc in self.subprocess_list:
            proc.terminate()
            try:
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                proc.kill()

        # Call the global shutdown service
        if self.shutdown_srv_client.wait_for_service(timeout_sec=0.5):
            self.log_signal.emit('[INFO] Calling global shutdown service...')
            self.shutdown_srv_client.call_async(Trigger.Request())
        else:
            self.log_signal.emit('[WARNING] Global shutdown service not available.')


# =========================================================================
# Helpers
# =========================================================================
def _make_btn(text, color=None, bold=False, font_size=12):
    btn = QPushButton(text)
    style = ""
    if color:
        style += f"background-color: {color}; "
    if bold:
        style += "font-weight: bold; "
    if font_size != 12:
        style += f"font-size: {font_size}px; "
    if style:
        btn.setStyleSheet(style + "color: #FFFFFF; border-radius: 8px; padding: 8px 12px;")
    return btn


def _section_label(text):
    lbl = QLabel(text)
    lbl.setStyleSheet(f"color: {NEON_GREEN}; font-size: 10px; font-weight: bold; letter-spacing: 1px;")
    return lbl


def _divider():
    line = QFrame()
    line.setFrameShape(QFrame.HLine)
    line.setStyleSheet(f"color: {BORDER_COL};")
    return line


# =========================================================================
# Main Window
# =========================================================================
class MainWindow(QMainWindow):
    def __init__(self, ros_node: GuiRosNode):
        super().__init__()
        self.ros_node    = ros_node
        self.is_executing = False
        self._stop_flash_active = False

        self._build_ui()

        self.ros_node.log_signal.connect(self.append_log)
        self.ros_node.command_finished_signal.connect(self.on_command_finished)
        self.ros_node.nearest_wp_signal.connect(self.on_nearest_wp_received)
        self.ros_node.amcl_pose_signal.connect(self.on_amcl_pose_received)

        # Status timer for UI updates and LED polling
        self._status_timer = QTimer()
        self._status_timer.timeout.connect(self._update_status)
        self._status_timer.start(1000) # 1Hz is fast enough for LED checks

        self._sim_x = 0.0
        self._sim_y = 0.0
        self._sim_h = 0.0
        self._current_wp = "—"

    # -----------------------------------------------------------------
    # UI BUILD
    # -----------------------------------------------------------------
    def _build_ui(self):
        self.setWindowTitle("Dynominion Mission Commander")
        self.resize(1280, 820)
        self.setStyleSheet(GLOBAL_STYLE)

        root = QWidget()
        root_layout = QVBoxLayout(root)
        root_layout.setSpacing(0)
        root_layout.setContentsMargins(0, 0, 0, 0)

        root_layout.addWidget(self._build_topbar())

        body = QWidget()
        body_layout = QHBoxLayout(body)
        body_layout.setContentsMargins(10, 8, 10, 8)
        body_layout.setSpacing(8)
        body_layout.addWidget(self._build_left_panel(), 3)
        body_layout.addWidget(self._build_center_panel(), 3)
        body_layout.addWidget(self._build_right_panel(), 4)
        root_layout.addWidget(body, 1)

        root_layout.addWidget(self._build_bottom_bar())

        self.setCentralWidget(root)
        self.set_execution_state(False)
        self.append_log("[INFO] Dynominion Mission Commander ready.")

    # -----------------------------------------------------------------
    # TOP BAR
    # -----------------------------------------------------------------
    def _build_topbar(self):
        bar = QWidget()
        bar.setFixedHeight(52)
        bar.setStyleSheet(f"""
            background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                stop:0 #0D1117, stop:0.5 #161B22, stop:1 #0D1117);
            border-bottom: 1px solid {BORDER_COL};
        """)
        layout = QHBoxLayout(bar)
        layout.setContentsMargins(16, 0, 16, 0)

        title = QLabel("DYNOMINION MISSION COMMANDER")
        title.setStyleSheet(f"""
            font-size: 16px;
            font-weight: 900;
            letter-spacing: 2px;
            color: {NEON_GREEN};
        """)
        layout.addWidget(title)
        layout.addStretch()

        # LED indicators
        indicators = [
            ("ROS",  "ros_led"),
            ("NAV2", "nav_led"),
            ("AMCL", "amcl_led"),
            ("EXEC", "exec_led"),
        ]
        for label_text, attr_name in indicators:
            col = QVBoxLayout()
            col.setSpacing(2)
            led = LEDIndicator(label_text)
            setattr(self, attr_name, led)
            lbl = QLabel(label_text)
            lbl.setStyleSheet(f"color: {TEXT_DIM}; font-size: 9px; font-weight: bold; letter-spacing:1px;")
            lbl.setAlignment(Qt.AlignCenter)
            col.addWidget(led, alignment=Qt.AlignCenter)
            col.addWidget(lbl)
            row = QWidget()
            row.setLayout(col)
            layout.addWidget(row)
            layout.addSpacing(10)

        self.ros_led.set_color(LEDIndicator.GREEN)
        return bar

    # -----------------------------------------------------------------
    # LEFT PANEL
    # -----------------------------------------------------------------
    def _build_left_panel(self):
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        # --- System Controls ---
        sys_grp = QGroupBox("SYSTEM CONTROLS")
        sys_lay = QVBoxLayout(sys_grp)
        self.btn_start_sim = _make_btn("Start Simulation", "#1565C0")
        self.btn_start_sim.clicked.connect(self.ros_node.launch_simulation)
        self.btn_start_exec = _make_btn("Start Mission Executor", "#6A1B9A")
        self.btn_start_exec.clicked.connect(self.ros_node.launch_executor)
        sys_lay.addWidget(self.btn_start_sim)
        sys_lay.addWidget(self.btn_start_exec)
        layout.addWidget(sys_grp)

        # --- Waypoints ---
        wp_grp = QGroupBox("WAYPOINT NAVIGATION")
        wp_lay = QVBoxLayout(wp_grp)
        btn_grid = QGridLayout()
        btn_grid.setSpacing(6)
        self.btn_A = QPushButton("Target A")
        self.btn_B = QPushButton("Target B")
        self.btn_C = QPushButton("Target C")
        self.btn_D = QPushButton("Target D")
        for btn, wp in [(self.btn_A, 'A'), (self.btn_B, 'B'), (self.btn_C, 'C'), (self.btn_D, 'D')]:
            btn.clicked.connect(lambda _, w=wp: self.add_to_queue(w))
        btn_grid.addWidget(self.btn_A, 0, 0)
        btn_grid.addWidget(self.btn_B, 0, 1)
        btn_grid.addWidget(self.btn_C, 1, 0)
        btn_grid.addWidget(self.btn_D, 1, 1)
        wp_lay.addLayout(btn_grid)
        self.btn_nearest = QPushButton("Nearby point")
        self.btn_nearest.setStyleSheet(f"""
            background-color: #0F2D1A;
            color: {NEON_GREEN};
            border: 1px solid {NEON_GREEN}80;
            border-radius: 8px;
            font-weight: bold;
            padding: 9px;
        """)
        self.btn_nearest.clicked.connect(self.ros_node.trigger_nearest_waypoint)
        wp_lay.addWidget(self.btn_nearest)
        layout.addWidget(wp_grp)

        # --- Maneuvers ---
        mnv_grp = QGroupBox("ROTATION MANEUVER")
        mnv_lay = QVBoxLayout(mnv_grp)

        angle_lay = QHBoxLayout()
        self.selected_angle = 90
        self.angle_btns = {}
        for deg in [90, 180, 270, 360]:
            b = QPushButton(f"{deg}°")
            b.setCheckable(True)
            b.setChecked(deg == 90)
            b.clicked.connect(lambda _, d=deg: self._set_angle(d))
            self.angle_btns[deg] = b
            angle_lay.addWidget(b)
        mnv_lay.addLayout(angle_lay)

        dir_lay = QHBoxLayout()
        self.selected_dir = "CW"
        self.btn_cw = QPushButton("CW")
        self.btn_ccw = QPushButton("CCW")
        self.btn_cw.setCheckable(True)
        self.btn_ccw.setCheckable(True)
        self.btn_cw.setChecked(True)
        self.btn_cw.clicked.connect(lambda: self._set_dir("CW"))
        self.btn_ccw.clicked.connect(lambda: self._set_dir("CCW"))
        dir_lay.addWidget(self.btn_cw)
        dir_lay.addWidget(self.btn_ccw)
        mnv_lay.addLayout(dir_lay)

        self.btn_exec_maneuver = QPushButton("Rotation Maneuver")
        self.btn_exec_maneuver.setStyleSheet(f"""
            background-color: #3D2000;
            color: {WARN_AMB};
            border: 1px solid {WARN_AMB}80;
            border-radius: 8px;
            font-weight: bold;
            padding: 9px;
        """)
        self.btn_exec_maneuver.clicked.connect(self.trigger_custom_maneuver)
        mnv_lay.addWidget(self.btn_exec_maneuver)
        layout.addWidget(mnv_grp)

        # --- Manual Joystick ---
        joy_grp = QGroupBox("JOYSTICK")
        joy_lay = QVBoxLayout(joy_grp)
        self.joystick = JoystickWidget()
        self.joystick.joystickMoved.connect(self.ros_node.publish_joystick_cmd)
        joy_note = QLabel("Locked during automatic execution")
        joy_note.setStyleSheet(f"color: {TEXT_DIM}; font-size: 10px;")
        joy_note.setAlignment(Qt.AlignCenter)
        joy_lay.addWidget(self.joystick, alignment=Qt.AlignCenter)
        joy_lay.addWidget(joy_note)
        layout.addWidget(joy_grp)

        layout.addStretch()
        return panel

    # -----------------------------------------------------------------
    # CENTER PANEL
    # -----------------------------------------------------------------
    def _build_center_panel(self):
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        queue_grp = QGroupBox("MISSION QUEUE")
        queue_lay = QVBoxLayout(queue_grp)

        self.queue_list = QListWidget()
        self.queue_list.setSelectionMode(QAbstractItemView.ExtendedSelection)
        queue_lay.addWidget(self.queue_list)

        ctrl_lay = QHBoxLayout()
        self.btn_remove = QPushButton("Remove Selected")
        self.btn_clear  = QPushButton("Clear All")
        self.btn_remove.clicked.connect(self.remove_selected_queue_items)
        self.btn_clear.clicked.connect(self.queue_list.clear)
        ctrl_lay.addWidget(self.btn_remove)
        ctrl_lay.addWidget(self.btn_clear)
        queue_lay.addLayout(ctrl_lay)
        layout.addWidget(queue_grp, 1)

        exec_grp = QGroupBox("EXECUTION CONTROLS")
        exec_lay = QVBoxLayout(exec_grp)

        self.chk_loop = QCheckBox("Loop Sequence Continuously")
        exec_lay.addWidget(self.chk_loop)

        self.btn_execute = QPushButton("EXECUTE SEQUENCE")
        self.btn_execute.setStyleSheet(f"""
            background-color: #0F3D2A;
            color: {NEON_GREEN};
            border: 2px solid {NEON_GREEN}80;
            border-radius: 8px;
            font-weight: bold;
            font-size: 14px;
            padding: 12px;
        """)
        self.btn_execute.clicked.connect(self.start_execution)
        exec_lay.addWidget(self.btn_execute)

        self.btn_stop = QPushButton("STOP / CANCEL ALL")
        self.btn_stop.setStyleSheet(f"""
            background-color: #3D0F0F;
            color: {DANGER_RED};
            border: 2px solid {DANGER_RED}80;
            border-radius: 8px;
            font-weight: bold;
            font-size: 14px;
            padding: 12px;
        """)
        self.btn_stop.clicked.connect(self.stop_execution)
        exec_lay.addWidget(self.btn_stop)
        layout.addWidget(exec_grp)

        # Current Executing label
        self.lbl_current = QLabel("Active Command: —")
        self.lbl_current.setStyleSheet(f"""
            background-color: {PANEL_BG};
            border: 1px solid {BORDER_COL};
            border-radius: 6px;
            padding: 6px 10px;
            color: {CYAN_HI};
            font-weight: bold;
            font-size: 11px;
        """)
        layout.addWidget(self.lbl_current)

        return panel

    # -----------------------------------------------------------------
    # RIGHT PANEL
    # -----------------------------------------------------------------
    def _build_right_panel(self):
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        log_grp = QGroupBox("MISSION TELEMETRY LOG")
        log_lay = QVBoxLayout(log_grp)
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setFont(QFont("Consolas, Courier New, monospace", 11))
        log_lay.addWidget(self.log_text)
        layout.addWidget(log_grp, 1)

        bottom_row = QHBoxLayout()
        self.btn_report = QPushButton("Generate CSV Report && Exit")
        self.btn_report.setStyleSheet(f"""
            background-color: #1C2128;
            color: {TEXT_LIGHT};
            border: 1px solid {BORDER_COL};
            border-radius: 8px;
            padding: 9px 14px;
        """)
        self.btn_report.clicked.connect(self.close)
        bottom_row.addWidget(self.btn_report)
        layout.addLayout(bottom_row)

        return panel

    # -----------------------------------------------------------------
    # BOTTOM BAR
    # -----------------------------------------------------------------
    def _build_bottom_bar(self):
        bar = QWidget()
        bar.setFixedHeight(38)
        bar.setStyleSheet(f"""
            background-color: #161B22;
            border-top: 1px solid {BORDER_COL};
        """)
        layout = QHBoxLayout(bar)
        layout.setContentsMargins(14, 0, 14, 0)
        layout.setSpacing(18)

        def _stat(prefix, attr):
            lbl = QLabel(f"{prefix} —")
            lbl.setStyleSheet(f"color: {TEXT_DIM}; font-size: 11px; font-family: monospace;")
            setattr(self, attr, lbl)
            return lbl

        layout.addWidget(_stat("XY:", "lbl_xy"))
        layout.addWidget(_stat("HDG:", "lbl_hdg"))
        layout.addWidget(_stat("WP:", "lbl_wp"))

        layout.addStretch()

        # Compass
        self.compass = CompassWidget()
        layout.addWidget(self.compass)

        return bar

    # -----------------------------------------------------------------
    # Status properties and loops
    # -----------------------------------------------------------------
    def _update_status(self):
        # 1. Update status bar text
        self.lbl_xy.setText(f"XY:  ({self._sim_x:.2f}, {self._sim_y:.2f})")
        self.lbl_hdg.setText(f"HDG: {self._sim_h:.1f}°")
        self.lbl_wp.setText(f"WP:  {self._current_wp}")
        self.compass.set_heading(self._sim_h)
        
        # 2. Check AMCL running status (publishers exist on topic)
        publishers = self.ros_node.count_publishers('/amcl_pose')
        if publishers > 0:
            self.amcl_led.set_color(LEDIndicator.GREEN)
        else:
            self.amcl_led.set_color(LEDIndicator.GRAY)
            
        # 3. Check Nav2 running status (Action server exists)
        # We can simply check if the action server for our executor is up, 
        # or if standard nav2 topics exist. Let's check generic nav2 topic:
        nav_clients = self.ros_node.count_subscribers('/goal_pose')
        if nav_clients > 0:
            self.nav_led.set_color(LEDIndicator.GREEN)
        else:
            self.nav_led.set_color(LEDIndicator.GRAY)

    def on_amcl_pose_received(self, x, y, yaw):
        self._sim_x = x
        self._sim_y = y
        self._sim_h = yaw
        # Text update will be handled by the 1Hz _update_status loop now

    # -----------------------------------------------------------------
    # Angle / Direction helpers
    # -----------------------------------------------------------------
    def _set_angle(self, deg):
        self.selected_angle = deg
        for d, b in self.angle_btns.items():
            b.setChecked(d == deg)

    def _set_dir(self, direction):
        self.selected_dir = direction
        self.btn_cw.setChecked(direction == "CW")
        self.btn_ccw.setChecked(direction == "CCW")

    def trigger_custom_maneuver(self):
        cmd = f"MANEUVER_ROT_{self.selected_angle}_{self.selected_dir}"
        self.add_to_queue(cmd)
        if not self.is_executing:
            self.start_execution()

    # -----------------------------------------------------------------
    # Colored Log
    # -----------------------------------------------------------------
    def append_log(self, text):
        ts = datetime.now().strftime("%H:%M:%S")

        if "[SUCCESS]" in text:
            color = NEON_GREEN
        elif "[ERROR]" in text:
            color = DANGER_RED
        elif "[WARNING]" in text:
            color = WARN_AMB
        elif "[ENGINE]" in text:
            color = CYAN_HI
        elif "[NEAR" in text:
            color = "#BB86FC"
        else:
            color = TEXT_LIGHT

        html = (f'<span style="color:{TEXT_DIM};font-size:10px;">[{ts}]</span> '
                f'<span style="color:{color};">{text}</span>')
        self.log_text.append(html)
        sb = self.log_text.verticalScrollBar()
        sb.setValue(sb.maximum())

    # -----------------------------------------------------------------
    # Nearest WP callback
    # -----------------------------------------------------------------
    def on_nearest_wp_received(self, success, waypoint_name):
        if not success:
            self.append_log(f"[ERROR] Nearest WP failed: {waypoint_name}")
            return
        self.append_log(f"[INFO] Nearest waypoint is '{waypoint_name}'.")
        if not self.is_executing:
            self.append_log("[ENGINE] Robot idle – auto-executing nearest waypoint.")
            self.queue_list.clear()
            self.add_to_queue(waypoint_name)
            self.start_execution()
        else:
            self.append_log("[ENGINE] Mission running – appending nearest to queue.")
            self.add_to_queue(waypoint_name)

    # -----------------------------------------------------------------
    # Queue Management
    # -----------------------------------------------------------------
    def add_to_queue(self, cmd_name):
        self.queue_list.addItem(cmd_name)

    def remove_selected_queue_items(self):
        for item in self.queue_list.selectedItems():
            self.queue_list.takeItem(self.queue_list.row(item))

    # -----------------------------------------------------------------
    # Execution State
    # -----------------------------------------------------------------
    def set_execution_state(self, is_running):
        self.is_executing = is_running
        self.btn_execute.setEnabled(not is_running)
        self.chk_loop.setEnabled(not is_running)
        self.joystick.setEnabled(not is_running)
        self.btn_nearest.setEnabled(not is_running)
        self.btn_exec_maneuver.setEnabled(not is_running)
        for b in self.angle_btns.values():
            b.setEnabled(not is_running)
        self.btn_cw.setEnabled(not is_running)
        self.btn_ccw.setEnabled(not is_running)
        self.joystick.update()
        if not is_running:
            self.lbl_current.setText("Active Command: —")
            self._current_wp = "—"
        # Flash exec LED
        self.exec_led.set_color(LEDIndicator.GREEN if is_running else LEDIndicator.GRAY)

    def start_execution(self):
        if self.is_executing: return
        if self.queue_list.count() == 0:
            self.append_log("[WARNING] Queue is empty. Add commands first.")
            return
        self.append_log("[ENGINE] Starting sequence – Joystick LOCKED.")
        self.set_execution_state(True)
        self.execute_next_in_queue()

    def stop_execution(self):
        self.append_log("[ENGINE] Stop triggered! Clearing queue. Joystick UNLOCKED.")
        self.queue_list.clear()
        self.ros_node.trigger_stop()
        self.set_execution_state(False)
        self._flash_stop_border()

    def _flash_stop_border(self):
        self.setStyleSheet(GLOBAL_STYLE + f"QMainWindow {{ border: 3px solid {DANGER_RED}; }}")
        QTimer.singleShot(800, lambda: self.setStyleSheet(GLOBAL_STYLE))

    def execute_next_in_queue(self):
        if not self.is_executing: return
        if self.queue_list.count() == 0:
            self.append_log("[ENGINE] Sequence complete. Joystick UNLOCKED.")
            self.set_execution_state(False)
            return

        item = self.queue_list.takeItem(0)
        cmd_name = item.text()
        self.lbl_current.setText(f"Active Command:  {cmd_name}")
        self._current_wp = cmd_name
        self.append_log(f"\n[ENGINE] Firing command: {cmd_name}")
        self.ros_node.execute_command(cmd_name)

        if self.chk_loop.isChecked():
            self.add_to_queue(cmd_name)

    def on_command_finished(self, success, cmd_name, duration):
        if not self.is_executing: return
        if success:
            QTimer.singleShot(400, self.execute_next_in_queue)
        else:
            self.append_log(f"[ENGINE] Halted on failure: {cmd_name}. Joystick UNLOCKED.")
            self.set_execution_state(False)

    def closeEvent(self, event):
        self.ros_node.shutdown_system()
        event.accept()


# =========================================================================
# ROS2 spin thread
# =========================================================================
def ros_spin_thread(node):
    rclpy.spin(node)


def main(args=None):
    rclpy.init(args=args)

    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    ros_node  = GuiRosNode()
    spin_thread = threading.Thread(target=ros_spin_thread, args=(ros_node,), daemon=True)
    spin_thread.start()

    window = MainWindow(ros_node)
    window.show()

    # Handle Ctrl+C gracefully
    signal.signal(signal.SIGINT, lambda sig, frame: app.quit())

    # Keep Python interpreter responsive to signals
    timer = QTimer()
    timer.start(500)
    timer.timeout.connect(lambda: None)

    app.exec_()


if __name__ == '__main__':
    main()

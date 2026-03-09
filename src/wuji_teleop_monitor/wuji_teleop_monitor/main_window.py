"""
Main window for teleop monitor.
"""

from __future__ import annotations

import os
import signal
import subprocess
import sys
import threading
import time
import gc
from typing import Optional

import rclpy
from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QFrame, QPushButton, QComboBox, QMessageBox
)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QFont

from .ros_node import TeleopMonitorNode
from .widgets import DeviceStatusWidget, DataFlowWidget, NodeStatusWidget, CameraStatusWidget, StereoHeadStatusWidget
from .logger import get_logger

# Setup module logger
logger = get_logger("teleop_monitor.main_window")


# Launch configurations
LAUNCH_CONFIGS = {
    "仅手部 (Manus+wuji)": ["wuji_teleop_hand.launch.py", "hand_input:=manus"],
    "仅机械臂 (Tracker+tianji)": ["wuji_teleop_arm.launch.py", "arm_input:=tracker"],
    "手+臂-完整(Manus&Tracker+wuji&tianji)": ["wuji_teleop.launch.py", "hand_input:=manus", "arm_input:=tracker"],
    "手+臂+相机-完整(Manus&Tracker+wuji&tianji)": ["wuji_teleop_camera.launch.py", "hand_input:=manus", "arm_input:=tracker"],
    "手+臂-单侧(左,Manus&Tracker+wuji&tianji)": ["wuji_teleop_single.launch.py", "side:=left", "hand_input:=manus", "arm_input:=tracker"],
    "手+臂-单侧(右,Manus&Tracker+wuji&tianji)": ["wuji_teleop_single.launch.py", "side:=right", "hand_input:=manus", "arm_input:=tracker"],
}


class MonitorWindow(QMainWindow):
    """Main window for the teleop monitor."""

    def __init__(self, ros_node: TeleopMonitorNode):
        super().__init__()
        logger.info("初始化主窗口")
        self.ros_node = ros_node
        self.launch_process: Optional[subprocess.Popen] = None

        # 机器人连接（用于松闸/抱闸）
        self._robot = None
        self._dcss = None
        self._robot_connected = False
        self._robot_ever_connected = False  # 标记是否曾连接过机器人

        # 后台扫描线程
        self._scanning = True
        self._scan_thread = threading.Thread(target=self._scan_worker, daemon=True)

        logger.debug("设置 UI 界面")
        self._setup_ui()

        logger.debug("设置定时器")
        self._setup_timer()

        logger.debug("启动后台扫描线程")
        self._scan_thread.start()
        logger.info("主窗口初始化完成")

        # 启动时检查是否有残留的 ROS2 节点
        self._check_existing_nodes_on_startup()

    def _setup_ui(self):
        self.setWindowTitle("Teleop Launcher")
        self.setMinimumSize(700, 520)
        self.resize(900, 820)

        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout()
        main_layout.setSpacing(8)
        main_layout.setContentsMargins(12, 12, 12, 12)

        # ===== Device Status Section =====
        device_layout = QHBoxLayout()

        # Manus Gloves
        manus_group = QWidget()
        manus_layout = QVBoxLayout()
        manus_layout.setSpacing(4)
        manus_title = QLabel("Manus 手套")
        manus_title.setFont(QFont("Arial", 11, QFont.Bold))
        manus_layout.addWidget(manus_title)

        manus_devices = QHBoxLayout()
        self.manus_left = DeviceStatusWidget("左手")
        self.manus_right = DeviceStatusWidget("右手")
        manus_devices.addWidget(self.manus_left)
        manus_devices.addWidget(self.manus_right)
        manus_layout.addLayout(manus_devices)
        manus_group.setLayout(manus_layout)

        # Wuji Hands
        hand_group = QWidget()
        hand_layout = QVBoxLayout()
        hand_layout.setSpacing(4)
        hand_title = QLabel("Wuji 灵巧手")
        hand_title.setFont(QFont("Arial", 11, QFont.Bold))
        hand_layout.addWidget(hand_title)

        hand_devices = QHBoxLayout()
        self.hand_left = DeviceStatusWidget("左手")
        self.hand_right = DeviceStatusWidget("右手")
        hand_devices.addWidget(self.hand_left)
        hand_devices.addWidget(self.hand_right)
        hand_layout.addLayout(hand_devices)
        hand_group.setLayout(hand_layout)

        # Tianji Arm
        tianji_group = QWidget()
        tianji_layout = QVBoxLayout()
        tianji_layout.setSpacing(4)
        tianji_title = QLabel("tianji机械臂")
        tianji_title.setFont(QFont("Arial", 11, QFont.Bold))
        tianji_layout.addWidget(tianji_title)

        self.tianji_arm = DeviceStatusWidget("双手")
        tianji_layout.addWidget(self.tianji_arm)

        # 松闸/抱闸按钮
        brake_layout = QHBoxLayout()
        brake_layout.setSpacing(4)

        # 连接按钮
        self.btn_connect_robot = QPushButton("连接")
        self.btn_connect_robot.setFixedSize(50, 28)
        self.btn_connect_robot.setStyleSheet("background-color: #2196F3; color: white;")
        self.btn_connect_robot.clicked.connect(self._toggle_robot_connection)
        brake_layout.addWidget(self.btn_connect_robot)

        brake_layout.addSpacing(10)

        # 左臂 (A臂)
        left_label = QLabel("左臂:")
        brake_layout.addWidget(left_label)
        self.btn_left_release = QPushButton("松闸")
        self.btn_left_release.setFixedSize(50, 28)
        self.btn_left_release.setStyleSheet("background-color: #ff9800; color: white;")
        self.btn_left_release.clicked.connect(lambda: self._on_release_brake('A'))
        self.btn_left_release.setEnabled(False)
        brake_layout.addWidget(self.btn_left_release)
        self.btn_left_brake = QPushButton("抱闸")
        self.btn_left_brake.setFixedSize(50, 28)
        self.btn_left_brake.clicked.connect(lambda: self._on_brake('A'))
        self.btn_left_brake.setEnabled(False)
        brake_layout.addWidget(self.btn_left_brake)

        brake_layout.addSpacing(10)

        # 右臂 (B臂)
        right_label = QLabel("右臂:")
        brake_layout.addWidget(right_label)
        self.btn_right_release = QPushButton("松闸")
        self.btn_right_release.setFixedSize(50, 28)
        self.btn_right_release.setStyleSheet("background-color: #ff9800; color: white;")
        self.btn_right_release.clicked.connect(lambda: self._on_release_brake('B'))
        self.btn_right_release.setEnabled(False)
        brake_layout.addWidget(self.btn_right_release)
        self.btn_right_brake = QPushButton("抱闸")
        self.btn_right_brake.setFixedSize(50, 28)
        self.btn_right_brake.clicked.connect(lambda: self._on_brake('B'))
        self.btn_right_brake.setEnabled(False)
        brake_layout.addWidget(self.btn_right_brake)

        brake_layout.addStretch()
        tianji_layout.addLayout(brake_layout)

        # 关节角度显示
        self.joint_label = QLabel("关节角度: 未连接")
        self.joint_label.setStyleSheet("color: #666; font-size: 10px;")
        tianji_layout.addWidget(self.joint_label)

        tianji_group.setLayout(tianji_layout)
        tianji_group.setMinimumHeight(120)

        # Vive Tracker - 禁用以避免 OpenVR 崩溃问题
        # vive_group = QWidget()
        # vive_layout = QVBoxLayout()
        # vive_layout.setSpacing(4)
        # vive_title = QLabel("Vive Tracker")
        # vive_title.setFont(QFont("Arial", 11, QFont.Bold))
        # vive_layout.addWidget(vive_title)
        #
        # vive_devices = QHBoxLayout()
        # self.tracker_widget = DeviceStatusWidget("Tracker")
        # self.basestation_widget = DeviceStatusWidget("基站")
        # vive_devices.addWidget(self.tracker_widget)
        # vive_devices.addWidget(self.basestation_widget)
        # vive_layout.addLayout(vive_devices)
        # vive_group.setLayout(vive_layout)
        # vive_group.setMinimumHeight(120)

        device_layout.addWidget(manus_group)
        device_layout.addWidget(hand_group)
        main_layout.addLayout(device_layout)

        # Tianji Arm (second row) - Vive Tracker 已禁用
        device_layout2 = QHBoxLayout()
        # device_layout2.addWidget(vive_group)  # 已禁用
        device_layout2.addWidget(tianji_group)
        main_layout.addLayout(device_layout2)

        # Separator
        self._add_separator(main_layout)

        # ===== Topic Status & Node Status (same row) =====
        status_row = QHBoxLayout()
        self.data_flow = DataFlowWidget()
        self.camera_status = CameraStatusWidget()
        self.node_status = NodeStatusWidget()
        status_row.addWidget(self.data_flow)
        status_row.addWidget(self.camera_status)
        status_row.addWidget(self.node_status)
        main_layout.addLayout(status_row)

        # ===== Stereo Head Camera Status Section =====
        self.stereo_head_status = StereoHeadStatusWidget()
        main_layout.addWidget(self.stereo_head_status)

        # Separator
        self._add_separator(main_layout)

        # ===== Launch Section (dropdown + button) =====
        launch_layout = QHBoxLayout()
        launch_layout.setContentsMargins(0, 20, 0, 20)  
        launch_layout.addStretch()

        # Label
        launch_label = QLabel("启动选项:")
        launch_label.setFont(QFont("Arial", 12))
        launch_layout.addWidget(launch_label)

        # Dropdown menu
        self.launch_combo = QComboBox()
        self.launch_combo.setFixedWidth(320)
        self.launch_combo.setFixedHeight(55)
        self.launch_combo.setFont(QFont("Arial", 16))
        self.launch_combo.setStyleSheet("""
            QComboBox { padding: 10px; }
            QComboBox QAbstractItemView {
                font-size: 16px;
            }
            QComboBox QAbstractItemView::item {
                min-height: 50px;
                padding: 10px;
            }
        """)
        for config_name in LAUNCH_CONFIGS.keys():
            self.launch_combo.addItem(config_name)
        launch_layout.addWidget(self.launch_combo)

        # Start/Stop button
        self.launch_btn = QPushButton("启动遥操作")
        self.launch_btn.setFixedSize(160, 55)
        self.launch_btn.setFont(QFont("Arial", 16, QFont.Bold))
        self.launch_btn.setStyleSheet(
            "QPushButton { background-color: #4CAF50; color: white; border-radius: 4px; border: none; }"
            "QPushButton:hover { background-color: #45a049; }"
        )
        self.launch_btn.clicked.connect(self._toggle_launch)
        launch_layout.addWidget(self.launch_btn)

        # Clean nodes button
        self.clean_nodes_btn = QPushButton("清理节点")
        self.clean_nodes_btn.setFixedSize(120, 55)
        self.clean_nodes_btn.setFont(QFont("Arial", 14, QFont.Bold))
        self.clean_nodes_btn.setStyleSheet(
            "QPushButton { background-color: #FF9800; color: white; border-radius: 4px; border: none; }"
            "QPushButton:hover { background-color: #F57C00; }"
        )
        self.clean_nodes_btn.clicked.connect(self._clean_ros_nodes)
        launch_layout.addWidget(self.clean_nodes_btn)

        launch_layout.addStretch()
        main_layout.addLayout(launch_layout)

        # Separator
        self._add_separator(main_layout)

        # Status bar
        self.status_label = QLabel("监听中...")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet("color: #666666;")
        main_layout.addWidget(self.status_label)

        central.setLayout(main_layout)

    def _add_separator(self, layout):
        """Add a horizontal separator line."""
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setStyleSheet("color: #cccccc;")
        layout.addWidget(line)

    def _setup_timer(self):
        self.timer = QTimer()
        self.timer.timeout.connect(self._update_ui)
        self.timer.start(100)  # 10Hz refresh

    def _scan_worker(self):
        """后台线程：执行耗时扫描操作"""
        logger.info("后台扫描线程已启动")
        scan_count = 0
        while self._scanning:
            scan_count += 1
            logger.debug(f"开始第 {scan_count} 次设备扫描")
            try:
                logger.debug("扫描 Manus 手套 USB 设备")
                self.ros_node.update_glove_status()    # Manus USB scan

                logger.debug("扫描 Wuji 灵巧手 USB 设备")
                self.ros_node.update_hand_status()     # Wuji hand USB scan

                logger.debug("扫描天机机械臂网络状态 (ping)")
                self.ros_node.update_tianji_status()   # Tianji ping

                # 禁用 OpenVR 扫描以避免崩溃
                # logger.debug("扫描 Vive Tracker 和基站状态")
                # self.ros_node.update_vive_status()     # Vive Tracker + base station

                logger.debug("扫描 ROS2 节点状态")
                self.ros_node.update_node_status()     # ros2 node list

                logger.debug("扫描头部双目相机硬件状态")
                self.ros_node.update_stereo_head_hardware_status()  # Stereo head hardware

                logger.debug(f"第 {scan_count} 次设备扫描完成")
            except Exception as e:
                logger.error(f"设备扫描异常 (第 {scan_count} 次): {e}", exc_info=True)
            time.sleep(2.0)  # 扫描间隔
        logger.info("后台扫描线程已退出")

    def _update_ui(self):
        # Spin ROS node
        try:
            rclpy.spin_once(self.ros_node, timeout_sec=0)
        except Exception as e:
            logger.error(f"ROS2 spin_once 异常: {e}", exc_info=True)
            return

        # Update topic status (lightweight, check timestamps only)
        try:
            self.ros_node.update_hand_input_status()
            self.ros_node.update_glove_topic_status()
            self.ros_node.update_camera_status()
            self.ros_node.update_stereo_head_status()
        except Exception as e:
            logger.error(f"更新话题状态异常: {e}", exc_info=True)

        # Update UI components (read data from ros_node, updated by background thread)
        try:
            self._update_data_flow()
            self._update_camera_widget()
            self._update_stereo_head_widget()
            self._update_manus_widgets()
            self._update_hand_widgets()
            self._update_tianji_widget()
            # self._update_vive_widget()  # 禁用 Vive Tracker 监视
            self._update_node_status()
            self._update_status_bar()
            self._check_launch_status()
        except Exception as e:
            logger.error(f"更新 UI 组件异常: {e}", exc_info=True)

        # 实时更新关节角度
        if self._robot_connected:
            try:
                self._update_joint_display()
            except Exception as e:
                logger.error(f"更新关节角度显示异常: {e}", exc_info=True)

    def _update_data_flow(self):
        """Update data flow widget."""
        # /manus_glove_0
        self.data_flow.glove_0.set_active(self.ros_node.glove_0_connected)
        self.data_flow.glove_0.set_info(f"{self.ros_node.glove_0_msg_count} msgs")

        # /manus_glove_1
        self.data_flow.glove_1.set_active(self.ros_node.glove_1_connected)
        self.data_flow.glove_1.set_info(f"{self.ros_node.glove_1_msg_count} msgs")

        # /hand_input
        self.data_flow.hand_input.set_active(self.ros_node.hand_input_connected)
        info = self.ros_node.get_hand_input_info()
        self.data_flow.hand_input.set_info(f"{self.ros_node.hand_input_msg_count} ({info})")

    def _update_camera_widget(self):
        """Update camera status widget."""
        cam_status = self.ros_node.get_camera_status()

        # Head camera
        head = cam_status["head"]
        self.camera_status.cam_head.set_active(head["connected"])
        self.camera_status.cam_head.set_info(f"{head['msg_count']} frames")

        # Left wrist camera
        left = cam_status["left_wrist"]
        self.camera_status.cam_left.set_active(left["connected"])
        self.camera_status.cam_left.set_info(f"{left['msg_count']} frames")

        # Right wrist camera
        right = cam_status["right_wrist"]
        self.camera_status.cam_right.set_active(right["connected"])
        self.camera_status.cam_right.set_info(f"{right['msg_count']} frames")

    def _update_stereo_head_widget(self):
        """Update stereo head camera status widget."""
        # 更新硬件状态
        hw_info = self.ros_node.get_stereo_head_hardware_info()
        camera_info = hw_info["camera_info"]
        loopback_info = hw_info["loopback_info"]

        self.stereo_head_status.set_camera_available(
            camera_info["available"],
            camera_info.get("info", "")
        )
        self.stereo_head_status.set_loopback_available(
            loopback_info["available"],
            loopback_info.get("info", "")
        )

        # 更新话题状态
        stereo_status = self.ros_node.get_stereo_head_status()
        left = stereo_status["left"]
        right = stereo_status["right"]
        self.stereo_head_status.set_topic_status(
            left["connected"], left["msg_count"],
            right["connected"], right["msg_count"]
        )

    def _update_manus_widgets(self):
        """Update Manus glove display based on USB scan."""
        serials_list = list(self.ros_node.manus_glove_serials)

        # Left hand: show first connected device
        if serials_list:
            self.manus_left.set_connected(True)
            self.manus_left.set_id(f"SN: {serials_list[0]}")
            self.manus_left.set_info("")
        else:
            self.manus_left.set_connected(False)
            self.manus_left.set_id("SN: --")
            self.manus_left.set_info("")

        # Right hand: show second connected device if exists
        if len(serials_list) > 1:
            self.manus_right.set_connected(True)
            self.manus_right.set_id(f"SN: {serials_list[1]}")
            self.manus_right.set_info("")
        else:
            self.manus_right.set_connected(False)
            self.manus_right.set_id("SN: --")
            self.manus_right.set_info("")

    def _update_hand_widgets(self):
        """Update Wuji hand display."""
        connected_serials = self.ros_node.get_connected_hands()
        serials_list = list(connected_serials)

        # Left hand: show first connected device
        if serials_list:
            self.hand_left.set_connected(True)
            self.hand_left.set_id(f"SN: {serials_list[0]}")
        else:
            self.hand_left.set_connected(False)
            self.hand_left.set_id("SN: --")

        # Right hand: show second connected device if exists
        if len(serials_list) > 1:
            self.hand_right.set_connected(True)
            self.hand_right.set_id(f"SN: {serials_list[1]}")
        else:
            self.hand_right.set_connected(False)
            self.hand_right.set_id("SN: --")

    def _update_tianji_widget(self):
        """Update Tianji arm display."""
        if self.ros_node.tianji_arm_reachable:
            self.tianji_arm.set_connected(True)
            self.tianji_arm.set_id(f"IP: {self.ros_node.get_tianji_ip()}")
        else:
            self.tianji_arm.set_connected(False)
            self.tianji_arm.set_id("IP: --")

    def _update_vive_widget(self):
        """Update Vive Tracker display."""
        # Update Tracker widget - 显示各部位连接状态
        from .scanner import ViveTrackerScanner
        tracker_config = ViveTrackerScanner.get_tracker_config()
        connected_serials = self.ros_node.vive_tracker_serials
        body_names = ViveTrackerScanner.BODY_PART_NAMES

        if tracker_config:
            # 检查每个部位的连接状态
            status_parts = []
            connected_count = 0
            for part, serial in tracker_config.items():
                name = body_names.get(part, part)
                if serial in connected_serials:
                    status_parts.append(f'<span style="color: #00aa00;">● {name}</span>')
                    connected_count += 1
                else:
                    status_parts.append(f'<span style="color: #cc0000;">○ {name}</span>')

            all_connected = (connected_count == len(tracker_config))
            self.tracker_widget.set_connected(all_connected)
            self.tracker_widget.set_id(f"{connected_count}/{len(tracker_config)}")
            # 使用 HTML 格式显示带颜色的状态
            self.tracker_widget.info_label.setText("<br>".join(status_parts))
        else:
            # 没有配置，显示原来的数量
            tracker_count = len(connected_serials)
            if tracker_count > 0:
                self.tracker_widget.set_connected(True)
                self.tracker_widget.set_id(f"{tracker_count} 个")
                self.tracker_widget.set_info("")
            else:
                self.tracker_widget.set_connected(False)
                self.tracker_widget.set_id("无设备")
                self.tracker_widget.set_info("")

        # Update Base Station widget
        base_stations = self.ros_node.base_stations
        station_count = len(base_stations)
        connected_count = sum(1 for s in base_stations.values() if s.get('connected', False))
        if station_count > 0:
            all_connected = (connected_count == station_count)
            self.basestation_widget.set_connected(all_connected)
            self.basestation_widget.set_id(f"{connected_count}/{station_count} 个")
            # Show modes (A/B/C)
            modes = [s.get('mode', '?') for s in base_stations.values()]
            self.basestation_widget.set_info(" ".join(modes))
        else:
            self.basestation_widget.set_connected(False)
            self.basestation_widget.set_id("无基站")
            self.basestation_widget.set_info("")

    def _update_node_status(self):
        """Update node status widget."""
        node_status = self.ros_node.get_monitored_nodes_status()
        self.node_status.update_nodes(node_status)

    def _update_status_bar(self):
        """Update status bar text."""
        # Count connected devices
        manus_count = len(self.ros_node.manus_glove_serials)
        hand_count = len(self.ros_node.get_connected_hands())
        tianji_connected = self.ros_node.tianji_arm_reachable
        tracker_count = len(self.ros_node.vive_tracker_serials)
        node_count = sum(self.ros_node.get_monitored_nodes_status().values())

        # Count connected cameras
        cam_status = self.ros_node.get_camera_status()
        cam_count = sum(1 for s in cam_status.values() if s["connected"])

        parts = []
        if manus_count > 0:
            parts.append(f"手套: {manus_count}")
        if hand_count > 0:
            parts.append(f"灵巧手: {hand_count}")
        if tianji_connected:
            parts.append("天机臂: 在线")
        if tracker_count > 0:
            parts.append(f"Tracker: {tracker_count}")
        if cam_count > 0:
            parts.append(f"相机: {cam_count}/3")
        if node_count > 0:
            parts.append(f"节点: {node_count}/5")

        if parts:
            self.status_label.setText(" | ".join(parts))
        else:
            self.status_label.setText("等待设备连接...")

    def moveEvent(self, event):
        """窗口移动时关闭下拉菜单"""
        self.launch_combo.hidePopup()
        super().moveEvent(event)

    def closeEvent(self, event):
        """窗口关闭事件，确保正确清理所有资源和进程"""
        logger.info("窗口关闭事件触发，开始清理资源...")
        self.stop()  # 调用 stop 方法清理所有资源
        event.accept()  # 接受关闭事件
        logger.info("窗口关闭事件处理完成")

    def stop(self):
        """Stop the timer, background thread, and launch process."""
        # 防止重复调用
        if hasattr(self, '_stopped') and self._stopped:
            logger.warning("stop() 方法被重复调用，忽略")
            return
        self._stopped = True
        logger.info("开始执行清理操作")

        # 停止定时器
        if hasattr(self, 'timer'):
            logger.debug("停止 UI 更新定时器")
            self.timer.stop()
            logger.debug("UI 更新定时器已停止")

        # 停止后台扫描线程
        logger.debug("准备停止后台扫描线程")
        self._scanning = False
        if hasattr(self, '_scan_thread') and self._scan_thread.is_alive():
            logger.debug("等待后台扫描线程结束 (超时 2 秒)")
            self._scan_thread.join(timeout=2.0)  # 等待线程结束，最多2秒
            if self._scan_thread.is_alive():
                logger.warning("后台扫描线程在 2 秒后仍未结束")
            else:
                logger.debug("后台扫描线程已正常结束")

        # 停止启动的进程
        logger.debug("准备停止 launch 进程")
        self._stop_launch()

        # 释放机器人连接
        logger.debug("准备断开机器人连接")
        self._disconnect_robot()

        logger.info("所有资源清理完成")

    def _toggle_launch(self):
        """Toggle launch process start/stop."""
        if self.launch_process is not None and self.launch_process.poll() is None:
            # Process is running, stop it
            self._stop_launch()
        else:
            # Process not running, start it
            self._start_launch()

    def _start_launch(self):
        """Start the teleop launch file based on selected configuration."""
        logger.info("尝试启动遥操作系统")

        # 如果曾连接过机器人，提示用户需要重启
        if self._robot_ever_connected:
            logger.warning("检测到本次会话曾连接过机器人，需要重启才能启动遥操作")
            reply = QMessageBox.question(
                self, "需要重启",
                "检测到本次会话中曾连接过机器人。\n"
                "由于 SDK 端口占用问题，需要重启 Monitor 才能启动遥操作。\n\n"
                "是否立即重启？",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.Yes
            )
            if reply == QMessageBox.Yes:
                logger.info("用户选择重启应用程序")
                self._restart_application()
            else:
                logger.info("用户取消重启")
            return

        try:
            config_name = self.launch_combo.currentText()
            config_args = LAUNCH_CONFIGS.get(config_name, [])
            if not config_args:
                logger.error(f"未找到配置: {config_name}")
                return

            launch_file = config_args[0]
            extra_args = config_args[1:]

            cmd = ["ros2", "launch", "wuji_teleop_bringup", launch_file] + extra_args
            logger.info(f"启动配置: {config_name}")
            logger.debug(f"执行命令: {' '.join(cmd)}")

            self.launch_process = subprocess.Popen(
                cmd,
                env=os.environ.copy(),
                preexec_fn=os.setsid  # 创建新进程组
            )
            logger.info(f"Launch 进程已启动，PID: {self.launch_process.pid}")

            self.launch_btn.setText("停止")
            self.launch_combo.setEnabled(False)  # 禁用下拉菜单
            self.btn_connect_robot.setEnabled(False)  # 禁用连接按钮
            self.launch_btn.setStyleSheet(
                "QPushButton { background-color: #f44336; color: white; border-radius: 4px; border: none; }"
                "QPushButton:hover { background-color: #da190b; }"
            )
        except Exception as e:
            logger.error(f"启动 launch 失败: {e}", exc_info=True)

    def _stop_launch(self):
        """Stop the teleop launch process."""
        if self.launch_process is not None:
            logger.info(f"准备停止 launch 进程，PID: {self.launch_process.pid}")
            try:
                pgid = os.getpgid(self.launch_process.pid)
                logger.debug(f"进程组 ID: {pgid}")

                # 先发送 SIGINT
                logger.debug("发送 SIGINT 信号")
                os.killpg(pgid, signal.SIGINT)
                try:
                    self.launch_process.wait(timeout=3)
                    logger.info("进程通过 SIGINT 正常退出")
                except subprocess.TimeoutExpired:
                    # SIGINT 超时，发送 SIGTERM
                    logger.warning("SIGINT 超时 (3秒)，发送 SIGTERM 信号")
                    os.killpg(pgid, signal.SIGTERM)
                    try:
                        self.launch_process.wait(timeout=2)
                        logger.info("进程通过 SIGTERM 退出")
                    except subprocess.TimeoutExpired:
                        # SIGTERM 也超时，强制 SIGKILL
                        logger.warning("SIGTERM 超时 (2秒)，发送 SIGKILL 信号")
                        os.killpg(pgid, signal.SIGKILL)
                        self.launch_process.wait(timeout=2)
                        logger.info("进程通过 SIGKILL 强制退出")
            except ProcessLookupError:
                # 进程已经退出
                logger.debug("进程已经退出 (ProcessLookupError)")
            except Exception as e:
                logger.error(f"停止 launch 进程异常: {e}", exc_info=True)
                # 尝试直接杀死进程
                try:
                    logger.debug("尝试直接杀死进程")
                    self.launch_process.kill()
                    self.launch_process.wait(timeout=2)
                    logger.info("进程已通过 kill() 强制退出")
                except Exception as e2:
                    logger.error(f"强制杀死进程失败: {e2}", exc_info=True)
            self.launch_process = None
            logger.info("Launch 进程已停止")
        else:
            logger.debug("没有运行中的 launch 进程")

        self.launch_btn.setText("启动遥操作")
        self.launch_combo.setEnabled(True)  # 重新启用下拉菜单
        self.btn_connect_robot.setEnabled(True)  # 重新启用连接按钮
        self.launch_btn.setStyleSheet(
            "QPushButton { background-color: #4CAF50; color: white; border-radius: 4px; border: none; }"
            "QPushButton:hover { background-color: #45a049; }"
        )

    def _check_launch_status(self):
        """Check if launch process is still running."""
        if self.launch_process is not None and self.launch_process.poll() is not None:
            # Process has exited
            exit_code = self.launch_process.returncode
            logger.warning(f"Launch 进程意外退出，退出码: {exit_code}")
            self.launch_process = None
            self.launch_btn.setText("启动遥操作")
            self.launch_combo.setEnabled(True)  # 重新启用下拉菜单
            self.btn_connect_robot.setEnabled(True)  # 重新启用连接按钮
            self.launch_btn.setStyleSheet(
                "QPushButton { background-color: #4CAF50; color: white; border-radius: 4px; border: none; }"
                "QPushButton:hover { background-color: #45a049; }"
            )

    def _toggle_robot_connection(self):
        """切换机器人连接状态"""
        if self._robot_connected:
            self._disconnect_robot()
        else:
            self._connect_robot()

    def _connect_robot(self):
        """连接机器人"""
        logger.info("尝试连接机器人")

        if not self.ros_node.tianji_arm_reachable:
            logger.warning("机器人不可达，无法连接")
            QMessageBox.warning(self, "警告", "机器人不可达！请检查网络连接。")
            return

        try:
            logger.debug("导入 tianji_output 模块")
            from tianji_output import Marvin_Robot
            from tianji_output._internal import DCSS
            robot_ip = self.ros_node.get_tianji_ip()
            logger.debug(f"机器人 IP: {robot_ip}")

            # 每次连接都创建新实例
            logger.debug("创建 Marvin_Robot 实例")
            self._robot = Marvin_Robot()
            self._dcss = DCSS()

            logger.debug("正在连接机器人...")
            result = self._robot.connect(robot_ip)
            # SDK返回值: 1=成功, 0=失败
            if result == 1:
                self._robot_connected = True
                self._robot_ever_connected = True  # 标记曾连接过
                logger.info(f"机器人连接成功: {robot_ip}")
                self._update_brake_buttons_state(True)
                self.btn_connect_robot.setText("断开")
                self.btn_connect_robot.setStyleSheet("background-color: #f44336; color: white;")
                # 显示关节角度
                self._update_joint_display()
            else:
                self._robot_connected = False
                logger.error(f"机器人连接失败: {robot_ip}, SDK 返回错误码: {result}")
                QMessageBox.warning(self, "警告", f"机器人连接失败！错误码: {result}")
        except ImportError as e:
            self._robot_connected = False
            logger.error(f"导入 tianji_output 模块失败: {e}", exc_info=True)
            QMessageBox.critical(self, "错误", f"导入机器人 SDK 失败: {e}")
        except Exception as e:
            self._robot_connected = False
            logger.error(f"连接机器人异常: {e}", exc_info=True)
            QMessageBox.critical(self, "错误", f"连接失败: {e}")

    def _disconnect_robot(self):
        """断开连接"""
        logger.info("尝试断开机器人连接")
        try:
            if self._robot is not None:
                logger.debug("释放机器人资源")
                self._robot.release_robot()
                # 销毁实例，让 Python GC 回收资源
                del self._robot
                self._robot = None
                self._dcss = None
                logger.debug("执行垃圾回收")
                gc.collect()  # 强制垃圾回收
                time.sleep(0.3)
            self._robot_connected = False
            logger.info("机器人已断开连接")
        except Exception as e:
            logger.error(f"断开机器人连接异常: {e}", exc_info=True)
        self._update_brake_buttons_state(False)
        self.btn_connect_robot.setText("连接")
        self.btn_connect_robot.setStyleSheet("background-color: #2196F3; color: white;")
        self.joint_label.setText("关节角度: 未连接")

    def _update_brake_buttons_state(self, enabled: bool):
        """更新松闸/抱闸按钮的启用状态"""
        self.btn_left_release.setEnabled(enabled)
        self.btn_left_brake.setEnabled(enabled)
        self.btn_right_release.setEnabled(enabled)
        self.btn_right_brake.setEnabled(enabled)

    def _on_brake(self, arm: str):
        """抱闸回调"""
        if not self._robot_connected:
            QMessageBox.warning(self, "警告", "请先连接机器人！")
            return

        try:
            param_name = 'BRAK0' if arm == 'A' else 'BRAK1'
            arm_name = "左臂" if arm == 'A' else "右臂"
            logger.info(f"执行抱闸操作: {arm_name} (参数: {param_name})")
            self._robot.set_param('int', param_name, 1)
            logger.info(f"{arm_name}抱闸成功")
            QMessageBox.information(self, "成功", f"{arm_name}已抱闸")
        except Exception as e:
            logger.error(f"抱闸失败: {e}", exc_info=True)
            QMessageBox.critical(self, "错误", f"抱闸失败: {e}")

    def _on_release_brake(self, arm: str):
        """松闸回调"""
        arm_name = "左臂" if arm == 'A' else "右臂"
        logger.info(f"用户请求松闸: {arm_name}")

        # 松闸有风险，需要确认
        reply = QMessageBox.warning(
            self, "警告",
            f"确定要松开{arm_name}的抱闸吗？\n松闸后机械臂可能因重力下坠！",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        if reply != QMessageBox.Yes:
            logger.info(f"用户取消松闸操作: {arm_name}")
            return

        if not self._robot_connected:
            QMessageBox.warning(self, "警告", "请先连接机器人！")
            return

        try:
            param_name = 'BRAK0' if arm == 'A' else 'BRAK1'
            logger.info(f"执行松闸操作: {arm_name} (参数: {param_name})")
            self._robot.set_param('int', param_name, 2)
            logger.info(f"{arm_name}松闸成功")
            QMessageBox.information(self, "成功", f"{arm_name}已松闸")
        except Exception as e:
            logger.error(f"松闸失败: {e}", exc_info=True)
            QMessageBox.critical(self, "错误", f"松闸失败: {e}")

    def _update_joint_display(self):
        """更新关节角度显示"""
        if not self._robot_connected or self._robot is None or self._dcss is None:
            return

        try:
            data = self._robot.subscribe(self._dcss)
            if data is None:
                logger.debug("机器人返回数据为空")
                return

            # 获取左臂(A)和右臂(B)的关节角度
            left_joints = data['outputs'][0]['fb_joint_pos']
            right_joints = data['outputs'][1]['fb_joint_pos']

            # 格式化显示
            left_str = ", ".join([f"{j:.1f}" for j in left_joints[:7]])
            right_str = ", ".join([f"{j:.1f}" for j in right_joints[:7]])

            self.joint_label.setText(f"左臂: [{left_str}]\n右臂: [{right_str}]")
        except KeyError as e:
            logger.error(f"解析关节角度数据失败，键不存在: {e}", exc_info=True)
        except IndexError as e:
            logger.error(f"解析关节角度数据失败，索引越界: {e}", exc_info=True)
        except Exception as e:
            logger.error(f"更新关节角度显示失败: {e}", exc_info=True)

    def _restart_application(self):
        """重启应用程序"""
        logger.info("准备重启应用程序")

        # 先清理资源
        logger.debug("清理当前应用资源")
        self.stop()

        # 获取当前脚本路径和参数
        python = sys.executable
        args = sys.argv[:]
        logger.debug(f"重启命令: {python} {' '.join(args)}")

        # 启动新进程
        logger.info("启动新的应用程序进程")
        subprocess.Popen([python] + args)

        # 退出当前进程
        logger.info("退出当前进程")
        from PyQt5.QtWidgets import QApplication
        QApplication.quit()

    def _check_existing_nodes_on_startup(self):
        """启动时检查是否有正在运行的 ROS2 节点"""
        logger.info("检查是否有正在运行的 ROS2 节点")

        try:
            result = subprocess.run(
                ["ros2", "node", "list"],
                capture_output=True,
                text=True,
                timeout=3
            )

            if result.returncode != 0:
                logger.debug("无法获取节点列表")
                return

            nodes = [node.strip() for node in result.stdout.strip().split('\n') if node.strip()]

            # 过滤掉 monitor 自己的节点
            teleop_nodes = [node for node in nodes if 'teleop_monitor' not in node and node]

            if teleop_nodes:
                logger.warning(f"发现 {len(teleop_nodes)} 个运行中的节点: {teleop_nodes}")

                # 弹出对话框询问是否清理
                reply = QMessageBox.question(
                    self, "发现运行中的节点",
                    f"发现 {len(teleop_nodes)} 个正在运行的 ROS2 节点：\n\n" +
                    "\n".join(f"  • {node}" for node in teleop_nodes[:10]) +
                    (f"\n  ... 还有 {len(teleop_nodes) - 10} 个节点" if len(teleop_nodes) > 10 else "") +
                    "\n\n是否清理这些节点？",
                    QMessageBox.Yes | QMessageBox.No,
                    QMessageBox.No
                )

                if reply == QMessageBox.Yes:
                    logger.info("用户选择清理节点")
                    self._do_clean_nodes(teleop_nodes)
            else:
                logger.info("没有发现运行中的节点")

        except subprocess.TimeoutExpired:
            logger.error("获取节点列表超时")
        except Exception as e:
            logger.error(f"检查节点时出错: {e}", exc_info=True)

    def _clean_ros_nodes(self):
        """清理 ROS2 节点（按钮回调）"""
        logger.info("用户请求清理 ROS2 节点")

        try:
            result = subprocess.run(
                ["ros2", "node", "list"],
                capture_output=True,
                text=True,
                timeout=3
            )

            if result.returncode != 0:
                QMessageBox.warning(self, "错误", "无法获取节点列表！")
                return

            nodes = [node.strip() for node in result.stdout.strip().split('\n') if node.strip()]

            # 过滤掉 monitor 自己的节点
            teleop_nodes = [node for node in nodes if 'teleop_monitor' not in node and node]

            if not teleop_nodes:
                QMessageBox.information(self, "提示", "没有发现需要清理的节点。")
                logger.info("没有发现需要清理的节点")
                return

            # 弹出确认对话框
            reply = QMessageBox.question(
                self, "确认清理",
                f"发现 {len(teleop_nodes)} 个节点：\n\n" +
                "\n".join(f"  • {node}" for node in teleop_nodes[:10]) +
                (f"\n  ... 还有 {len(teleop_nodes) - 10} 个节点" if len(teleop_nodes) > 10 else "") +
                "\n\n确定要清理这些节点吗？",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No
            )

            if reply == QMessageBox.Yes:
                self._do_clean_nodes(teleop_nodes)
            else:
                logger.info("用户取消清理")

        except subprocess.TimeoutExpired:
            logger.error("获取节点列表超时")
            QMessageBox.critical(self, "错误", "获取节点列表超时！")
        except Exception as e:
            logger.error(f"清理节点时出错: {e}", exc_info=True)
            QMessageBox.critical(self, "错误", f"清理节点失败：{e}")

    def _do_clean_nodes(self, nodes: list):
        """执行清理节点操作"""
        logger.info(f"开始清理 {len(nodes)} 个节点")

        success_count = 0
        failed_nodes = []

        for node in nodes:
            try:
                logger.debug(f"尝试清理节点: {node}")

                # 使用 pkill 根据节点名称杀死进程
                # ROS2 节点进程名通常包含节点名
                node_name = node.split('/')[-1]  # 提取节点名称（去掉命名空间）

                # 尝试多种方法杀死节点
                killed = False

                # 方法1: 通过 ros2 daemon stop（清理 daemon）
                try:
                    subprocess.run(
                        ["ros2", "daemon", "stop"],
                        capture_output=True,
                        timeout=5
                    )
                    time.sleep(0.5)
                    subprocess.run(
                        ["ros2", "daemon", "start"],
                        capture_output=True,
                        timeout=5
                    )
                    killed = True
                    logger.debug("已重启 ROS2 daemon")
                except Exception:
                    pass

                # 方法2: 使用 pkill 杀死相关进程
                if not killed:
                    try:
                        result = subprocess.run(
                            ["pkill", "-f", node_name],
                            capture_output=True,
                            timeout=2
                        )
                        if result.returncode == 0:
                            killed = True
                            logger.debug(f"通过 pkill 清理节点: {node_name}")
                    except Exception:
                        pass

                # 方法3: 杀死所有 Python ROS2 节点
                if not killed:
                    try:
                        subprocess.run(
                            ["pkill", "-f", "ros2.*python"],
                            capture_output=True,
                            timeout=2
                        )
                        killed = True
                        logger.debug("清理了所有 Python ROS2 节点")
                    except Exception:
                        pass

                if killed:
                    success_count += 1
                else:
                    failed_nodes.append(node)

            except Exception as e:
                logger.error(f"清理节点 {node} 失败: {e}", exc_info=True)
                failed_nodes.append(node)

        # 等待进程清理
        time.sleep(1.0)

        # 显示结果
        if success_count > 0:
            logger.info(f"成功清理 {success_count} 个节点")
            if failed_nodes:
                QMessageBox.warning(
                    self, "部分成功",
                    f"成功清理 {success_count} 个节点。\n\n"
                    f"失败 {len(failed_nodes)} 个节点：\n" +
                    "\n".join(f"  • {node}" for node in failed_nodes[:5])
                )
            else:
                QMessageBox.information(
                    self, "成功",
                    f"成功清理 {success_count} 个节点！"
                )
        else:
            logger.warning("没有成功清理任何节点")
            QMessageBox.warning(self, "失败", "清理节点失败！\n可能需要手动清理。")



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
    "Hand Only (Manus+Wuji)": ["wuji_teleop_hand.launch.py", "hand_input:=manus"],
    "Arm Only (Tracker+Tianji)": ["wuji_teleop_arm.launch.py", "arm_input:=tracker"],
    "Hand+Arm Full": ["wuji_teleop.launch.py", "hand_input:=manus", "arm_input:=tracker"],
    "Hand+Arm+Camera Full": ["wuji_teleop_camera.launch.py", "hand_input:=manus", "arm_input:=tracker"],
    "Hand+Arm Single (Left)": ["wuji_teleop_single.launch.py", "side:=left", "hand_input:=manus", "arm_input:=tracker"],
    "Hand+Arm Single (Right)": ["wuji_teleop_single.launch.py", "side:=right", "hand_input:=manus", "arm_input:=tracker"],
}


class MonitorWindow(QMainWindow):
    """Main window for the teleop monitor."""

    def __init__(self, ros_node: TeleopMonitorNode):
        super().__init__()
        logger.info("Initializing main window")
        self.ros_node = ros_node
        self.launch_process: Optional[subprocess.Popen] = None

        # Robot connection (for brake/release brake)
        self._robot = None
        self._dcss = None
        self._robot_connected = False
        self._robot_ever_connected = False  # Track if robot was ever connected

        # Background scan thread
        self._scanning = True
        self._scan_thread = threading.Thread(target=self._scan_worker, daemon=True)

        logger.debug("Setting up UI")
        self._setup_ui()

        logger.debug("Setting up timers")
        self._setup_timer()

        logger.debug("Starting background scan thread")
        self._scan_thread.start()
        logger.info("Main window initialized")

        # Check for leftover ROS2 nodes on startup
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
        manus_title = QLabel("Manus Glove")
        manus_title.setFont(QFont("Arial", 11, QFont.Bold))
        manus_layout.addWidget(manus_title)

        manus_devices = QHBoxLayout()
        self.manus_left = DeviceStatusWidget("Left")
        self.manus_right = DeviceStatusWidget("Right")
        manus_devices.addWidget(self.manus_left)
        manus_devices.addWidget(self.manus_right)
        manus_layout.addLayout(manus_devices)
        manus_group.setLayout(manus_layout)

        # Wuji Hands
        hand_group = QWidget()
        hand_layout = QVBoxLayout()
        hand_layout.setSpacing(4)
        hand_title = QLabel("Wuji Hand")
        hand_title.setFont(QFont("Arial", 11, QFont.Bold))
        hand_layout.addWidget(hand_title)

        hand_devices = QHBoxLayout()
        self.hand_left = DeviceStatusWidget("Left")
        self.hand_right = DeviceStatusWidget("Right")
        hand_devices.addWidget(self.hand_left)
        hand_devices.addWidget(self.hand_right)
        hand_layout.addLayout(hand_devices)
        hand_group.setLayout(hand_layout)

        # Tianji Arm
        tianji_group = QWidget()
        tianji_layout = QVBoxLayout()
        tianji_layout.setSpacing(4)
        tianji_title = QLabel("Tianji Arm")
        tianji_title.setFont(QFont("Arial", 11, QFont.Bold))
        tianji_layout.addWidget(tianji_title)

        self.tianji_arm = DeviceStatusWidget("Dual")
        tianji_layout.addWidget(self.tianji_arm)

        # Release brake / brake buttons
        brake_layout = QHBoxLayout()
        brake_layout.setSpacing(4)

        # Connect button
        self.btn_connect_robot = QPushButton("Connect")
        self.btn_connect_robot.setFixedSize(50, 28)
        self.btn_connect_robot.setStyleSheet("background-color: #2196F3; color: white;")
        self.btn_connect_robot.clicked.connect(self._toggle_robot_connection)
        brake_layout.addWidget(self.btn_connect_robot)

        brake_layout.addSpacing(10)

        # Left arm (Arm A)
        left_label = QLabel("Left:")
        brake_layout.addWidget(left_label)
        self.btn_left_release = QPushButton("Release")
        self.btn_left_release.setFixedSize(50, 28)
        self.btn_left_release.setStyleSheet("background-color: #ff9800; color: white;")
        self.btn_left_release.clicked.connect(lambda: self._on_release_brake('A'))
        self.btn_left_release.setEnabled(False)
        brake_layout.addWidget(self.btn_left_release)
        self.btn_left_brake = QPushButton("Brake")
        self.btn_left_brake.setFixedSize(50, 28)
        self.btn_left_brake.clicked.connect(lambda: self._on_brake('A'))
        self.btn_left_brake.setEnabled(False)
        brake_layout.addWidget(self.btn_left_brake)

        brake_layout.addSpacing(10)

        # Right arm (Arm B)
        right_label = QLabel("Right:")
        brake_layout.addWidget(right_label)
        self.btn_right_release = QPushButton("Release")
        self.btn_right_release.setFixedSize(50, 28)
        self.btn_right_release.setStyleSheet("background-color: #ff9800; color: white;")
        self.btn_right_release.clicked.connect(lambda: self._on_release_brake('B'))
        self.btn_right_release.setEnabled(False)
        brake_layout.addWidget(self.btn_right_release)
        self.btn_right_brake = QPushButton("Brake")
        self.btn_right_brake.setFixedSize(50, 28)
        self.btn_right_brake.clicked.connect(lambda: self._on_brake('B'))
        self.btn_right_brake.setEnabled(False)
        brake_layout.addWidget(self.btn_right_brake)

        brake_layout.addStretch()
        tianji_layout.addLayout(brake_layout)

        # Joint angle display
        self.joint_label = QLabel("Joints: N/A")
        self.joint_label.setStyleSheet("color: #666; font-size: 10px;")
        tianji_layout.addWidget(self.joint_label)

        tianji_group.setLayout(tianji_layout)
        tianji_group.setMinimumHeight(120)

        # Vive Tracker - disabled to avoid OpenVR crash
        # vive_group = QWidget()
        # vive_layout = QVBoxLayout()
        # vive_layout.setSpacing(4)
        # vive_title = QLabel("Vive Tracker")
        # vive_title.setFont(QFont("Arial", 11, QFont.Bold))
        # vive_layout.addWidget(vive_title)
        #
        # vive_devices = QHBoxLayout()
        # self.tracker_widget = DeviceStatusWidget("Tracker")
        # self.basestation_widget = DeviceStatusWidget("Base Station")
        # vive_devices.addWidget(self.tracker_widget)
        # vive_devices.addWidget(self.basestation_widget)
        # vive_layout.addLayout(vive_devices)
        # vive_group.setLayout(vive_layout)
        # vive_group.setMinimumHeight(120)

        device_layout.addWidget(manus_group)
        device_layout.addWidget(hand_group)
        main_layout.addLayout(device_layout)

        # Tianji Arm (second row) - Vive Tracker disabled
        device_layout2 = QHBoxLayout()
        # device_layout2.addWidget(vive_group)  # disabled
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
        launch_label = QLabel("Launch:")
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
        self.launch_btn = QPushButton("Start Teleop")
        self.launch_btn.setFixedSize(160, 55)
        self.launch_btn.setFont(QFont("Arial", 16, QFont.Bold))
        self.launch_btn.setStyleSheet(
            "QPushButton { background-color: #4CAF50; color: white; border-radius: 4px; border: none; }"
            "QPushButton:hover { background-color: #45a049; }"
        )
        self.launch_btn.clicked.connect(self._toggle_launch)
        launch_layout.addWidget(self.launch_btn)

        # Clean nodes button
        self.clean_nodes_btn = QPushButton("Clean Nodes")
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
        self.status_label = QLabel("Listening...")
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
        """Background thread: perform time-consuming scan operations."""
        logger.info("Background scan thread started")
        scan_count = 0
        while self._scanning:
            scan_count += 1
            logger.debug(f"Starting device scan #{scan_count}")
            try:
                logger.debug("Scanning Manus glove USB devices")
                self.ros_node.update_glove_status()    # Manus USB scan

                logger.debug("Scanning Wuji hand USB devices")
                self.ros_node.update_hand_status()     # Wuji hand USB scan

                logger.debug("Scanning Tianji arm network status (ping)")
                self.ros_node.update_tianji_status()   # Tianji ping

                # OpenVR scan disabled to avoid crash
                # logger.debug("Scanning Vive Tracker and base station status")
                # self.ros_node.update_vive_status()     # Vive Tracker + base station

                logger.debug("Scanning ROS2 node status")
                self.ros_node.update_node_status()     # ros2 node list

                logger.debug("Scanning stereo head camera hardware status")
                self.ros_node.update_stereo_head_hardware_status()  # Stereo head hardware

                logger.debug(f"Device scan #{scan_count} completed")
            except Exception as e:
                logger.error(f"Device scan error (#{scan_count}): {e}", exc_info=True)
            time.sleep(2.0)  # Scan interval
        logger.info("Background scan thread exited")

    def _update_ui(self):
        # Spin ROS node
        try:
            rclpy.spin_once(self.ros_node, timeout_sec=0)
        except Exception as e:
            logger.error(f"ROS2 spin_once error: {e}", exc_info=True)
            return

        # Update topic status (lightweight, check timestamps only)
        try:
            self.ros_node.update_hand_input_status()
            self.ros_node.update_glove_topic_status()
            self.ros_node.update_camera_status()
            self.ros_node.update_stereo_head_status()
        except Exception as e:
            logger.error(f"Topic status update error: {e}", exc_info=True)

        # Update UI components (read data from ros_node, updated by background thread)
        try:
            self._update_data_flow()
            self._update_camera_widget()
            self._update_stereo_head_widget()
            self._update_manus_widgets()
            self._update_hand_widgets()
            self._update_tianji_widget()
            # self._update_vive_widget()  # Vive Tracker monitoring disabled
            self._update_node_status()
            self._update_status_bar()
            self._check_launch_status()
        except Exception as e:
            logger.error(f"UI component update error: {e}", exc_info=True)

        # Real-time joint angle update
        if self._robot_connected:
            try:
                self._update_joint_display()
            except Exception as e:
                logger.error(f"Joint angle display update error: {e}", exc_info=True)

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
        # Update hardware status
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

        # Update topic status
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
        # Update Tracker widget - show connection status per body part
        from .scanner import ViveTrackerScanner
        tracker_config = ViveTrackerScanner.get_tracker_config()
        connected_serials = self.ros_node.vive_tracker_serials
        body_names = ViveTrackerScanner.BODY_PART_NAMES

        if tracker_config:
            # Check connection status for each body part
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
            # Display colored status using HTML format
            self.tracker_widget.info_label.setText("<br>".join(status_parts))
        else:
            # No config, display raw count
            tracker_count = len(connected_serials)
            if tracker_count > 0:
                self.tracker_widget.set_connected(True)
                self.tracker_widget.set_id(f"{tracker_count}")
                self.tracker_widget.set_info("")
            else:
                self.tracker_widget.set_connected(False)
                self.tracker_widget.set_id("No Devices")
                self.tracker_widget.set_info("")

        # Update Base Station widget
        base_stations = self.ros_node.base_stations
        station_count = len(base_stations)
        connected_count = sum(1 for s in base_stations.values() if s.get('connected', False))
        if station_count > 0:
            all_connected = (connected_count == station_count)
            self.basestation_widget.set_connected(all_connected)
            self.basestation_widget.set_id(f"{connected_count}/{station_count}")
            # Show modes (A/B/C)
            modes = [s.get('mode', '?') for s in base_stations.values()]
            self.basestation_widget.set_info(" ".join(modes))
        else:
            self.basestation_widget.set_connected(False)
            self.basestation_widget.set_id("No Base Station")
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
            parts.append(f"Glove: {manus_count}")
        if hand_count > 0:
            parts.append(f"Hand: {hand_count}")
        if tianji_connected:
            parts.append("Arm: Online")
        if tracker_count > 0:
            parts.append(f"Tracker: {tracker_count}")
        if cam_count > 0:
            parts.append(f"Camera: {cam_count}/3")
        if node_count > 0:
            parts.append(f"Node: {node_count}/5")

        if parts:
            self.status_label.setText(" | ".join(parts))
        else:
            self.status_label.setText("Waiting for devices...")

    def moveEvent(self, event):
        """Close dropdown menu when window is moved."""
        self.launch_combo.hidePopup()
        super().moveEvent(event)

    def closeEvent(self, event):
        """Window close event, ensure proper cleanup of all resources and processes."""
        logger.info("Window close event triggered, cleaning up resources...")
        self.stop()  # Call stop to clean up all resources
        event.accept()  # Accept close event
        logger.info("Window close event handling completed")

    def stop(self):
        """Stop the timer, background thread, and launch process."""
        # Prevent duplicate calls
        if hasattr(self, '_stopped') and self._stopped:
            logger.warning("stop() called repeatedly, ignoring")
            return
        self._stopped = True
        logger.info("Starting cleanup")

        # Stop timers
        if hasattr(self, 'timer'):
            logger.debug("Stopping UI update timer")
            self.timer.stop()
            logger.debug("UI update timer stopped")

        # Stop background scan thread
        logger.debug("Stopping background scan thread")
        self._scanning = False
        if hasattr(self, '_scan_thread') and self._scan_thread.is_alive():
            logger.debug("Waiting for background scan thread (timeout 2s)")
            self._scan_thread.join(timeout=2.0)  # Wait for thread, max 2s
            if self._scan_thread.is_alive():
                logger.warning("Background scan thread did not stop within 2s")
            else:
                logger.debug("Background scan thread stopped normally")

        # Stop launch process
        logger.debug("Stopping launch process")
        self._stop_launch()

        # Release robot connection
        logger.debug("Disconnecting robot")
        self._disconnect_robot()

        logger.info("All resources cleaned up")

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
        logger.info("Attempting to start teleop system")

        # If robot was connected before, prompt user to restart
        if self._robot_ever_connected:
            logger.warning("Robot was connected in this session, restart required to launch teleop")
            reply = QMessageBox.question(
                self, "Restart Required",
                "A robot connection was made in this session.\n"
                "Due to SDK port conflicts, Monitor must restart before launching teleop.\n\n"
                "Restart now?",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.Yes
            )
            if reply == QMessageBox.Yes:
                logger.info("User chose to restart application")
                self._restart_application()
            else:
                logger.info("User cancelled restart")
            return

        try:
            config_name = self.launch_combo.currentText()
            config_args = LAUNCH_CONFIGS.get(config_name, [])
            if not config_args:
                logger.error(f"Config not found: {config_name}")
                return

            launch_file = config_args[0]
            extra_args = config_args[1:]

            cmd = ["ros2", "launch", "wuji_teleop_bringup", launch_file] + extra_args
            logger.info(f"Launching config: {config_name}")
            logger.debug(f"Executing command: {' '.join(cmd)}")

            self.launch_process = subprocess.Popen(
                cmd,
                env=os.environ.copy(),
                preexec_fn=os.setsid  # Create new process group
            )
            logger.info(f"Launch process started, PID: {self.launch_process.pid}")

            self.launch_btn.setText("Stop")
            self.launch_combo.setEnabled(False)  # Disable dropdown
            self.btn_connect_robot.setEnabled(False)  # Disable connect button
            self.launch_btn.setStyleSheet(
                "QPushButton { background-color: #f44336; color: white; border-radius: 4px; border: none; }"
                "QPushButton:hover { background-color: #da190b; }"
            )
        except Exception as e:
            logger.error(f"Failed to start launch: {e}", exc_info=True)

    def _stop_launch(self):
        """Stop the teleop launch process."""
        if self.launch_process is not None:
            logger.info(f"Stopping launch process, PID: {self.launch_process.pid}")
            try:
                pgid = os.getpgid(self.launch_process.pid)
                logger.debug(f"Process group ID: {pgid}")

                # Send SIGINT first
                logger.debug("Sending SIGINT")
                os.killpg(pgid, signal.SIGINT)
                try:
                    self.launch_process.wait(timeout=3)
                    logger.info("Process exited via SIGINT")
                except subprocess.TimeoutExpired:
                    # SIGINT timed out, send SIGTERM
                    logger.warning("SIGINT timed out (3s), sending SIGTERM")
                    os.killpg(pgid, signal.SIGTERM)
                    try:
                        self.launch_process.wait(timeout=2)
                        logger.info("Process exited via SIGTERM")
                    except subprocess.TimeoutExpired:
                        # SIGTERM also timed out, force SIGKILL
                        logger.warning("SIGTERM timed out (2s), sending SIGKILL")
                        os.killpg(pgid, signal.SIGKILL)
                        self.launch_process.wait(timeout=2)
                        logger.info("Process force-killed via SIGKILL")
            except ProcessLookupError:
                # Process already exited
                logger.debug("Process already exited (ProcessLookupError)")
            except Exception as e:
                logger.error(f"Error stopping launch process: {e}", exc_info=True)
                # Try to kill process directly
                try:
                    logger.debug("Attempting to kill process directly")
                    self.launch_process.kill()
                    self.launch_process.wait(timeout=2)
                    logger.info("Process force-killed via kill()")
                except Exception as e2:
                    logger.error(f"Failed to force-kill process: {e2}", exc_info=True)
            self.launch_process = None
            logger.info("Launch process stopped")
        else:
            logger.debug("No running launch process")

        self.launch_btn.setText("Start Teleop")
        self.launch_combo.setEnabled(True)  # Re-enable dropdown
        self.btn_connect_robot.setEnabled(True)  # Re-enable connect button
        self.launch_btn.setStyleSheet(
            "QPushButton { background-color: #4CAF50; color: white; border-radius: 4px; border: none; }"
            "QPushButton:hover { background-color: #45a049; }"
        )

    def _check_launch_status(self):
        """Check if launch process is still running."""
        if self.launch_process is not None and self.launch_process.poll() is not None:
            # Process has exited
            exit_code = self.launch_process.returncode
            logger.warning(f"Launch process exited unexpectedly, exit code: {exit_code}")
            self.launch_process = None
            self.launch_btn.setText("Start Teleop")
            self.launch_combo.setEnabled(True)  # Re-enable dropdown
            self.btn_connect_robot.setEnabled(True)  # Re-enable connect button
            self.launch_btn.setStyleSheet(
                "QPushButton { background-color: #4CAF50; color: white; border-radius: 4px; border: none; }"
                "QPushButton:hover { background-color: #45a049; }"
            )

    def _toggle_robot_connection(self):
        """Toggle robot connection state."""
        if self._robot_connected:
            self._disconnect_robot()
        else:
            self._connect_robot()

    def _connect_robot(self):
        """Connect to robot."""
        logger.info("Attempting to connect robot")

        if not self.ros_node.tianji_arm_reachable:
            logger.warning("Robot unreachable, cannot connect")
            QMessageBox.warning(self, "Warning", "Robot unreachable! Check network connection.")
            return

        try:
            logger.debug("Importing tianji_output module")
            from tianji_output import Marvin_Robot
            from tianji_output._internal import DCSS
            robot_ip = self.ros_node.get_tianji_ip()
            logger.debug(f"Robot IP: {robot_ip}")

            # Create new instance for each connection
            logger.debug("Creating Marvin_Robot instance")
            self._robot = Marvin_Robot()
            self._dcss = DCSS()

            logger.debug("Connecting to robot...")
            result = self._robot.connect(robot_ip)
            # SDK return value: 1=success, 0=failure
            if result == 1:
                self._robot_connected = True
                self._robot_ever_connected = True  # Mark as previously connected
                logger.info(f"Robot connected: {robot_ip}")
                self._update_brake_buttons_state(True)
                self.btn_connect_robot.setText("Disconnect")
                self.btn_connect_robot.setStyleSheet("background-color: #f44336; color: white;")
                # Display joint angles
                self._update_joint_display()
            else:
                self._robot_connected = False
                logger.error(f"Robot connection failed: {robot_ip}, SDK error code: {result}")
                QMessageBox.warning(self, "Warning", f"Robot connection failed! Error code: {result}")
        except ImportError as e:
            self._robot_connected = False
            logger.error(f"Failed to import tianji_output module: {e}", exc_info=True)
            QMessageBox.critical(self, "Error", f"Failed to import robot SDK: {e}")
        except Exception as e:
            self._robot_connected = False
            logger.error(f"Robot connection error: {e}", exc_info=True)
            QMessageBox.critical(self, "Error", f"Connection failed: {e}")

    def _disconnect_robot(self):
        """Disconnect from robot."""
        logger.info("Attempting to disconnect robot")
        try:
            if self._robot is not None:
                logger.debug("Releasing robot resources")
                self._robot.release_robot()
                # Destroy instance, let Python GC reclaim resources
                del self._robot
                self._robot = None
                self._dcss = None
                logger.debug("Running garbage collection")
                gc.collect()  # Force garbage collection
                time.sleep(0.3)
            self._robot_connected = False
            logger.info("Robot disconnected")
        except Exception as e:
            logger.error(f"Robot disconnect error: {e}", exc_info=True)
        self._update_brake_buttons_state(False)
        self.btn_connect_robot.setText("Connect")
        self.btn_connect_robot.setStyleSheet("background-color: #2196F3; color: white;")
        self.joint_label.setText("Joints: N/A")

    def _update_brake_buttons_state(self, enabled: bool):
        """Update release brake / brake button enabled state."""
        self.btn_left_release.setEnabled(enabled)
        self.btn_left_brake.setEnabled(enabled)
        self.btn_right_release.setEnabled(enabled)
        self.btn_right_brake.setEnabled(enabled)

    def _on_brake(self, arm: str):
        """Brake callback."""
        if not self._robot_connected:
            QMessageBox.warning(self, "Warning", "Please connect the robot first!")
            return

        try:
            param_name = 'BRAK0' if arm == 'A' else 'BRAK1'
            arm_name = "Left arm" if arm == 'A' else "Right arm"
            logger.info(f"Applying brake: {arm_name} (param: {param_name})")
            self._robot.set_param('int', param_name, 1)
            logger.info(f"{arm_name} brake applied")
            QMessageBox.information(self, "Success", f"{arm_name} brake applied")
        except Exception as e:
            logger.error(f"Brake failed: {e}", exc_info=True)
            QMessageBox.critical(self, "Error", f"Brake failed: {e}")

    def _on_release_brake(self, arm: str):
        """Release brake callback."""
        arm_name = "Left arm" if arm == 'A' else "Right arm"
        logger.info(f"User requested release brake: {arm_name}")

        # Releasing brake is risky, require confirmation
        reply = QMessageBox.warning(
            self, "Warning",
            f"Are you sure you want to release the {arm_name} brake?\n"
            f"The arm may drop due to gravity after release!",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        if reply != QMessageBox.Yes:
            logger.info(f"User cancelled release brake: {arm_name}")
            return

        if not self._robot_connected:
            QMessageBox.warning(self, "Warning", "Please connect the robot first!")
            return

        try:
            param_name = 'BRAK0' if arm == 'A' else 'BRAK1'
            logger.info(f"Releasing brake: {arm_name} (param: {param_name})")
            self._robot.set_param('int', param_name, 2)
            logger.info(f"{arm_name} brake released")
            QMessageBox.information(self, "Success", f"{arm_name} brake released")
        except Exception as e:
            logger.error(f"Release brake failed: {e}", exc_info=True)
            QMessageBox.critical(self, "Error", f"Release brake failed: {e}")

    def _update_joint_display(self):
        """Update joint angle display."""
        if not self._robot_connected or self._robot is None or self._dcss is None:
            return

        try:
            data = self._robot.subscribe(self._dcss)
            if data is None:
                logger.debug("Robot returned empty data")
                return

            # Get left arm (A) and right arm (B) joint angles
            left_joints = data['outputs'][0]['fb_joint_pos']
            right_joints = data['outputs'][1]['fb_joint_pos']

            # Format display
            left_str = ", ".join([f"{j:.1f}" for j in left_joints[:7]])
            right_str = ", ".join([f"{j:.1f}" for j in right_joints[:7]])

            self.joint_label.setText(f"L Arm: [{left_str}]\nR Arm: [{right_str}]")
        except KeyError as e:
            logger.error(f"Failed to parse joint data, key not found: {e}", exc_info=True)
        except IndexError as e:
            logger.error(f"Failed to parse joint data, index out of range: {e}", exc_info=True)
        except Exception as e:
            logger.error(f"Failed to update joint display: {e}", exc_info=True)

    def _restart_application(self):
        """Restart the application."""
        logger.info("Preparing to restart application")

        # Clean up resources first
        logger.debug("Cleaning up current application resources")
        self.stop()

        # Get current script path and arguments
        python = sys.executable
        args = sys.argv[:]
        logger.debug(f"Restart command: {python} {' '.join(args)}")

        # Start new process
        logger.info("Starting new application process")
        subprocess.Popen([python] + args)

        # Exit current process
        logger.info("Exiting current process")
        from PyQt5.QtWidgets import QApplication
        QApplication.quit()

    def _check_existing_nodes_on_startup(self):
        """Check for running ROS2 nodes on startup."""
        logger.info("Checking for running ROS2 nodes")

        try:
            result = subprocess.run(
                ["ros2", "node", "list"],
                capture_output=True,
                text=True,
                timeout=3
            )

            if result.returncode != 0:
                logger.debug("Unable to get node list")
                return

            nodes = [node.strip() for node in result.stdout.strip().split('\n') if node.strip()]

            # Filter out monitor's own nodes
            teleop_nodes = [node for node in nodes if 'teleop_monitor' not in node and node]

            if teleop_nodes:
                logger.warning(f"Found {len(teleop_nodes)} running nodes: {teleop_nodes}")

                # Show dialog asking whether to clean up
                reply = QMessageBox.question(
                    self, "Running Nodes Detected",
                    f"Found {len(teleop_nodes)} running ROS2 nodes:\n\n" +
                    "\n".join(f"  • {node}" for node in teleop_nodes[:10]) +
                    (f"\n  ... and {len(teleop_nodes) - 10} more" if len(teleop_nodes) > 10 else "") +
                    "\n\nClean up these nodes?",
                    QMessageBox.Yes | QMessageBox.No,
                    QMessageBox.No
                )

                if reply == QMessageBox.Yes:
                    logger.info("User chose to clean up nodes")
                    self._do_clean_nodes(teleop_nodes)
            else:
                logger.info("No running nodes found")

        except subprocess.TimeoutExpired:
            logger.error("Timed out getting node list")
        except Exception as e:
            logger.error(f"Error checking nodes: {e}", exc_info=True)

    def _clean_ros_nodes(self):
        """Clean up ROS2 nodes (button callback)."""
        logger.info("User requested ROS2 node cleanup")

        try:
            result = subprocess.run(
                ["ros2", "node", "list"],
                capture_output=True,
                text=True,
                timeout=3
            )

            if result.returncode != 0:
                QMessageBox.warning(self, "Error", "Unable to get node list!")
                return

            nodes = [node.strip() for node in result.stdout.strip().split('\n') if node.strip()]

            # Filter out monitor's own nodes
            teleop_nodes = [node for node in nodes if 'teleop_monitor' not in node and node]

            if not teleop_nodes:
                QMessageBox.information(self, "Info", "No nodes found to clean up.")
                logger.info("No nodes found to clean up")
                return

            # Show confirmation dialog
            reply = QMessageBox.question(
                self, "Confirm Cleanup",
                f"Found {len(teleop_nodes)} nodes:\n\n" +
                "\n".join(f"  • {node}" for node in teleop_nodes[:10]) +
                (f"\n  ... and {len(teleop_nodes) - 10} more" if len(teleop_nodes) > 10 else "") +
                "\n\nClean up these nodes?",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No
            )

            if reply == QMessageBox.Yes:
                self._do_clean_nodes(teleop_nodes)
            else:
                logger.info("User cancelled cleanup")

        except subprocess.TimeoutExpired:
            logger.error("Timed out getting node list")
            QMessageBox.critical(self, "Error", "Timed out getting node list!")
        except Exception as e:
            logger.error(f"Error during node cleanup: {e}", exc_info=True)
            QMessageBox.critical(self, "Error", f"Node cleanup failed: {e}")

    def _do_clean_nodes(self, nodes: list):
        """Execute node cleanup operation."""
        logger.info(f"Starting cleanup of {len(nodes)} nodes")

        success_count = 0
        failed_nodes = []

        for node in nodes:
            try:
                logger.debug(f"Attempting to clean node: {node}")

                # Use pkill to kill process by node name
                # ROS2 node process names usually contain the node name
                node_name = node.split('/')[-1]  # Extract node name (strip namespace)

                # Try multiple methods to kill the node
                killed = False

                # Method 1: via ros2 daemon stop (clean daemon)
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
                    logger.debug("ROS2 daemon restarted")
                except Exception:
                    pass

                # Method 2: use pkill to kill related processes
                if not killed:
                    try:
                        result = subprocess.run(
                            ["pkill", "-f", node_name],
                            capture_output=True,
                            timeout=2
                        )
                        if result.returncode == 0:
                            killed = True
                            logger.debug(f"Killed node via pkill: {node_name}")
                    except Exception:
                        pass

                # Method 3: kill all Python ROS2 nodes
                if not killed:
                    try:
                        subprocess.run(
                            ["pkill", "-f", "ros2.*python"],
                            capture_output=True,
                            timeout=2
                        )
                        killed = True
                        logger.debug("Killed all Python ROS2 nodes")
                    except Exception:
                        pass

                if killed:
                    success_count += 1
                else:
                    failed_nodes.append(node)

            except Exception as e:
                logger.error(f"Failed to clean node {node}: {e}", exc_info=True)
                failed_nodes.append(node)

        # Wait for process cleanup
        time.sleep(1.0)

        # Display results
        if success_count > 0:
            logger.info(f"Successfully cleaned {success_count} nodes")
            if failed_nodes:
                QMessageBox.warning(
                    self, "Partial Success",
                    f"Successfully cleaned {success_count} nodes.\n\n"
                    f"Failed to clean {len(failed_nodes)} nodes:\n" +
                    "\n".join(f"  • {node}" for node in failed_nodes[:5])
                )
            else:
                QMessageBox.information(
                    self, "Success",
                    f"Successfully cleaned {success_count} nodes!"
                )
        else:
            logger.warning("Failed to clean any nodes")
            QMessageBox.warning(self, "Failed", "Node cleanup failed!\nManual cleanup may be required.")



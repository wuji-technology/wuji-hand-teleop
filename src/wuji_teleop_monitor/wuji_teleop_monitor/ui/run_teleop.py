#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Teleop Monitor — one-click start/stop of the Wuji teleop stack.

Drives the single-machine open-source launch (`wuji_teleop_bringup
wuji_teleop.launch.py` for the hand-only / HTC presets, `pico_teleop.launch.py`
for the PICO 4 preset) via subprocess. Subprocess liveness is the sole
"running" signal — no robot_enabler lifecycle topics, since they only exist
in the multi-machine deployment.

Three hand-first presets are exposed:
  - Hand only (Wuji Glove)
  - Hand + Arm (HTC Tracker)
  - Hand + Arm (PICO 4)

Usage:
    ros2 run wuji_teleop_monitor monitor
"""

import fcntl
import os
import signal
import subprocess
import sys
import threading
import time
from collections import deque
from pathlib import Path

# Shared UI components (sibling modules).
from .qt_setup import setup_qt_plugins
setup_qt_plugins()

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QComboBox, QPlainTextEdit, QMessageBox, QGroupBox,
    QFrame,
)
from PyQt5.QtCore import Qt, QTimer, QSize, pyqtSignal
from PyQt5.QtGui import QFont, QPixmap, QImage, QPainter
from PyQt5.QtSvg import QSvgRenderer

from .joint_panel import JointDisplayPanel
from .sn_dialog import ScanSNDialog
from .theme import DARK_THEME_CSS, LOG_TEXTEDIT_CSS

# ROS2 imports (optional — joint preview degrades gracefully if absent).
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (
        QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy,
    )
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Int8
    _HAS_ROS2 = True
except ImportError:
    _HAS_ROS2 = False

# Lifecycle states published by tianji_arm_controller on
# /tianji_arm/lifecycle_state. Keep in sync with controller/tianji_arm_node.py.
_LC_INITIALIZING = 0
_LC_ENABLING = 1
_LC_READY = 2
_LC_DISABLED = 3
_LC_ENABLE_FAILED = 4
_LC_SDK_ERROR = 5
_LC_NAMES = {
    _LC_INITIALIZING: "initializing",
    _LC_ENABLING: "enabling",
    _LC_READY: "ready",
    _LC_DISABLED: "disabled",
    _LC_ENABLE_FAILED: "enable_failed",
    _LC_SDK_ERROR: "sdk_error",
}
# States that mean "controller is done with its enable attempt, one way or
# another". Used to release the Stop button from the disabled-while-starting
# gate. ENABLE_FAILED + SDK_ERROR specifically need to release Stop too —
# otherwise the operator can't recover.
_LC_RELEASES_STOP_GATE = frozenset((
    _LC_READY, _LC_DISABLED, _LC_ENABLE_FAILED, _LC_SDK_ERROR,
))

# Single-instance lock.
_LOCK_FILE = "/tmp/wuji_teleop_monitor.lock"

# Process matcher for orphan-teleop detection. Matches any wuji_teleop_bringup
# launch — the HTC / Wuji-Glove path goes via wuji_teleop.launch.py, the PICO
# path via pico_teleop.launch.py.
_BACKEND_MATCH = "ros2 launch wuji_teleop_bringup"

# Hand-first launch presets — first entry is the default. Each value is a list
# of (launch_file, *ros2_launch_args). PICO uses pico_teleop.launch.py rather
# than wuji_teleop.launch.py with `arm_input:=pico`, because the PICO arm
# controller is `tianji_world_output_node` (subscribes to /left_arm_target_pose)
# and `pico_teleop.launch.py` is the only launch that spawns both
# `pico_input_node` and `tianji_world_output_node` wired together.
LAUNCH_CONFIGS = {
    "Hand only (Wuji Glove)": [
        "wuji_teleop.launch.py",
        "enable_hand:=true",
        "enable_arm:=false",
    ],
    "Hand + Arm (HTC Tracker)": [
        "wuji_teleop.launch.py",
        "enable_hand:=true",
        "enable_arm:=true",
        "arm_input:=tracker",
    ],
    "Hand + Arm (PICO 4)": [
        "pico_teleop.launch.py",
        "enable_hand:=true",
        "enable_robot:=true",
    ],
}

def _preset_waits_for_arm_lifecycle(preset: str) -> bool:
    """True if the preset spawns tianji_arm_controller (HTC path), which
    publishes /tianji_arm/lifecycle_state. The PICO arm controller
    (tianji_world_output_node) doesn't publish lifecycle yet, so its preset
    falls back to the 2 s subprocess-uptime heuristic. Hand-only doesn't
    bring up an arm controller at all."""
    args = LAUNCH_CONFIGS.get(preset, [])
    return "enable_arm:=true" in args


# Package logo (wuji.svg lives one level up from the ui/ subpackage).
_LOGO_PATH = Path(__file__).resolve().parent.parent / "wuji_icon.png"


class TeleopLauncherUI(QMainWindow):
    """One-click Wuji teleop launcher."""

    _MAX_LOG_LINES = 5000

    _state_signal = pyqtSignal(str)
    _stop_finished_signal = pyqtSignal(bool, bool)
    _arm_lifecycle_signal = pyqtSignal(int)

    def __init__(self, ros_node=None):
        super().__init__()
        self._process = None
        self._state = "stopped"
        self._start_time = None
        self._ros_node = ros_node

        self._launch_started_at = None
        self._output_lock = threading.Lock()
        self._pending_output = deque()
        self._stop_in_progress = False

        # Last lifecycle state we heard from tianji_arm_controller. None
        # means "no message yet" (the controller's TRANSIENT_LOCAL latch
        # hasn't reached us, or this preset doesn't have an arm controller).
        self._arm_lifecycle = None
        self._waiting_for_arm_lifecycle = False

        self._state_signal.connect(self._set_state)
        self._stop_finished_signal.connect(self._on_stop_finished)
        self._arm_lifecycle_signal.connect(self._on_arm_lifecycle_main)

        self._init_ui()
        self._joint_panel.set_phase("stopped")

        self._output_flush_timer = QTimer(self)
        self._output_flush_timer.setInterval(100)
        self._output_flush_timer.timeout.connect(self._flush_output)
        self._output_flush_timer.start()

        # Joint preview: subscribe to /{side}_{arm,hand}/joint_states.
        if self._ros_node and _HAS_ROS2:
            from .qos_utils import match_publisher_qos
            for side in ('left', 'right'):
                arm_topic = f'/{side}_arm/joint_states'
                self._ros_node.create_subscription(
                    JointState, arm_topic,
                    lambda msg, s=side: self._on_joint_state(s, msg),
                    match_publisher_qos(self._ros_node, arm_topic))
                hand_topic = f'/{side}_hand/joint_states'
                self._ros_node.create_subscription(
                    JointState, hand_topic,
                    lambda msg, s=side: self._joint_panel.update_hand_joints(
                        s, list(msg.position)),
                    match_publisher_qos(self._ros_node, hand_topic))

                # Cmd Hz monitor: count joint_commands arrivals per side so the
                # panel shows the live arm/hand command publish rate. Both arm
                # paths now publish /{side}_arm/joint_commands — HTC via
                # tianji_arm_node, PICO via tianji_world_output (which runs the
                # IK and so is the first place on that path with joint angles).
                # Whichever controller is live feeds the same arm Cmd Hz cell.
                arm_cmd_topic = f'/{side}_arm/joint_commands'
                self._ros_node.create_subscription(
                    JointState, arm_cmd_topic,
                    lambda msg, s=side: self._joint_panel.record_hz(f'{s}_arm'),
                    match_publisher_qos(self._ros_node, arm_cmd_topic))
                hand_cmd_topic = f'/{side}_hand/joint_commands'
                self._ros_node.create_subscription(
                    JointState, hand_cmd_topic,
                    lambda msg, s=side: self._joint_panel.record_hz(f'{s}_hand'),
                    match_publisher_qos(self._ros_node, hand_cmd_topic))

            # Latched lifecycle state from tianji_arm_controller. RELIABLE
            # + TRANSIENT_LOCAL matches the publisher so we receive the
            # last value even when subscribing before the controller comes
            # up. See _LC_* constants for the state machine.
            _lc_qos = QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            )
            self._ros_node.create_subscription(
                Int8, '/tianji_arm/lifecycle_state',
                self._on_arm_lifecycle, _lc_qos)

        # Uptime / liveness timers.
        self._uptime_timer = QTimer(self)
        self._uptime_timer.timeout.connect(self._update_uptime)
        self._uptime_timer.start(1000)

        self._liveness_timer = QTimer(self)
        self._liveness_timer.timeout.connect(self._poll_process_alive)
        self._liveness_timer.setInterval(500)

    @staticmethod
    def _render_logo(path: Path, target_height: int) -> QPixmap:
        if path.suffix.lower() == ".svg":
            renderer = QSvgRenderer(str(path))
            if not renderer.isValid():
                return QPixmap()
            src = renderer.defaultSize()
            if src.height() <= 0:
                return QPixmap()
            scale = target_height / src.height()
            size = QSize(max(1, int(round(src.width() * scale))), target_height)
            image = QImage(size, QImage.Format_ARGB32)
            image.fill(0)
            painter = QPainter(image)
            painter.setRenderHint(QPainter.Antialiasing, True)
            painter.setRenderHint(QPainter.SmoothPixmapTransform, True)
            renderer.render(painter)
            painter.end()
            return QPixmap.fromImage(image)
        pixmap = QPixmap(str(path))
        if pixmap.isNull():
            return pixmap
        return pixmap.scaledToHeight(target_height, Qt.SmoothTransformation)

    def _init_ui(self):
        self.setWindowTitle("Wuji Teleop Monitor")
        # Portrait, ≈ golden ratio (760/620 ≈ 1.226). Fits the header +
        # Launch Configuration + 4-row joint readout + Process Output log
        # vertically; the Scan-SNs modal opens at 560×760 inside.
        self.setMinimumSize(620, 760)
        self.resize(720, 880)

        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        layout.setContentsMargins(15, 15, 15, 15)
        layout.setSpacing(12)

        # Header: logo + status frame.
        header = QHBoxLayout()
        header.setSpacing(10)

        if _LOGO_PATH.exists():
            logo_label = QLabel()
            pixmap = self._render_logo(_LOGO_PATH, target_height=36)
            if pixmap is not None and not pixmap.isNull():
                logo_label.setPixmap(pixmap)
            logo_label.setFixedHeight(40)
            header.addWidget(logo_label)

        status_frame = QFrame()
        status_frame.setFixedHeight(40)
        status_frame.setStyleSheet(
            "QFrame { background-color: #333; border-radius: 6px; }")
        status_layout = QHBoxLayout(status_frame)
        status_layout.setContentsMargins(12, 4, 12, 4)

        self._status_label = QLabel("Teleop: stopped")
        self._status_label.setFont(QFont("Arial", 13, QFont.Bold))
        self._status_label.setStyleSheet("color: #aaa;")
        status_layout.addWidget(self._status_label)

        self._uptime_label = QLabel("")
        self._uptime_label.setFont(QFont("Courier", 11))
        self._uptime_label.setStyleSheet("color: #888;")
        self._uptime_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        status_layout.addWidget(self._uptime_label)
        header.addWidget(status_frame, 1)
        layout.addLayout(header)

        # Launch configuration.
        ctrl_group = QGroupBox("Launch Configuration")
        ctrl_layout = QVBoxLayout(ctrl_group)
        ctrl_layout.setSpacing(10)

        preset_row = QHBoxLayout()
        preset_lbl = QLabel("Preset:")
        preset_lbl.setFixedWidth(80)
        preset_lbl.setStyleSheet("color: #ccc; font-size: 12px;")
        preset_row.addWidget(preset_lbl)

        _COMBO_CSS = """
            QComboBox {
                background-color: #444; color: #ddd; border: 1px solid #666;
                border-radius: 4px; padding: 4px 10px; font-size: 12px;
            }
            QComboBox::drop-down { border: none; }
            QComboBox QAbstractItemView {
                background-color: #333; color: #ddd; selection-background-color: #555;
            }
        """
        self._preset_combo = QComboBox()
        self._preset_combo.addItems(list(LAUNCH_CONFIGS.keys()))
        self._preset_combo.setMinimumWidth(260)
        self._preset_combo.setStyleSheet(_COMBO_CSS)
        preset_row.addWidget(self._preset_combo)
        preset_row.addStretch()

        # Scan-SNs button: device grab conflicts with running teleop, so it
        # follows the same enable/disable gate as the preset combo.
        self._scan_sns_btn = QPushButton("Scan SNs")
        self._scan_sns_btn.setFixedHeight(28)
        self._scan_sns_btn.setStyleSheet("""
            QPushButton { background-color: #555; color: #ccc; border-radius: 4px;
                          padding: 2px 12px; font-size: 12px; }
            QPushButton:hover { background-color: #666; }
            QPushButton:disabled { background-color: #444; color: #777; }
        """)
        self._scan_sns_btn.clicked.connect(self._on_scan_sns)
        preset_row.addWidget(self._scan_sns_btn)
        ctrl_layout.addLayout(preset_row)

        # Start / Stop buttons.
        self._start_btn = QPushButton("Start Teleop")
        self._start_btn.setFixedHeight(55)
        self._start_btn.setFont(QFont("Arial", 14, QFont.Bold))
        self._start_btn.setStyleSheet("""
            QPushButton { background-color: #4CAF50; color: white; border-radius: 8px; }
            QPushButton:hover { background-color: #45a049; }
            QPushButton:disabled { background-color: #555; color: #888; }
        """)
        self._start_btn.clicked.connect(self._on_start)
        ctrl_layout.addWidget(self._start_btn)

        self._stop_btn = QPushButton("Stop Teleop")
        self._stop_btn.setFixedHeight(55)
        self._stop_btn.setFont(QFont("Arial", 14, QFont.Bold))
        self._stop_btn.setStyleSheet("""
            QPushButton { background-color: #C62828; color: white; border-radius: 8px; }
            QPushButton:hover { background-color: #B71C1C; }
            QPushButton:disabled { background-color: #555; color: #888; }
        """)
        self._stop_btn.setEnabled(False)
        self._stop_btn.clicked.connect(self._on_stop)
        ctrl_layout.addWidget(self._stop_btn)
        layout.addWidget(ctrl_group)

        # Joint preview (shared component).
        self._joint_panel = JointDisplayPanel()
        layout.addWidget(self._joint_panel)

        # Process output.
        log_group = QGroupBox("Process output")
        log_layout = QVBoxLayout(log_group)

        self._output_text = QPlainTextEdit()
        self._output_text.setReadOnly(True)
        self._output_text.setFont(QFont("Courier New", 9))
        self._output_text.setStyleSheet(LOG_TEXTEDIT_CSS)
        self._output_text.setPlaceholderText("Launch output appears here...")
        self._output_text.document().setMaximumBlockCount(self._MAX_LOG_LINES)
        log_layout.addWidget(self._output_text)

        clear_btn = QPushButton("Clear log")
        clear_btn.setFixedHeight(30)
        clear_btn.setStyleSheet("""
            QPushButton { background-color: #555; color: #ccc; border-radius: 4px; }
            QPushButton:hover { background-color: #666; }
        """)
        clear_btn.clicked.connect(self._clear_output)
        log_layout.addWidget(clear_btn)
        layout.addWidget(log_group, 1)

        self.setStyleSheet(DARK_THEME_CSS)

    # -------------------- Scan SNs --------------------

    def _on_scan_sns(self):
        dlg = ScanSNDialog(parent=self, log_callback=self._emit_output)
        dlg.exec_()

    # -------------------- ROS2 callbacks --------------------

    def _on_joint_state(self, side: str, msg):
        positions = list(msg.position[:7]) if len(msg.position) >= 7 else list(msg.position)
        self._joint_panel.update_joints(side, positions)

    def _on_arm_lifecycle(self, msg):
        # ROS2 spin thread — bounce to GUI thread.
        self._arm_lifecycle_signal.emit(int(msg.data))

    def _on_arm_lifecycle_main(self, lc: int):
        """Promote starting -> running once the controller stops being
        in-flight. Any 'done' state releases the Stop gate, including the
        failure ones — otherwise the operator can't recover when
        _do_enable raises (e.g. move_to_init residual too large)."""
        prev = self._arm_lifecycle
        self._arm_lifecycle = lc
        if prev != lc:
            self._emit_output(
                f">>> tianji_arm/lifecycle_state: "
                f"{_LC_NAMES.get(lc, str(lc))}"
            )
        if (self._state == "starting"
                and self._waiting_for_arm_lifecycle
                and lc in _LC_RELEASES_STOP_GATE):
            self._waiting_for_arm_lifecycle = False
            self._state_signal.emit("running")

    # -------------------- State management --------------------

    def _set_state(self, state: str):
        self._state = state
        #                       text,                   color,    start_en, config_en
        _states = {
            "stopped":   ("Teleop: stopped",        "#aaa",    True,  True),
            "starting":  ("Teleop: starting...",    "#F57C00", False, False),
            "running":   ("Teleop: running",        "#4CAF50", False, False),
            "stopping":  ("Teleop: stopping...",    "#F57C00", False, False),
        }
        text, color, start_en, config_en = _states.get(state, _states["stopped"])
        self._status_label.setText(text)
        self._status_label.setStyleSheet(f"color: {color};")
        self._start_btn.setEnabled(start_en)
        # Arm presets disable Stop only while we haven't heard a terminal
        # lifecycle state yet — once the controller reports ready / failed /
        # disabled / sdk_error, Stop must be clickable so the operator can
        # always recover.
        if state == "starting" and self._waiting_for_arm_lifecycle:
            self._stop_btn.setEnabled(False)
        else:
            self._stop_btn.setEnabled(state in ("starting", "running"))
        self._preset_combo.setEnabled(config_en)
        self._scan_sns_btn.setEnabled(config_en)
        self._joint_panel.set_phase(state)

        if state == "stopped":
            self._uptime_label.setText("")
            self._start_time = None
            self._liveness_timer.stop()
            self._waiting_for_arm_lifecycle = False
            self._arm_lifecycle = None
        elif state == "running":
            if self._start_time is None:
                self._start_time = self._launch_started_at or time.monotonic()
        elif state == "starting":
            self._liveness_timer.start()

        if state == "starting" and self._waiting_for_arm_lifecycle:
            self._status_label.setText("Teleop: starting (arm init...)")

    # -------------------- Backend process discovery --------------------

    def _find_backend_processes(self):
        try:
            output = subprocess.check_output(
                ["ps", "-eo", "pid=,pgid=,args="],
                text=True,
                stderr=subprocess.DEVNULL,
            )
        except Exception:
            return []

        processes = []
        for line in output.splitlines():
            if _BACKEND_MATCH not in line:
                continue
            parts = line.strip().split(None, 2)
            if len(parts) < 3:
                continue
            try:
                pid = int(parts[0])
                pgid = int(parts[1])
            except ValueError:
                continue
            processes.append({"pid": pid, "pgid": pgid, "args": parts[2]})
        return processes

    def _poll_process_alive(self):
        """Promote starting -> running once the subprocess survives a short
        warm-up window. Subprocess liveness is the only running signal."""
        if self._state != "starting":
            return
        proc = self._process
        if proc is None:
            return
        if proc.poll() is not None:
            # Process died during startup — let the reader thread emit the
            # transition to stopped; nothing to do here.
            return
        if self._launch_started_at is None:
            return
        if self._waiting_for_arm_lifecycle:
            # Arm presets: the latched /tianji_arm/lifecycle_state edge owns
            # the starting -> running promotion. No time-based fallback —
            # any terminal lifecycle (ready / disabled / enable_failed /
            # sdk_error) releases Stop, so the operator can always recover.
            return
        if time.monotonic() - self._launch_started_at >= 2.0:
            self._state_signal.emit("running")

    # -------------------- Launch --------------------

    def _build_launch_cmd(self, preset: str) -> list:
        args = LAUNCH_CONFIGS[preset]
        # `ros2 launch <package> <launch_file> <args...>`
        # bash -c lets us pick up overlay setup files if the user happens
        # to have one sourced.
        ros2_args = " ".join(args)
        launch_cmd = (
            f"exec ros2 launch wuji_teleop_bringup {ros2_args}"
        )
        return ["bash", "-c", launch_cmd]

    def _on_start(self):
        if self._state != "stopped":
            return
        existing = self._find_backend_processes()
        if existing:
            QMessageBox.warning(
                self, "Teleop already running",
                "An existing teleop launch was detected on this machine.\n\n"
                "Stop the other instance first, then try again.",
            )
            return

        reply = QMessageBox.warning(
            self, "Safety check",
            "The arms will enter impedance mode and move to their initial pose.\n\n"
            "Please confirm:\n"
            "  1. The workspace is clear of obstacles\n"
            "  2. The e-stop is within reach\n",
            QMessageBox.Ok | QMessageBox.Cancel,
            QMessageBox.Cancel)
        if reply != QMessageBox.Ok:
            return

        preset = self._preset_combo.currentText()
        self._waiting_for_arm_lifecycle = _preset_waits_for_arm_lifecycle(preset)
        self._arm_lifecycle = None
        cmd = self._build_launch_cmd(preset)

        self._set_state("starting")
        self._clear_output()
        self._launch_started_at = time.monotonic()
        self._emit_output(f">>> Preset: {preset}")
        self._emit_output(">>> " + " ".join(LAUNCH_CONFIGS[preset]))

        def _run():
            proc = None
            try:
                proc = subprocess.Popen(
                    cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    text=True, bufsize=1, preexec_fn=os.setsid)
                self._process = proc
                self._emit_output(">>> Launch command issued; waiting for teleop to initialize...")

                try:
                    for line in iter(proc.stdout.readline, ''):
                        if not line:
                            break
                        self._emit_output(line.rstrip())
                finally:
                    # Always close the stdout pipe — avoids FD leaks.
                    if proc and proc.stdout:
                        proc.stdout.close()

                retcode = proc.wait()
                if self._process is proc:
                    self._process = None
                self._emit_output(f"\n>>> Process exited (code: {retcode})")
                self._state_signal.emit("stopped")
            except Exception as e:
                self._emit_output(f"\n>>> Error: {e}")
                if proc and proc.stdout:
                    try:
                        proc.stdout.close()
                    except Exception:
                        pass
                if self._process is proc:
                    self._process = None
                self._state_signal.emit("stopped")

        threading.Thread(target=_run, daemon=True, name="teleop-launch").start()

    def _on_stop(self):
        if self._state in ("stopped", "stopping"):
            return
        if self._process is None and not self._find_backend_processes():
            self._set_state("stopped")
            return
        self._start_async_stop(close_after=False,
                               reason="Stopping teleop safely...")

    def _start_async_stop(self, close_after: bool, reason: str):
        if self._stop_in_progress:
            return
        self._stop_in_progress = True
        self._set_state("stopping")
        self._emit_output(f"\n>>> {reason}")
        self._emit_output(">>> Waiting for teleop to shut down (up to 30s)...")

        def _stop():
            ok = self._graceful_stop_sync()
            self._process = None
            self._stop_finished_signal.emit(ok, close_after)

        threading.Thread(target=_stop, daemon=True, name="teleop-stop").start()

    def _on_stop_finished(self, ok: bool, close_after: bool):
        self._stop_in_progress = False
        self._process = None
        if ok:
            self._emit_output(">>> Teleop stopped")
            self._state_signal.emit("stopped")
            if close_after:
                self.close()
            return

        # Residual processes — surface it but still drop to stopped so the
        # operator can retry.
        self._emit_output(">>> Stop incomplete: residual teleop processes remain; investigate manually")
        self._state_signal.emit("stopped")
        if close_after:
            QMessageBox.warning(
                self,
                "Residual teleop processes",
                "The teleop backend did not fully exit.\n\n"
                "Stop it again and confirm the controllers are down.",
            )
            self.close()

    def _emit_output(self, text: str):
        with self._output_lock:
            self._pending_output.append(text)

    def _clear_output(self):
        with self._output_lock:
            self._pending_output.clear()
        self._output_text.clear()

    def _flush_output(self):
        with self._output_lock:
            if not self._pending_output:
                return
            text = "\n".join(self._pending_output)
            self._pending_output.clear()
        self._output_text.appendPlainText(text)
        scrollbar = self._output_text.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    def _update_uptime(self):
        if self._start_time is None:
            return
        elapsed = int(time.monotonic() - self._start_time)
        h, m, s = elapsed // 3600, (elapsed % 3600) // 60, elapsed % 60
        self._uptime_label.setText(f"Uptime: {h:02d}:{m:02d}:{s:02d}")

    def _graceful_stop_sync(self) -> bool:
        pgids = set()

        proc = self._process
        if proc is not None and proc.poll() is None:
            try:
                pgids.add(os.getpgid(proc.pid))
            except (ProcessLookupError, OSError):
                pass

        for item in self._find_backend_processes():
            pgids.add(item["pgid"])

        if not pgids:
            return True

        self._emit_output(
            ">>> Sending SIGINT to teleop process groups: "
            + ", ".join(str(pgid) for pgid in sorted(pgids)))
        for pgid in sorted(pgids):
            try:
                os.killpg(pgid, signal.SIGINT)
            except (ProcessLookupError, OSError):
                pass

        deadline = time.monotonic() + 30.0
        remaining = set(pgids)
        while remaining and time.monotonic() < deadline:
            alive = {item["pgid"] for item in self._find_backend_processes()}
            remaining &= alive
            if not remaining:
                return True
            time.sleep(0.2)

        self._emit_output(
            ">>> Timed out (30s); sending SIGKILL to residual process groups: "
            + ", ".join(str(pgid) for pgid in sorted(remaining)))
        for pgid in sorted(remaining):
            try:
                os.killpg(pgid, signal.SIGKILL)
            except (ProcessLookupError, OSError):
                pass

        kill_deadline = time.monotonic() + 5.0
        while remaining and time.monotonic() < kill_deadline:
            alive = {item["pgid"] for item in self._find_backend_processes()}
            remaining &= alive
            if not remaining:
                return True
            time.sleep(0.2)

        if remaining:
            self._emit_output(
                ">>> Warning: process groups still alive: "
                + ", ".join(str(pgid) for pgid in sorted(remaining)))
            return False
        return True

    def closeEvent(self, event):
        if self._stop_in_progress or self._state == "stopping":
            event.ignore()
            return

        if self._state != "stopped":
            reply = QMessageBox.question(
                self, "Confirm exit",
                "Teleop is still running.\n\n"
                "Closing the window will stop it first.\n\n"
                "Stop and exit?",
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if reply == QMessageBox.No:
                event.ignore()
                return
            self._start_async_stop(
                close_after=True,
                reason="Closing window: stopping teleop safely...")
            event.ignore()
            return

        # Stop timers before widgets are destroyed.
        self._uptime_timer.stop()
        self._liveness_timer.stop()
        self._output_flush_timer.stop()
        self._flush_output()
        event.accept()


def _acquire_lock():
    try:
        fd = open(_LOCK_FILE, 'w')
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        fd.write(str(os.getpid()))
        fd.flush()
        return fd
    except (IOError, OSError):
        print("Error: Teleop Monitor is already running (another instance holds the lock file).")
        try:
            app = QApplication(sys.argv)
            QMessageBox.critical(
                None, "Wuji Teleop Monitor",
                "Teleop Monitor is already running!\n\n"
                "Only one instance is allowed at a time.\n"
                "Close the existing window first.")
        except Exception:
            pass
        sys.exit(1)


def main():
    lock_fd = _acquire_lock()

    app = QApplication(sys.argv)
    app.setStyle('Fusion')

    ros_node = None
    if _HAS_ROS2:
        try:
            rclpy.init()
            ros_node = Node('wuji_teleop_monitor')

            def _spin():
                try:
                    rclpy.spin(ros_node)
                except Exception:
                    pass

            threading.Thread(target=_spin, daemon=True, name="ros2-spin").start()
        except Exception as e:
            print(f"Warning: ROS2 init failed ({e}); joint preview disabled")
            ros_node = None

    window = TeleopLauncherUI(ros_node=ros_node)
    window.show()
    exit_code = app.exec_()

    # Graceful ROS2 shutdown: shutdown first so spin() returns, then destroy_node.
    if ros_node:
        try:
            rclpy.shutdown()
        except Exception:
            pass
        try:
            ros_node.destroy_node()
        except Exception:
            pass

    lock_fd.close()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()

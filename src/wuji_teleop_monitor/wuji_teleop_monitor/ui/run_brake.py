#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Brake Control — direct-SDK arm recovery tool.

Talks to the Tianji controller cabinet (default 192.168.1.190) directly via
the Tianji SDK; no ROS2 services, no controller process required. Use it
when teleop is OFF and you need to manually release / hold the brakes,
clear servo errors, or read state codes.

Marvin allows a single TCP session at a time — if teleop's
`tianji_arm_controller` is running it already owns the connection, so
this UI's Connect will fail with "port in use". Stop teleop first.

Usage:
    ros2 run wuji_teleop_monitor brake
"""

import fcntl
import os
import sys
import threading
import time
from pathlib import Path

from .qt_setup import setup_qt_plugins
setup_qt_plugins()

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QGroupBox, QMessageBox, QLineEdit, QFrame,
)
from PyQt5.QtCore import Qt, QTimer, QSize, pyqtSignal
from PyQt5.QtGui import QFont, QPixmap, QImage, QPainter
from PyQt5.QtSvg import QSvgRenderer

from .joint_panel import JointDisplayPanel
from .arm_constants import STATE_NAMES, ERR_DESCRIPTIONS
from .theme import (
    DARK_THEME_CSS, RELEASE_BUTTON_CSS, HOLD_BUTTON_CSS, READ_BUTTON_CSS,
    CLEAR_ERROR_BUTTON_CSS
)

_LOCK_FILE = "/tmp/wuji_teleop_brake.lock"
_DEFAULT_ROBOT_IP = "192.168.1.190"

# Logo lives one level up from the ui/ subpackage (same as monitor).
_LOGO_PATH = Path(__file__).resolve().parent.parent / "wuji_icon.png"


class BrakeControlUI(QMainWindow):
    """Brake control UI — direct SDK to the Tianji controller cabinet.

    All arm operations go through `TianjiChestDriver`:
      - release_brake / hold_brake  : SDK direct
      - clear_arm_error             : SDK direct
      - get_current_joints (30Hz)   : background thread, signal to UI
      - get_arm_status              : on-demand
    """

    _log_signal = pyqtSignal(str)
    _conn_signal = pyqtSignal(bool, str)        # (connected, message)
    _joints_signal = pyqtSignal(list, list)     # (left_joints, right_joints)
    _status_signal = pyqtSignal(object)         # arm_status dict
    _op_result_signal = pyqtSignal(str)         # log line from worker

    def __init__(self):
        super().__init__()
        # SDK state — only touched from the SDK lock-holder.
        self._driver = None
        self._sdk_lock = threading.Lock()
        self._connected = False

        # Background joint poll thread.
        self._poll_stop = threading.Event()
        self._poll_thread = None

        self._log_signal.connect(self._append_log)
        self._conn_signal.connect(self._on_connection_changed)
        self._joints_signal.connect(self._on_joints)
        self._status_signal.connect(self._update_status_display)
        self._op_result_signal.connect(self._append_log)

        self._init_ui()
        self._set_brake_buttons_enabled(False)

    # -------------------- Logo --------------------

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

    # -------------------- UI --------------------

    def _init_ui(self):
        self.setWindowTitle("Brake Control")
        self.setMinimumSize(640, 660)

        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        layout.setContentsMargins(15, 15, 15, 15)
        layout.setSpacing(12)

        # ---------- Header: logo + title strip ----------
        header = QHBoxLayout()
        header.setSpacing(10)
        if _LOGO_PATH.exists():
            logo_label = QLabel()
            pixmap = self._render_logo(_LOGO_PATH, target_height=36)
            if pixmap is not None and not pixmap.isNull():
                logo_label.setPixmap(pixmap)
            logo_label.setFixedHeight(40)
            header.addWidget(logo_label)

        title_frame = QFrame()
        title_frame.setFixedHeight(40)
        title_frame.setStyleSheet(
            "QFrame { background-color: #333; border-radius: 6px; }")
        title_layout = QHBoxLayout(title_frame)
        title_layout.setContentsMargins(12, 4, 12, 4)
        title_lbl = QLabel("Brake Control — Direct SDK")
        title_lbl.setFont(QFont("Arial", 13, QFont.Bold))
        title_lbl.setStyleSheet("color: #ddd;")
        title_layout.addWidget(title_lbl)
        title_layout.addStretch()
        header.addWidget(title_frame, 1)
        layout.addLayout(header)

        # ---------- Connection panel ----------
        conn_group = QGroupBox("Connection")
        conn_layout = QVBoxLayout(conn_group)
        conn_layout.setSpacing(6)

        route_lbl = QLabel(
            "Route: Direct SDK → Tianji controller cabinet (no ROS2)")
        route_lbl.setStyleSheet("color: #888; font-size: 11px;")
        conn_layout.addWidget(route_lbl)

        ip_row = QHBoxLayout()
        ip_lbl = QLabel("Robot IP:")
        ip_lbl.setFixedWidth(80)
        ip_lbl.setStyleSheet("color: #ccc;")
        ip_row.addWidget(ip_lbl)

        self._ip_edit = QLineEdit(_DEFAULT_ROBOT_IP)
        self._ip_edit.setFixedWidth(160)
        self._ip_edit.setStyleSheet(
            "QLineEdit { background-color: #1e1e1e; color: #ddd;"
            " border: 1px solid #555; border-radius: 4px; padding: 4px 8px; }")
        ip_row.addWidget(self._ip_edit)

        self._connect_btn = QPushButton("Connect")
        self._connect_btn.setFixedHeight(28)
        self._connect_btn.setFixedWidth(110)
        self._connect_btn.setStyleSheet(READ_BUTTON_CSS)
        self._connect_btn.clicked.connect(self._on_connect_clicked)
        ip_row.addWidget(self._connect_btn)

        self._disconnect_btn = QPushButton("Disconnect")
        self._disconnect_btn.setFixedHeight(28)
        self._disconnect_btn.setFixedWidth(110)
        self._disconnect_btn.setStyleSheet(HOLD_BUTTON_CSS)
        self._disconnect_btn.setEnabled(False)
        self._disconnect_btn.clicked.connect(self._on_disconnect_clicked)
        ip_row.addWidget(self._disconnect_btn)

        ip_row.addStretch()
        conn_layout.addLayout(ip_row)

        self._conn_status_label = QLabel("Status: disconnected")
        self._conn_status_label.setStyleSheet(
            "color: #888; font-family: 'Courier New'; font-size: 11px;")
        conn_layout.addWidget(self._conn_status_label)

        warn = QLabel(
            "Stop teleop first — Marvin allows only one TCP session at a time.")
        warn.setStyleSheet("color: #F57C00; font-size: 10px;")
        warn.setWordWrap(True)
        conn_layout.addWidget(warn)
        layout.addWidget(conn_group)

        # ---------- Joint display ----------
        self._joint_panel = JointDisplayPanel()
        layout.addWidget(self._joint_panel)

        # ---------- Brake operations ----------
        brake_group = QGroupBox("Brake Operations")
        brake_layout = QVBoxLayout(brake_group)
        brake_layout.setSpacing(8)

        self._brake_buttons = []
        for side_label, side_key, color in [
            ("Left Arm", "left", "#4FC3F7"),
            ("Right Arm", "right", "#FF8A65"),
        ]:
            row = QHBoxLayout()
            lbl = QLabel(side_label)
            lbl.setFixedWidth(90)
            lbl.setStyleSheet(f"color: {color}; font-size: 12px; font-weight: bold;")
            row.addWidget(lbl)

            release_btn = QPushButton("Release")
            release_btn.setFixedWidth(120)
            release_btn.setStyleSheet(RELEASE_BUTTON_CSS)
            release_btn.setCursor(Qt.PointingHandCursor)
            release_btn.clicked.connect(
                lambda checked, s=side_key: self._on_release(s))
            row.addWidget(release_btn)
            self._brake_buttons.append(release_btn)

            hold_btn = QPushButton("Hold")
            hold_btn.setFixedWidth(120)
            hold_btn.setStyleSheet(HOLD_BUTTON_CSS)
            hold_btn.setCursor(Qt.PointingHandCursor)
            hold_btn.clicked.connect(
                lambda checked, s=side_key: self._on_hold(s))
            row.addWidget(hold_btn)
            self._brake_buttons.append(hold_btn)

            clear_btn = QPushButton("Clear Error")
            clear_btn.setFixedWidth(120)
            clear_btn.setStyleSheet(CLEAR_ERROR_BUTTON_CSS)
            clear_btn.setCursor(Qt.PointingHandCursor)
            clear_btn.clicked.connect(
                lambda checked, s=side_key: self._on_clear_error(s))
            row.addWidget(clear_btn)
            self._brake_buttons.append(clear_btn)

            row.addStretch()
            brake_layout.addLayout(row)

        # State switch row (both arms switch together — SDK API is global).
        state_row_caption = QLabel("Arm state (both arms)")
        state_row_caption.setStyleSheet(
            "color: #aaa; font-size: 11px; padding-top: 6px;")
        brake_layout.addWidget(state_row_caption)

        state_row = QHBoxLayout()
        state_row.addSpacing(98)  # align with the per-arm rows above

        standby_btn = QPushButton("Standby (servo off)")
        standby_btn.setFixedWidth(200)
        standby_btn.setStyleSheet(RELEASE_BUTTON_CSS)
        standby_btn.setCursor(Qt.PointingHandCursor)
        standby_btn.clicked.connect(self._on_set_standby)
        state_row.addWidget(standby_btn)
        self._brake_buttons.append(standby_btn)

        position_btn = QPushButton("Position mode")
        position_btn.setFixedWidth(200)
        position_btn.setStyleSheet(READ_BUTTON_CSS)
        position_btn.setCursor(Qt.PointingHandCursor)
        position_btn.clicked.connect(self._on_set_position_mode)
        state_row.addWidget(position_btn)
        self._brake_buttons.append(position_btn)

        impedance_btn = QPushButton("Impedance mode")
        impedance_btn.setFixedWidth(200)
        impedance_btn.setStyleSheet(READ_BUTTON_CSS)
        impedance_btn.setCursor(Qt.PointingHandCursor)
        impedance_btn.clicked.connect(self._on_set_impedance_mode)
        state_row.addWidget(impedance_btn)
        self._brake_buttons.append(impedance_btn)

        state_row.addStretch()
        brake_layout.addLayout(state_row)

        warn = QLabel(
            "WARNING: releasing the brake lets the arm drop under gravity. "
            "Keep the e-stop within reach.")
        warn.setStyleSheet("color: #F57C00; font-size: 10px; padding: 4px;")
        brake_layout.addWidget(warn)
        layout.addWidget(brake_group)

        # ---------- Arm status ----------
        status_group = QGroupBox("Arm Status")
        status_layout = QVBoxLayout(status_group)
        status_layout.setSpacing(4)

        self._left_status_label = QLabel("Left  --")
        self._left_status_label.setFont(QFont("Courier New", 10))
        self._left_status_label.setStyleSheet("color: #888;")
        status_layout.addWidget(self._left_status_label)

        self._right_status_label = QLabel("Right  --")
        self._right_status_label.setFont(QFont("Courier New", 10))
        self._right_status_label.setStyleSheet("color: #888;")
        status_layout.addWidget(self._right_status_label)

        self._err_desc_label = QLabel("")
        self._err_desc_label.setStyleSheet("color: #F57C00; font-size: 10px;")
        self._err_desc_label.setWordWrap(True)
        status_layout.addWidget(self._err_desc_label)

        self._read_status_btn = QPushButton("Read Status")
        self._read_status_btn.setFixedWidth(140)
        self._read_status_btn.setStyleSheet(READ_BUTTON_CSS)
        self._read_status_btn.setCursor(Qt.PointingHandCursor)
        self._read_status_btn.clicked.connect(self._on_read_status)
        status_layout.addWidget(self._read_status_btn, alignment=Qt.AlignCenter)
        layout.addWidget(status_group)

        # ---------- Log ----------
        log_group = QGroupBox("Operation Log")
        log_layout = QVBoxLayout(log_group)
        self._log_label = QLabel("Ready — connect to begin.")
        self._log_label.setFont(QFont("Courier New", 9))
        self._log_label.setStyleSheet("color: #aaa;")
        self._log_label.setWordWrap(True)
        self._log_label.setMinimumHeight(40)
        log_layout.addWidget(self._log_label)
        layout.addWidget(log_group)

        self.setStyleSheet(DARK_THEME_CSS)

    # -------------------- Connection --------------------

    def _set_brake_buttons_enabled(self, enabled: bool):
        for btn in self._brake_buttons:
            btn.setEnabled(enabled)
        self._read_status_btn.setEnabled(enabled)

    def _on_connect_clicked(self):
        robot_ip = self._ip_edit.text().strip() or _DEFAULT_ROBOT_IP
        reply = QMessageBox.warning(
            self, "Confirm direct SDK connect",
            f"Open a direct TCP session to {robot_ip}?\n\n"
            f"Marvin allows only one client at a time. If a teleop\n"
            f"controller is currently using the arm, this Connect will\n"
            f"fail with a 'port in use' error — stop teleop first.\n",
            QMessageBox.Ok | QMessageBox.Cancel,
            QMessageBox.Cancel)
        if reply != QMessageBox.Ok:
            return

        self._connect_btn.setEnabled(False)
        self._ip_edit.setEnabled(False)
        self._log_signal.emit(f"Connecting to {robot_ip}...")
        self._conn_status_label.setText(f"Status: connecting to {robot_ip}...")
        self._conn_status_label.setStyleSheet(
            "color: #F57C00; font-family: 'Courier New'; font-size: 11px;")

        def _worker():
            try:
                from tianji_output import TianjiChestDriver
                driver = TianjiChestDriver(robot_ip=robot_ip)
                with self._sdk_lock:
                    self._driver = driver
                    self._connected = True
                self._conn_signal.emit(True, f"connected to {robot_ip}")
            except Exception as e:
                self._conn_signal.emit(False, f"connect failed: {e}")

        threading.Thread(target=_worker, daemon=True, name="brake-connect").start()

    def _on_disconnect_clicked(self):
        if not self._connected:
            return
        self._disconnect_btn.setEnabled(False)
        self._log_signal.emit("Disconnecting...")

        def _worker():
            self._stop_poll_thread()
            with self._sdk_lock:
                driver = self._driver
                self._driver = None
                self._connected = False
            if driver is not None:
                try:
                    driver.disable_and_release()
                except Exception as e:
                    self._op_result_signal.emit(f"Disconnect warning: {e}")
            self._conn_signal.emit(False, "disconnected")

        threading.Thread(target=_worker, daemon=True, name="brake-disconnect").start()

    def _on_connection_changed(self, connected: bool, message: str):
        self._log_label.setText(message)
        if connected:
            self._conn_status_label.setText(f"Status: {message}")
            self._conn_status_label.setStyleSheet(
                "color: #4CAF50; font-family: 'Courier New'; font-size: 11px;")
            self._connect_btn.setEnabled(False)
            self._disconnect_btn.setEnabled(True)
            self._ip_edit.setEnabled(False)
            self._set_brake_buttons_enabled(True)
            self._start_poll_thread()
        else:
            self._conn_status_label.setText(f"Status: {message}")
            self._conn_status_label.setStyleSheet(
                "color: #888; font-family: 'Courier New'; font-size: 11px;")
            self._connect_btn.setEnabled(True)
            self._disconnect_btn.setEnabled(False)
            self._ip_edit.setEnabled(True)
            self._set_brake_buttons_enabled(False)

    # -------------------- Joint polling --------------------

    def _start_poll_thread(self):
        self._poll_stop.clear()
        self._poll_thread = threading.Thread(
            target=self._poll_loop, daemon=True, name="brake-joint-poll")
        self._poll_thread.start()

    def _stop_poll_thread(self):
        self._poll_stop.set()
        t = self._poll_thread
        if t is not None and t.is_alive():
            t.join(timeout=1.0)
        self._poll_thread = None

    def _poll_loop(self):
        """Background poll: SDK subscribe -> joint signal. 30Hz."""
        interval = 1.0 / 30.0
        while not self._poll_stop.is_set():
            with self._sdk_lock:
                driver = self._driver if self._connected else None
            if driver is None:
                break
            try:
                left, right = driver.get_current_joints()
                self._joints_signal.emit(list(left), list(right))
            except Exception as e:
                self._op_result_signal.emit(f"Joint poll error: {e}")
                # Brief pause to avoid log flooding on a dead socket.
                time.sleep(0.5)
                continue
            time.sleep(interval)

    def _on_joints(self, left_joints: list, right_joints: list):
        self._joint_panel.update_joints('left', list(left_joints)[:7])
        self._joint_panel.update_joints('right', list(right_joints)[:7])

    # -------------------- Brake actions --------------------

    def _on_release(self, side: str):
        arm_label = "Left" if side == "left" else "Right"
        reply = QMessageBox.warning(
            self, "Release Confirmation",
            f"Release the {arm_label.lower()}-arm brake?\n\n"
            f"The arm will drop under gravity after release.\n"
            f"Keep the e-stop within reach.",
            QMessageBox.Ok | QMessageBox.Cancel,
            QMessageBox.Cancel)
        if reply != QMessageBox.Ok:
            return
        self._call_sdk_brake(side, release=True)

    def _on_hold(self, side: str):
        self._call_sdk_brake(side, release=False)

    def _call_sdk_brake(self, side: str, release: bool):
        arm_label = "Left" if side == "left" else "Right"
        arm_code = 'A' if side == "left" else 'B'
        action = "release" if release else "hold"
        self._log_signal.emit(f"{arm_label}: {action} in progress...")

        def _worker():
            with self._sdk_lock:
                driver = self._driver if self._connected else None
            if driver is None:
                self._op_result_signal.emit(f"{arm_label}: not connected")
                return
            try:
                if release:
                    driver.release_brake(arm_code)
                else:
                    driver.hold_brake(arm_code)
                self._op_result_signal.emit(f"{arm_label}: {action} ok")
            except Exception as e:
                self._op_result_signal.emit(f"{arm_label}: {action} error: {e}")

        threading.Thread(target=_worker, daemon=True, name=f"brake-{side}").start()

    def _on_clear_error(self, side: str):
        arm_label = "Left" if side == "left" else "Right"
        arm_code = 'A' if side == "left" else 'B'
        self._log_signal.emit(f"{arm_label}: clearing error...")

        def _worker():
            with self._sdk_lock:
                driver = self._driver if self._connected else None
            if driver is None:
                self._op_result_signal.emit(f"{arm_label}: not connected")
                return
            try:
                driver.clear_arm_error(arm_code)
                self._op_result_signal.emit(f"{arm_label}: clear-error ok")
            except Exception as e:
                self._op_result_signal.emit(
                    f"{arm_label}: clear-error error: {e}")

        threading.Thread(target=_worker, daemon=True,
                         name=f"clear-error-{side}").start()

    def _on_set_standby(self):
        reply = QMessageBox.warning(
            self, "Confirm servo-off",
            "Drop both arms to servo-off (cur_state=0)?\n\n"
            "The arms will lose holding torque. If no brake is held, "
            "they will collapse under gravity.\n"
            "Keep the e-stop within reach.",
            QMessageBox.Ok | QMessageBox.Cancel,
            QMessageBox.Cancel)
        if reply != QMessageBox.Ok:
            return
        self._run_state_switch("Standby", lambda d: d.set_standby())

    def _on_set_position_mode(self):
        self._run_state_switch(
            "Position mode",
            lambda d: d.set_position_mode(velRatio=30, AccRatio=30))

    def _on_set_impedance_mode(self):
        self._run_state_switch(
            "Impedance mode", lambda d: d.set_impedance_mode(mode='joint'))

    def _run_state_switch(self, label: str, action):
        self._log_signal.emit(f"{label}: switching...")

        def _worker():
            with self._sdk_lock:
                driver = self._driver if self._connected else None
            if driver is None:
                self._op_result_signal.emit(f"{label}: not connected")
                return
            try:
                action(driver)
                self._op_result_signal.emit(f"{label}: ok")
            except Exception as e:
                self._op_result_signal.emit(f"{label}: error: {e}")

        threading.Thread(target=_worker, daemon=True,
                         name=f"state-{label.lower().split()[0]}").start()

    # -------------------- Status read --------------------

    def _on_read_status(self):
        self._log_signal.emit("Reading status...")

        def _worker():
            with self._sdk_lock:
                driver = self._driver if self._connected else None
            if driver is None:
                self._op_result_signal.emit("Read status: not connected")
                return
            try:
                status = driver.get_arm_status()
                self._status_signal.emit(status)
                self._op_result_signal.emit("Read status: ok")
            except Exception as e:
                self._op_result_signal.emit(f"Read status: error: {e}")

        threading.Thread(target=_worker, daemon=True, name="read-status").start()

    def _update_status_display(self, data: dict):
        err_descs = []
        for side_key, label, arm_name in [
            ('left', self._left_status_label, 'Left'),
            ('right', self._right_status_label, 'Right'),
        ]:
            info = data[side_key]
            state = info['state']
            err_code = info['err_code']
            servo_errors = info['servo_errors']
            servo_descs = info['servo_descriptions']

            state_name = STATE_NAMES.get(state, f"unknown({state})")
            non_zero = [
                (i + 1, e, servo_descs[i])
                for i, e in enumerate(servo_errors)
                if e != '0x0000'
            ]
            if non_zero:
                servo_str = " ".join(
                    f"J{j}={e}({d})" if d else f"J{j}={e}"
                    for j, e, d in non_zero
                )
            else:
                servo_str = "all OK"

            label.setText(
                f"{arm_name}  state: {state_name}({state})  "
                f"err_code: {err_code}  servos: {servo_str}")
            if err_code != 0:
                label.setStyleSheet("color: #FF1744; font-weight: bold;")
                err_descs.append(
                    f"err_code={err_code}: "
                    f"{ERR_DESCRIPTIONS.get(err_code, f'unknown({err_code})')}")
            else:
                label.setStyleSheet("color: #4CAF50;")

        if err_descs:
            self._err_desc_label.setText("\n".join(err_descs))
            self._err_desc_label.setStyleSheet("color: #FF1744; font-size: 10px;")
        else:
            self._err_desc_label.setText("Both arms OK")
            self._err_desc_label.setStyleSheet("color: #4CAF50; font-size: 10px;")

    def _append_log(self, text: str):
        self._log_label.setText(text)

    # -------------------- Close --------------------

    def closeEvent(self, event):
        if self._connected:
            reply = QMessageBox.question(
                self, "Confirm exit",
                "Direct SDK session is still open.\n\n"
                "Disconnect and exit?",
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if reply == QMessageBox.No:
                event.ignore()
                return

        self._stop_poll_thread()
        with self._sdk_lock:
            driver = self._driver
            self._driver = None
            self._connected = False
        if driver is not None:
            try:
                driver.disable_and_release()
            except Exception:
                pass
        event.accept()


def _acquire_lock():
    try:
        fd = open(_LOCK_FILE, 'w')
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        fd.write(str(os.getpid()))
        fd.flush()
        return fd
    except (IOError, OSError):
        print("Error: Brake Control UI is already running "
              "(another instance holds the lock file).")
        try:
            app = QApplication(sys.argv)
            QMessageBox.critical(
                None, "Brake Control",
                "Brake Control UI is already running!\n\n"
                "Only one instance is allowed at a time.\n"
                "Close the existing window first.")
        except Exception:
            pass
        sys.exit(1)


def main():
    lock_fd = _acquire_lock()

    app = QApplication(sys.argv)
    app.setStyle('Fusion')

    window = BrakeControlUI()
    window.show()
    exit_code = app.exec_()

    lock_fd.close()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()

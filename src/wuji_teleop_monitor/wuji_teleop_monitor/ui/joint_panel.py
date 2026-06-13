# -*- coding: utf-8 -*-
"""Reusable arm joint-angle display panel.

Design goals:
- Serve the existing Teleop / Brake UIs only.
- ROS2 high-rate callbacks update caches; they never drive Qt events directly.
- The Qt main thread refreshes at a fixed cadence to keep the UI smooth.
- Connection state to the launch process is derived from joint-feedback
  freshness; it does not paper over stalls or disconnects.
- During brief startup/stop transitions, the panel keeps the last frame to
  avoid flicker.
"""

import threading
import time
from collections import deque

from PyQt5.QtWidgets import QGroupBox, QVBoxLayout, QHBoxLayout, QLabel, QFrame, QComboBox
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont

HAND_FINGERS = ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky']
HAND_JOINTS = ['MCP', 'PIP', 'DIP', 'Tip']

from .arm_constants import INIT_LEFT, INIT_RIGHT


class JointDisplayPanel(QGroupBox):
    """Joint-angle display panel — embeddable in any QMainWindow."""

    RENDER_INTERVAL_MS = 100
    STALE_THRESHOLD = 2.0  # seconds
    TRANSITION_GRACE = 5.0  # keep last frame briefly during startup/stop
    STYLE_NORMAL = "color: #ddd;"
    STYLE_ZERO = "color: #FF1744; font-weight: bold;"
    STYLE_STALE = "color: #F57C00;"
    STYLE_TRANSITION = "color: #FFB74D;"
    STYLE_IDLE = "color: #888;"
    STYLE_HZ_OK = "color: #4CAF50; font-weight: bold;"

    def __init__(self, title: str = "Arm Joint Angles", parent=None):
        super().__init__(title, parent)

        self._lock = threading.Lock()
        self._phase = "running"
        self._latest_joints = {"left": None, "right": None}
        self._last_msg_time = {"left": 0.0, "right": 0.0}
        # Hand-joint caches (20 joints per hand).
        self._hand_joints = {"left": None, "right": None}
        self._hand_msg_time = {"left": 0.0, "right": 0.0}
        self._hand_joint_idx = 0  # default MCP (0=MCP, 1=PIP, 2=DIP, 3=Tip)

        # Cmd Hz monitor: sliding 1s window of command-message timestamps.
        # len(stamps within the last 1.0s) == measured publish rate (Hz).
        self._hz_stamps = {
            'left_arm': deque(maxlen=400),
            'right_arm': deque(maxlen=400),
            'left_hand': deque(maxlen=400),
            'right_hand': deque(maxlen=400),
        }
        self._hz_labels = {}

        self._build_ui()

        self._render_timer = QTimer(self)
        self._render_timer.timeout.connect(self._refresh_display)
        self._render_timer.start(self.RENDER_INTERVAL_MS)

    def _build_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(4)

        lbl_width = 78
        val_width = 62
        hz_width = 60

        # Header
        header = QHBoxLayout()
        header.setSpacing(4)
        hdr = QLabel("Joint")
        hdr.setFixedWidth(lbl_width)
        hdr.setStyleSheet("color: #888; font-size: 10px; font-weight: bold;")
        header.addWidget(hdr)
        for i in range(1, 8):
            lbl = QLabel(f"J{i}")
            lbl.setFixedWidth(val_width)
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setStyleSheet("color: #888; font-size: 10px; font-weight: bold;")
            header.addWidget(lbl)
        hz_hdr = QLabel("Cmd Hz")
        hz_hdr.setFixedWidth(hz_width)
        hz_hdr.setAlignment(Qt.AlignCenter)
        hz_hdr.setStyleSheet("color: #888; font-size: 10px; font-weight: bold;")
        header.addWidget(hz_hdr)
        header.addStretch()
        layout.addLayout(header)

        # Left-arm row
        self._left_labels = self._add_arm_row(
            layout, "Left Arm", "#4FC3F7", lbl_width, val_width, hz_width, 'left_arm')

        # Right-arm row
        self._right_labels = self._add_arm_row(
            layout, "Right Arm", "#FF8A65", lbl_width, val_width, hz_width, 'right_arm')

        # Hand header
        self._add_hand_header(layout, lbl_width, val_width)

        # Left-hand row
        self._left_hand_labels = self._add_hand_row(
            layout, "Left Hand", "#81C784", lbl_width, val_width, hz_width, 'left_hand')

        # Right-hand row
        self._right_hand_labels = self._add_hand_row(
            layout, "Right Hand", "#FFB74D", lbl_width, val_width, hz_width, 'right_hand')

        # Separator
        sep = QFrame()
        sep.setFrameShape(QFrame.HLine)
        sep.setStyleSheet("color: #444;")
        layout.addWidget(sep)

        # Reference-value rows
        ref_style = "color: #666; font-size: 9px;"
        val_style = "color: #777;"
        for label_text, init_vals in [("Left ref", INIT_LEFT), ("Right ref", INIT_RIGHT)]:
            row = QHBoxLayout()
            row.setSpacing(4)
            lbl = QLabel(label_text)
            lbl.setFixedWidth(lbl_width)
            lbl.setStyleSheet(ref_style)
            row.addWidget(lbl)
            for v in init_vals:
                vlbl = QLabel(f"{v:.0f}")
                vlbl.setFixedWidth(val_width)
                vlbl.setAlignment(Qt.AlignCenter)
                vlbl.setFont(QFont("Courier", 9))
                vlbl.setStyleSheet(val_style)
                row.addWidget(vlbl)
            row.addStretch()
            layout.addLayout(row)

    def _add_arm_row(self, parent_layout, name, color, lbl_width, val_width, hz_width, hz_key):
        row = QHBoxLayout()
        row.setSpacing(4)
        lbl = QLabel(name)
        lbl.setFixedWidth(lbl_width)
        lbl.setStyleSheet(f"color: {color}; font-size: 11px; font-weight: bold;")
        row.addWidget(lbl)
        labels = []
        for _ in range(7):
            jlbl = QLabel("--")
            jlbl.setFixedWidth(val_width)
            jlbl.setAlignment(Qt.AlignCenter)
            jlbl.setFont(QFont("Courier", 10))
            jlbl.setStyleSheet(self.STYLE_IDLE)
            jlbl.setProperty("_wuji_text", "--")
            jlbl.setProperty("_wuji_style", self.STYLE_IDLE)
            row.addWidget(jlbl)
            labels.append(jlbl)
        self._add_hz_label(row, hz_width, hz_key)
        row.addStretch()
        parent_layout.addLayout(row)
        return labels

    def _add_hz_label(self, row, hz_width, hz_key):
        """Add a Cmd Hz cell to a data row and register it under hz_key."""
        hz_lbl = QLabel("--")
        hz_lbl.setFixedWidth(hz_width)
        hz_lbl.setAlignment(Qt.AlignCenter)
        hz_lbl.setFont(QFont("Courier", 10, QFont.Bold))
        hz_lbl.setStyleSheet(self.STYLE_IDLE)
        hz_lbl.setProperty("_wuji_text", "--")
        hz_lbl.setProperty("_wuji_style", self.STYLE_IDLE)
        row.addWidget(hz_lbl)
        self._hz_labels[hz_key] = hz_lbl

    def _add_hand_header(self, parent_layout, lbl_width, val_width):
        """Hand header: [Joint v] Thumb Index Middle Ring Pinky [pad]."""
        _combo_style = (
            'QComboBox { background: #3a3a3a; color: #ccc; border: 1px solid #555;'
            ' border-radius: 3px; padding: 1px 4px; font-size: 10px; }'
            'QComboBox::drop-down { border: none; }'
            'QComboBox QAbstractItemView { background: #333; color: #ccc;'
            ' selection-background-color: #555; }'
        )
        row = QHBoxLayout()
        row.setSpacing(4)
        hdr = QLabel("Joint")
        hdr.setFixedWidth(lbl_width)
        hdr.setStyleSheet("color: #888; font-size: 10px; font-weight: bold;")
        row.addWidget(hdr)
        self._hand_joint_combo = QComboBox()
        self._hand_joint_combo.setFixedWidth(val_width)
        self._hand_joint_combo.addItems(HAND_JOINTS)
        self._hand_joint_combo.setCurrentIndex(self._hand_joint_idx)
        self._hand_joint_combo.currentIndexChanged.connect(self._on_hand_joint_changed)
        self._hand_joint_combo.setStyleSheet(_combo_style)
        row.addWidget(self._hand_joint_combo)
        for fname in HAND_FINGERS:
            lbl = QLabel(fname)
            lbl.setFixedWidth(val_width)
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setStyleSheet("color: #888; font-size: 10px; font-weight: bold;")
            row.addWidget(lbl)
        # Pad to align with the arm row (combo + 5 fingers = 6, arm has 7 columns).
        sp = QLabel("")
        sp.setFixedWidth(val_width)
        row.addWidget(sp)
        row.addStretch()
        parent_layout.addLayout(row)

    def _add_hand_row(self, parent_layout, name, color, lbl_width, val_width, hz_width, hz_key):
        """Hand data row: [label] [pad] 5 finger values [pad] [Cmd Hz]."""
        row = QHBoxLayout()
        row.setSpacing(4)
        lbl = QLabel(name)
        lbl.setFixedWidth(lbl_width)
        lbl.setStyleSheet(f"color: {color}; font-size: 11px; font-weight: bold;")
        row.addWidget(lbl)
        # Combo placeholder column
        sp0 = QLabel("")
        sp0.setFixedWidth(val_width)
        row.addWidget(sp0)
        # Five finger-angle labels
        labels = []
        for _ in range(5):
            jlbl = QLabel("--")
            jlbl.setFixedWidth(val_width)
            jlbl.setAlignment(Qt.AlignCenter)
            jlbl.setFont(QFont("Courier", 10))
            jlbl.setStyleSheet(self.STYLE_IDLE)
            jlbl.setProperty("_wuji_text", "--")
            jlbl.setProperty("_wuji_style", self.STYLE_IDLE)
            row.addWidget(jlbl)
            labels.append(jlbl)
        # Pad to align with the arm row.
        sp1 = QLabel("")
        sp1.setFixedWidth(val_width)
        row.addWidget(sp1)
        self._add_hz_label(row, hz_width, hz_key)
        row.addStretch()
        parent_layout.addLayout(row)
        return labels

    def _on_hand_joint_changed(self, index):
        self._hand_joint_idx = index

    # -------------------- Public API --------------------

    def update_joints(self, side: str, positions: list):
        """Thread-safe arm-joint cache update (called from ROS2 callbacks)."""
        clipped = list(positions[:7]) if len(positions) >= 7 else list(positions)
        with self._lock:
            self._latest_joints[side] = clipped
            self._last_msg_time[side] = time.monotonic()

    def update_hand_joints(self, side: str, positions: list):
        """Thread-safe hand-joint cache update (called from ROS2 callbacks, 20 joints)."""
        with self._lock:
            self._hand_joints[side] = list(positions[:20]) if len(positions) >= 20 else list(positions)
            self._hand_msg_time[side] = time.monotonic()

    def record_hz(self, key: str):
        """Record one command-message arrival (called from ROS2 callbacks).

        key is one of 'left_arm' / 'right_arm' / 'left_hand' / 'right_hand'.
        Cheap and thread-safe enough: deque.append is atomic under the GIL,
        and the render thread only reads/pops — no lock needed for this.
        """
        stamps = self._hz_stamps.get(key)
        if stamps is not None:
            stamps.append(time.monotonic())

    def set_phase(self, phase: str):
        """Set the panel phase: stopped / starting / running / startup_timeout / stopping."""
        self._phase = phase
        self._refresh_display()

    # -------------------- Internal --------------------

    def _arm_feedback_state(self, positions, last_time: float, now: float):
        """Return (display_positions, style). display_positions is None when
        the cell should render '--' (no feedback yet, or stale beyond grace)."""
        if positions is None or last_time <= 0:
            return None, self.STYLE_IDLE

        age = now - last_time
        if age <= self.STALE_THRESHOLD:
            if all(abs(v) < 0.01 for v in positions):
                return positions, self.STYLE_ZERO
            return positions, self.STYLE_NORMAL

        if self._phase in ("starting", "stopping") and age <= self.TRANSITION_GRACE:
            return positions, self.STYLE_TRANSITION

        return None, self.STYLE_STALE

    def _refresh_display(self):
        with self._lock:
            snapshot = {
                "left": (
                    None if self._latest_joints["left"] is None else list(self._latest_joints["left"]),
                    self._last_msg_time["left"],
                ),
                "right": (
                    None if self._latest_joints["right"] is None else list(self._latest_joints["right"]),
                    self._last_msg_time["right"],
                ),
            }

        now = time.monotonic()
        for side, labels in [("left", self._left_labels), ("right", self._right_labels)]:
            positions, last_time = snapshot[side]
            display_positions, style = self._arm_feedback_state(positions, last_time, now)
            if display_positions is None:
                for lbl in labels:
                    self._set_label_state(lbl, "--", style)
                continue
            for i, lbl in enumerate(labels):
                if i < len(display_positions):
                    self._set_label_state(lbl, f"{display_positions[i]:.1f}", style)
                else:
                    self._set_label_state(lbl, "--", self.STYLE_IDLE)

        # Hand-joint angles: selected joint type x 5 fingers.
        joint_offset = self._hand_joint_idx  # 0=MCP, 1=PIP, 2=DIP, 3=Tip
        for side, labels in [("left", self._left_hand_labels), ("right", self._right_hand_labels)]:
            with self._lock:
                hand_pos = self._hand_joints[side]
                hand_time = self._hand_msg_time[side]
            display_positions, style = self._arm_feedback_state(hand_pos, hand_time, now)
            if display_positions is None:
                for lbl in labels:
                    self._set_label_state(lbl, "--", style)
            else:
                for finger_idx, lbl in enumerate(labels):
                    idx = finger_idx * 4 + joint_offset  # 4 joints per finger
                    if idx < len(display_positions):
                        self._set_label_state(lbl, f"{display_positions[idx]:.2f}", style)
                    else:
                        self._set_label_state(lbl, "--", self.STYLE_IDLE)

        # Cmd Hz: messages received in the last 1.0s == measured publish rate.
        for key, stamps in self._hz_stamps.items():
            while stamps and (now - stamps[0]) > 1.0:
                stamps.popleft()
            lbl = self._hz_labels.get(key)
            if lbl is None:
                continue
            hz = len(stamps)
            if hz == 0:
                self._set_label_state(lbl, "--", self.STYLE_IDLE)
            else:
                self._set_label_state(lbl, f"{hz}Hz", self.STYLE_HZ_OK)

    def _set_label_state(self, label: QLabel, text: str, style: str):
        if label.property("_wuji_text") != text:
            label.setText(text)
            label.setProperty("_wuji_text", text)
        if label.property("_wuji_style") != style:
            label.setStyleSheet(style)
            label.setProperty("_wuji_style", style)

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Camera Preview — four live camera feeds (stereo head + dual wrist), 2x2.

Layout: stereo head left/right eye (top row) + left/right wrist cameras
(bottom row).
Topics: /stereo/{left,right}/compressed  (stereo head, two eyes),
        /cam_{left,right}_wrist/color/image_raw/compressed  (wrist D405s).

Usage:
    python3 run_camera.py
"""

import fcntl
import os
import sys
import threading
from pathlib import Path

# Shared UI components (same directory).
_TELEOP_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_TELEOP_DIR))
import time

# Qt plugin path (must be set before any PyQt5 import).
try:
    from .qt_setup import setup_qt_plugins
    setup_qt_plugins()
except Exception:
    pass

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QFrame, QMessageBox, QGridLayout,
)
from PyQt5.QtCore import Qt, pyqtSignal, QThread, QTimer, QSize
from PyQt5.QtGui import QPixmap, QImage, QFont, QPainter
from PyQt5.QtSvg import QSvgRenderer

# ROS2
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import CompressedImage
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False

_LOCK_FILE = "/tmp/wuji_teleop_camera.lock"

# Logo lives one level up from the ui/ subpackage (same as monitor/brake).
_LOGO_PATH = Path(__file__).resolve().parent.parent / "wuji_icon.png"

# Camera config: key -> (topic, display label). Four feeds, 2x2.
# The head is a STEREO pair published as /stereo/{left,right}/compressed (two
# eyes); the wrist D405s publish /cam_{left,right}_wrist/color/image_raw/compressed.
# Topic names intentionally left as published (no rename); see
# docs/wuji-camera-topics.md for the full map.
CAMERAS = {
    'head_left':  ('/stereo/left/compressed',  'Head Left'),
    'head_right': ('/stereo/right/compressed', 'Head Right'),
    'left':  ('/cam_left_wrist/color/image_raw/compressed',  'Left Wrist'),
    'right': ('/cam_right_wrist/color/image_raw/compressed', 'Right Wrist'),
}

_STYLE_NORMAL = (
    "QLabel { background-color: #111; border: 1px solid #333; "
    "border-radius: 4px; color: #666; }")
_STYLE_STALE = (
    "QLabel { background-color: #111; border: 1px solid #C62828; "
    "border-radius: 4px; color: #C62828; }")


class _FrameDecoder(QThread):
    """Background JPEG decode thread — one per camera.

    QImage.loadFromData() is thread-safe (per Qt docs). The decoded
    QImage is delivered to the Qt main thread via signal for rendering.
    A single-slot buffer provides natural backpressure: only the latest
    frame is kept; older frames are dropped.
    """

    decoded = pyqtSignal(QImage)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._lock = threading.Lock()
        self._data = None
        self._event = threading.Event()
        self._running = True

    def submit(self, data: bytes):
        """Called from the ROS2 thread: store the latest frame, wake the decoder."""
        with self._lock:
            self._data = data
        self._event.set()

    def run(self):
        while self._running:
            self._event.wait()
            self._event.clear()
            if not self._running:
                break
            with self._lock:
                data = self._data
                self._data = None
            if data is None:
                continue
            img = QImage()
            if img.loadFromData(data):
                self.decoded.emit(img)

    def stop(self):
        self._running = False
        self._event.set()


class CameraLabel(QLabel):
    """Single-camera display label — background JPEG decode, no OpenCV.

    No explicit frame-rate throttle — the decoder's single-slot buffer
    handles it: when the decoder keeps up it renders at full rate
    (30fps); when it falls behind, only the latest frame is kept.
    """

    STALE_THRESHOLD = 2.0

    def __init__(self, name: str, parent=None):
        super().__init__(parent)
        self._last_frame_time = 0.0
        self._stale = True
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumSize(320, 240)
        self.setStyleSheet(_STYLE_NORMAL)
        self.setText(f"{name}\nWaiting for frames...")

        self._decoder = _FrameDecoder(parent=self)
        self._decoder.decoded.connect(self._on_decoded)
        self._decoder.start()

        self._stale_timer = QTimer(self)
        self._stale_timer.timeout.connect(self._check_stale)
        self._stale_timer.start(1000)

    def submit_frame(self, data: bytes):
        """Called from the ROS2 callback thread — hand off to the background
        decoder (single-slot overwrite handles rate-limiting)."""
        self._last_frame_time = time.monotonic()
        self._decoder.submit(data)

    def _on_decoded(self, img: QImage):
        """Qt main thread: QImage -> QPixmap render (fast memcpy, no JPEG decode)."""
        if self._stale:
            self._stale = False
            self.setStyleSheet(_STYLE_NORMAL)
        pixmap = QPixmap.fromImage(img)
        scaled = pixmap.scaled(
            self.size(), Qt.KeepAspectRatio, Qt.FastTransformation)
        del pixmap
        old = self.pixmap()
        self.setText("")
        self.setPixmap(scaled)
        if old is not None:
            del old

    def _check_stale(self):
        """1Hz QTimer: detect a stalled feed."""
        if self._last_frame_time == 0.0:
            return
        was_stale = self._stale
        self._stale = (time.monotonic() - self._last_frame_time) > self.STALE_THRESHOLD
        if self._stale and not was_stale:
            self.setStyleSheet(_STYLE_STALE)
            self.setText("Signal lost")

    def cleanup(self):
        """Stop the decoder thread and the stale timer."""
        self._stale_timer.stop()
        self._decoder.stop()
        self._decoder.wait(2000)


class CameraWindow(QMainWindow):
    """Three-camera preview window."""

    def __init__(self, ros_node=None):
        super().__init__()
        self._ros_node = ros_node
        self._labels = {}
        self._init_ui()

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
        self.setWindowTitle("Camera Preview")
        self.setMinimumSize(800, 660)
        self.setStyleSheet("QMainWindow { background-color: #1e1e1e; }")

        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(6)

        # Header: logo + title strip (matches monitor / brake style).
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
        title_lbl = QLabel("Camera Preview — stereo head + dual wrist")
        title_lbl.setFont(QFont("Arial", 13, QFont.Bold))
        title_lbl.setStyleSheet("color: #ddd;")
        title_layout.addWidget(title_lbl)
        title_layout.addStretch()
        header.addWidget(title_frame, 1)
        layout.addLayout(header)

        # 2x2 grid: stereo head (left/right eye) on top, wrists below.
        grid = QGridLayout()
        grid.setSpacing(6)
        positions = {
            'head_left':  (0, 0), 'head_right': (0, 1),
            'left':       (1, 0), 'right':      (1, 1),
        }
        for key, (row, col) in positions.items():
            label = CameraLabel(CAMERAS[key][1])
            self._labels[key] = label
            grid.addWidget(label, row, col)
        layout.addLayout(grid, 1)

    def get_label(self, key: str) -> CameraLabel:
        return self._labels.get(key)

    def closeEvent(self, event):
        for label in self._labels.values():
            label.cleanup()
        event.accept()


def _acquire_lock():
    try:
        fd = open(_LOCK_FILE, 'w')
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        fd.write(str(os.getpid()))
        fd.flush()
        return fd
    except (IOError, OSError):
        print("Error: Camera Preview UI is already running (another instance holds the lock file).")
        try:
            app = QApplication(sys.argv)
            QMessageBox.critical(
                None, "Camera Preview",
                "Camera Preview UI is already running!\n\n"
                "Only one instance is allowed at a time.\n"
                "Close the existing window first.")
        except Exception:
            pass
        sys.exit(1)


def main():
    lock_fd = _acquire_lock()

    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    ros_node = None
    if HAS_ROS2:
        rclpy.init()
        ros_node = Node('camera_preview')

    window = CameraWindow(ros_node)

    # Subscribe to camera topics (QoS auto-matches the publisher).
    if ros_node:
        from .qos_utils import match_publisher_qos

        for key, (topic, _name) in CAMERAS.items():
            label = window.get_label(key)
            qos = match_publisher_qos(ros_node, topic, depth=1)
            ros_node.create_subscription(
                CompressedImage, topic,
                lambda msg, lb=label: lb.submit_frame(msg.data),
                qos)

        spin_thread = threading.Thread(
            target=rclpy.spin, args=(ros_node,), daemon=True)
        spin_thread.start()

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

"""
Qt widgets for teleop monitor.
"""

from __future__ import annotations

from PyQt5.QtWidgets import QVBoxLayout, QHBoxLayout, QLabel, QGroupBox, QWidget
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont


class DeviceStatusWidget(QGroupBox):
    """Widget to display device connection status."""

    def __init__(self, title: str, parent=None):
        super().__init__(title, parent)
        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout()

        # Status indicator
        self.status_label = QLabel("● Disconnected")
        self.status_label.setFont(QFont("Arial", 14, QFont.Bold))
        self.status_label.setStyleSheet("color: #cc0000;")
        layout.addWidget(self.status_label)

        # ID/Serial label
        self.id_label = QLabel("ID: --")
        self.id_label.setFont(QFont("Arial", 10))
        layout.addWidget(self.id_label)

        # Extra info label
        self.info_label = QLabel("")
        self.info_label.setFont(QFont("Arial", 10))
        self.info_label.setStyleSheet("color: #666666;")
        self.info_label.setTextFormat(Qt.RichText)  # Enable rich text / HTML rendering
        layout.addWidget(self.info_label)

        self.setLayout(layout)
        self.setMinimumWidth(200)
        self.setMinimumHeight(90)

    def set_connected(self, connected: bool):
        """Set connection status display."""
        if connected:
            self.status_label.setText("● Connected")
            self.status_label.setStyleSheet("color: #00aa00;")
        else:
            self.status_label.setText("● Disconnected")
            self.status_label.setStyleSheet("color: #cc0000;")

    def set_id(self, id_text: str):
        """Set ID/Serial display."""
        self.id_label.setText(id_text)

    def set_info(self, info_text: str):
        """Set extra info display."""
        self.info_label.setText(info_text)


class TopicStatusWidget(QWidget):
    """Widget to display a single topic status."""

    def __init__(self, topic_name: str, parent=None):
        super().__init__(parent)
        self.topic_name = topic_name
        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout()
        layout.setContentsMargins(5, 5, 5, 5)

        # Topic name
        name_label = QLabel(self.topic_name)
        name_label.setFont(QFont("Arial", 9))
        name_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(name_label)

        # Status indicator
        self.status_label = QLabel("○")
        self.status_label.setFont(QFont("Arial", 16, QFont.Bold))
        self.status_label.setStyleSheet("color: #cc0000;")
        self.status_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.status_label)

        # Info (message count or frequency)
        self.info_label = QLabel("--")
        self.info_label.setFont(QFont("Arial", 9))
        self.info_label.setStyleSheet("color: #666666;")
        self.info_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.info_label)

        self.setLayout(layout)
        self.setMinimumWidth(100)

    def set_active(self, active: bool):
        """Set topic active status."""
        if active:
            self.status_label.setText("●")
            self.status_label.setStyleSheet("color: #00aa00;")
        else:
            self.status_label.setText("○")
            self.status_label.setStyleSheet("color: #cc0000;")

    def set_info(self, info_text: str):
        """Set info text."""
        self.info_label.setText(info_text)


class DataFlowWidget(QGroupBox):
    """Widget to display topic status."""

    def __init__(self, parent=None):
        super().__init__("Topic Status", parent)
        self._setup_ui()

    def _setup_ui(self):
        layout = QHBoxLayout()

        # Topic widgets
        self.glove_0 = TopicStatusWidget("/manus_glove_0")
        self.glove_1 = TopicStatusWidget("/manus_glove_1")
        self.hand_input = TopicStatusWidget("/hand_input")

        layout.addWidget(self.glove_0)
        layout.addWidget(self.glove_1)
        layout.addWidget(self.hand_input)

        self.setLayout(layout)


class CameraStatusWidget(QGroupBox):
    """Widget to display camera status."""

    def __init__(self, parent=None):
        super().__init__("Camera Status", parent)
        self._setup_ui()

    def _setup_ui(self):
        layout = QHBoxLayout()

        # Camera topic widgets
        self.cam_head = TopicStatusWidget("Head")
        self.cam_left = TopicStatusWidget("L Wrist")
        self.cam_right = TopicStatusWidget("R Wrist")

        layout.addWidget(self.cam_head)
        layout.addWidget(self.cam_left)
        layout.addWidget(self.cam_right)

        self.setLayout(layout)


class NodeStatusWidget(QGroupBox):
    """Widget to display ROS node running status."""

    def __init__(self, parent=None):
        super().__init__("Node Status", parent)
        self.node_labels = {}
        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout()

        # Will be populated dynamically
        self.content_layout = QVBoxLayout()
        layout.addLayout(self.content_layout)

        self.setLayout(layout)

    def update_nodes(self, node_status: dict):
        """Update node status display."""
        # Clear existing labels
        for i in reversed(range(self.content_layout.count())):
            widget = self.content_layout.itemAt(i).widget()
            if widget:
                widget.deleteLater()

        self.node_labels.clear()

        # Create labels for each node
        for node_name, is_running in node_status.items():
            label = QLabel()
            label.setFont(QFont("Arial", 10))

            if is_running:
                label.setText(f"● {node_name}")
                label.setStyleSheet("color: #00aa00;")
            else:
                label.setText(f"○ {node_name}")
                label.setStyleSheet("color: #cc0000;")

            self.content_layout.addWidget(label)
            self.node_labels[node_name] = label


class StereoHeadStatusWidget(QGroupBox):
    """Widget to display stereo head camera status."""

    def __init__(self, parent=None):
        super().__init__("Stereo Head Camera", parent)
        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout()

        # Hardware status row
        hw_layout = QHBoxLayout()

        # Stereo camera status
        self.camera_label = QLabel("○ Camera")
        self.camera_label.setFont(QFont("Arial", 10))
        self.camera_label.setStyleSheet("color: #cc0000;")
        hw_layout.addWidget(self.camera_label)

        # v4l2loopback status
        self.loopback_label = QLabel("○ Loopback")
        self.loopback_label.setFont(QFont("Arial", 10))
        self.loopback_label.setStyleSheet("color: #cc0000;")
        hw_layout.addWidget(self.loopback_label)

        hw_layout.addStretch()
        layout.addLayout(hw_layout)

        # Topic status row
        topic_layout = QHBoxLayout()

        # Left eye topic
        self.topic_left = TopicStatusWidget("Left Eye")
        topic_layout.addWidget(self.topic_left)

        # Right eye topic
        self.topic_right = TopicStatusWidget("Right Eye")
        topic_layout.addWidget(self.topic_right)

        layout.addLayout(topic_layout)

        self.setLayout(layout)

    def set_camera_available(self, available: bool, info: str = ""):
        """Set camera hardware status."""
        if available:
            self.camera_label.setText(f"● Camera {info}")
            self.camera_label.setStyleSheet("color: #00aa00;")
        else:
            self.camera_label.setText("○ Camera N/A")
            self.camera_label.setStyleSheet("color: #cc0000;")

    def set_loopback_available(self, available: bool, info: str = ""):
        """Set loopback device status."""
        if available:
            self.loopback_label.setText("● Loopback")
            self.loopback_label.setStyleSheet("color: #00aa00;")
        else:
            self.loopback_label.setText(f"○ {info}")
            self.loopback_label.setStyleSheet("color: #cc0000;")

    def set_topic_status(self, left_connected: bool, left_count: int,
                         right_connected: bool, right_count: int):
        """Set topic connection status."""
        self.topic_left.set_active(left_connected)
        self.topic_left.set_info(f"{left_count} frames")

        self.topic_right.set_active(right_connected)
        self.topic_right.set_info(f"{right_count} frames")

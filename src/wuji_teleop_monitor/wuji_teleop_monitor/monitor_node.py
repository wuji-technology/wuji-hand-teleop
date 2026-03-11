"""
Wuji Teleop Monitor - Entry point
usgage:
ros2 run wuji_teleop_monitor monitor

Monitor for Manus glove and Wuji hand connection status.
"""

from __future__ import annotations

import signal
import sys

import rclpy
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QTimer

from .ros_node import TeleopMonitorNode
from .main_window import MonitorWindow
from .logger import setup_logger

# Setup global logger
logger = setup_logger("teleop_monitor")


def main():
    """Main entry point."""
    logger.info("=" * 60)
    logger.info("Starting Wuji Teleop Monitor")
    logger.info("=" * 60)

    try:
        logger.debug("Initializing ROS2")
        rclpy.init()

        logger.debug("Creating ROS2 node")
        ros_node = TeleopMonitorNode()

        logger.debug("Creating PyQt5 application")
        app = QApplication(sys.argv)

        logger.debug("Creating main window")
        window = MonitorWindow(ros_node)
        window.show()
        logger.info("Main window displayed")

        # Setup signal handler so Ctrl+C can properly close the window
        def signal_handler(signum, frame):
            sig_name = "SIGINT" if signum == signal.SIGINT else "SIGTERM"
            logger.warning(f"Received signal {sig_name}, exiting...")
            window.close()  # Triggers closeEvent

        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)
        logger.debug("Signal handlers registered (SIGINT, SIGTERM)")

        # Timer to allow Python to process signals (PyQt event loop blocks signals)
        signal_timer = QTimer()
        signal_timer.timeout.connect(lambda: None)  # No-op, just to let event loop process signals
        signal_timer.start(100)
        logger.debug("Signal timer started (100ms)")

        exit_code = 0
        try:
            logger.info("Entering Qt event loop")
            exit_code = app.exec_()
            logger.info(f"Qt event loop ended, exit code: {exit_code}")
        except KeyboardInterrupt:
            logger.warning("Keyboard interrupt caught")
            window.close()
        except Exception as e:
            logger.error(f"Qt event loop error: {e}", exc_info=True)
        finally:
            logger.info("Starting resource cleanup...")

            signal_timer.stop()
            logger.debug("Signal timer stopped")

            window.stop()
            logger.debug("Window stopped")

            try:
                ros_node.destroy_node()
                logger.debug("ROS2 node destroyed")
            except Exception as e:
                logger.error(f"Failed to destroy ROS2 node: {e}", exc_info=True)

            try:
                rclpy.shutdown()
                logger.debug("ROS2 shutdown complete")
            except Exception as e:
                logger.error(f"Failed to shutdown ROS2: {e}", exc_info=True)

            logger.info("Resource cleanup complete")

        logger.info(f"Program exiting, exit code: {exit_code}")
        logger.info("=" * 60)
        sys.exit(exit_code)

    except Exception as e:
        logger.critical(f"Program startup failed: {e}", exc_info=True)
        sys.exit(1)


if __name__ == "__main__":
    main()

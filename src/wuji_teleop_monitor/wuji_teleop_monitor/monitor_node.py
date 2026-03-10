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
    logger.info("启动 Wuji Teleop Monitor")
    logger.info("=" * 60)

    try:
        logger.debug("初始化 ROS2 系统")
        rclpy.init()

        logger.debug("创建 ROS2 节点")
        ros_node = TeleopMonitorNode()

        logger.debug("创建 PyQt5 应用")
        app = QApplication(sys.argv)

        logger.debug("创建主窗口")
        window = MonitorWindow(ros_node)
        window.show()
        logger.info("主窗口已显示")

        # 设置信号处理，使 Ctrl+C 能正确关闭窗口
        def signal_handler(signum, frame):
            sig_name = "SIGINT" if signum == signal.SIGINT else "SIGTERM"
            logger.warning(f"收到中断信号 {sig_name}，正在退出...")
            window.close()  # 触发 closeEvent

        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)
        logger.debug("信号处理器已注册 (SIGINT, SIGTERM)")

        # 定时器用于让 Python 能够处理信号（PyQt 事件循环会阻塞信号）
        signal_timer = QTimer()
        signal_timer.timeout.connect(lambda: None)  # 空操作，只是为了让事件循环能处理信号
        signal_timer.start(100)
        logger.debug("信号定时器已启动 (100ms)")

        exit_code = 0
        try:
            logger.info("进入 Qt 事件循环")
            exit_code = app.exec_()
            logger.info(f"Qt 事件循环结束，退出码: {exit_code}")
        except KeyboardInterrupt:
            logger.warning("捕获到键盘中断")
            window.close()
        except Exception as e:
            logger.error(f"Qt 事件循环异常: {e}", exc_info=True)
        finally:
            logger.info("开始清理资源...")

            signal_timer.stop()
            logger.debug("信号定时器已停止")

            window.stop()
            logger.debug("窗口已停止")

            try:
                ros_node.destroy_node()
                logger.debug("ROS2 节点已销毁")
            except Exception as e:
                logger.error(f"销毁 ROS2 节点失败: {e}", exc_info=True)

            try:
                rclpy.shutdown()
                logger.debug("ROS2 系统已关闭")
            except Exception as e:
                logger.error(f"关闭 ROS2 系统失败: {e}", exc_info=True)

            logger.info("资源清理完成")

        logger.info(f"程序退出，退出码: {exit_code}")
        logger.info("=" * 60)
        sys.exit(exit_code)

    except Exception as e:
        logger.critical(f"程序启动失败: {e}", exc_info=True)
        sys.exit(1)


if __name__ == "__main__":
    main()

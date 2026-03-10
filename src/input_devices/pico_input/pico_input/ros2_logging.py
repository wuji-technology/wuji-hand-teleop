"""
ROS2 日志桥接工具

将 Python stdlib logging 桥接到 ROS2 /rosout，
使非 Node 类的日志也能被 rqt_console 等标准工具捕获。

使用方法:
    # 在 Node.__init__ 中调用一次:
    from pico_input.ros2_logging import setup_ros2_logging_bridge
    setup_ros2_logging_bridge(self.get_logger())

    # 之后所有 logging.getLogger() 的输出自动转发到 /rosout
"""
import logging


class ROS2LoggingHandler(logging.Handler):
    """将 stdlib logging 转发到 ROS2 logger"""

    def __init__(self, ros_logger):
        super().__init__()
        self._ros_logger = ros_logger

    def emit(self, record: logging.LogRecord):
        msg = self.format(record)
        level = record.levelno
        if level >= logging.ERROR:
            self._ros_logger.error(msg)
        elif level >= logging.WARNING:
            self._ros_logger.warning(msg)
        elif level >= logging.INFO:
            self._ros_logger.info(msg)
        else:
            self._ros_logger.debug(msg)


class ROS2LoggerAdapter:
    """适配器: 让 stdlib logging 接口调用 ROS2 logger (用于依赖注入)"""

    def __init__(self, ros_logger):
        self._logger = ros_logger

    def _format(self, msg, args):
        return msg % args if args else msg

    def info(self, msg, *args):
        self._logger.info(self._format(msg, args))

    def debug(self, msg, *args):
        self._logger.debug(self._format(msg, args))

    def warning(self, msg, *args):
        self._logger.warning(self._format(msg, args))

    def error(self, msg, *args):
        self._logger.error(self._format(msg, args))

    @property
    def handlers(self):
        return []


_bridge_installed = False


def setup_ros2_logging_bridge(ros_logger, level: int = logging.INFO):
    """
    安装 stdlib→ROS2 桥接 (全局，仅首次生效)

    在 Node.__init__ 中调用一次，之后所有 logging.getLogger()
    的输出会同时:
    1. 写入 stderr (默认 StreamHandler)
    2. 转发到 ROS2 /rosout (ROS2LoggingHandler)

    Args:
        ros_logger: ROS2 node 的 get_logger() 返回值
        level: 最低转发级别 (默认 INFO)
    """
    global _bridge_installed
    if _bridge_installed:
        return
    handler = ROS2LoggingHandler(ros_logger)
    handler.setLevel(level)
    handler.setFormatter(logging.Formatter('[%(name)s] %(message)s'))
    logging.getLogger().addHandler(handler)
    logging.getLogger().setLevel(min(logging.getLogger().level or logging.WARNING, level))
    _bridge_installed = True

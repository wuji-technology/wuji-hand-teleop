"""
ROS2 Logging Bridge Utility

Bridges Python stdlib logging to ROS2 /rosout,
so that logs from non-Node classes can be captured by standard tools like rqt_console.

Usage:
    # Call once in Node.__init__:
    from pico_input.ros2_logging import setup_ros2_logging_bridge
    setup_ros2_logging_bridge(self.get_logger())

    # After that, all logging.getLogger() output is automatically forwarded to /rosout
"""
import logging


class ROS2LoggingHandler(logging.Handler):
    """Forward stdlib logging to ROS2 logger"""

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
    """Adapter: makes stdlib logging interface call ROS2 logger (for dependency injection)"""

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
    Install stdlib->ROS2 bridge (global, only effective on first call)

    Call once in Node.__init__, after which all logging.getLogger()
    output will simultaneously:
    1. Write to stderr (default StreamHandler)
    2. Forward to ROS2 /rosout (ROS2LoggingHandler)

    Args:
        ros_logger: Return value of ROS2 node's get_logger()
        level: Minimum forwarding level (default INFO)
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

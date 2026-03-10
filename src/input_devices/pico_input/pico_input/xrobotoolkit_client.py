#!/usr/bin/env python3
"""
XRoboToolkit Python Client - 使用 Pybind SDK 的 PICO VR 设备数据客户端

通过 xrobotoolkit_sdk (Pybind) 连接到 XRoboToolkit PC-Service，获取 PICO 设备追踪数据。

数据流:
    PICO 头显 → XRoboToolkit APK → WiFi → PC-Service → Pybind SDK → 本客户端

使用方法:
    from pico_input.xrobotoolkit_client import XRoboToolkitClient

    client = XRoboToolkitClient()
    client.init()

    # 获取数据
    hmd_pose = client.get_headset_pose()       # [x, y, z, qx, qy, qz, qw]
    trackers = client.get_motion_tracker_pose() # [[x,y,z,qx,qy,qz,qw], ...]
    num = client.num_motion_data_available()

    client.close()

依赖:
    - xrobotoolkit_sdk (Pybind): https://github.com/lzhu686/XRoboToolkit-PC-Service-Pybind
"""

import logging
from typing import List, Optional

logger = logging.getLogger(__name__)

# 尝试导入 Pybind SDK
try:
    import xrobotoolkit_sdk as xrt
    _USE_PYBIND = True
except ImportError:
    _USE_PYBIND = False
    logger.warning("xrobotoolkit_sdk 未安装，将无法连接 PC-Service")
    logger.warning("安装方法: cd ~/Desktop/XRoboToolkit-PC-Service-Pybind && pip install .")


class XRoboToolkitClient:
    """
    XRoboToolkit Python 客户端 (Pybind 版本)

    通过 xrobotoolkit_sdk 直接连接 PC-Service。
    """

    def __init__(self, host: str = "127.0.0.1", port: int = 60061):
        """
        初始化客户端

        Args:
            host: PC-Service 地址 (Pybind 版本忽略此参数，使用本地连接)
            port: PC-Service 端口 (Pybind 版本忽略此参数)
        """
        self._connected = False
        # Pybind SDK 使用本地共享内存，不需要 host/port

    def init(self) -> bool:
        """
        初始化并连接到 PC-Service

        Returns:
            True 如果连接成功
        """
        if not _USE_PYBIND:
            logger.error("xrobotoolkit_sdk 未安装")
            return False

        try:
            xrt.init()
            self._connected = True
            logger.info("已连接到 PC-Service (Pybind SDK)")
            return True
        except Exception as e:
            logger.error("初始化失败: %s", e)
            logger.error("请确保 PC-Service 正在运行: /opt/apps/roboticsservice/runService.sh")
            return False

    def close(self):
        """关闭连接"""
        self._connected = False
        logger.info("已关闭连接")

    def is_connected(self) -> bool:
        """检查是否已连接"""
        return self._connected

    # ==================== 位姿获取 API ====================

    def get_headset_pose(self) -> Optional[List[float]]:
        """获取 HMD 位姿 [x, y, z, qx, qy, qz, qw]"""
        if not self._connected or not _USE_PYBIND:
            return None
        try:
            pose = xrt.get_headset_pose()
            return list(pose) if pose else None
        except Exception:
            return None

    def get_left_controller_pose(self) -> Optional[List[float]]:
        """获取左手柄位姿 [x, y, z, qx, qy, qz, qw]"""
        if not self._connected or not _USE_PYBIND:
            return None
        try:
            pose = xrt.get_left_controller_pose()
            return list(pose) if pose else None
        except Exception:
            return None

    def get_right_controller_pose(self) -> Optional[List[float]]:
        """获取右手柄位姿 [x, y, z, qx, qy, qz, qw]"""
        if not self._connected or not _USE_PYBIND:
            return None
        try:
            pose = xrt.get_right_controller_pose()
            return list(pose) if pose else None
        except Exception:
            return None

    def num_motion_data_available(self) -> int:
        """获取可用 Motion Tracker 数量"""
        if not self._connected or not _USE_PYBIND:
            return 0
        try:
            return xrt.num_motion_data_available()
        except Exception:
            return 0

    def get_motion_tracker_pose(self) -> Optional[List[List[float]]]:
        """获取所有 Motion Tracker 位姿 [[x,y,z,qx,qy,qz,qw], ...]"""
        if not self._connected or not _USE_PYBIND:
            return None
        try:
            poses = xrt.get_motion_tracker_pose()
            return [list(p) for p in poses] if poses else None
        except Exception:
            return None

    def get_motion_tracker_velocity(self) -> Optional[List[List[float]]]:
        """获取所有 Motion Tracker 速度 [[vx,vy,vz,wx,wy,wz], ...]"""
        if not self._connected or not _USE_PYBIND:
            return None
        try:
            velocities = xrt.get_motion_tracker_velocity()
            return [list(v) for v in velocities] if velocities else None
        except Exception:
            return None

    def get_motion_tracker_acceleration(self) -> Optional[List[List[float]]]:
        """获取所有 Motion Tracker 加速度"""
        if not self._connected or not _USE_PYBIND:
            return None
        try:
            accel = xrt.get_motion_tracker_acceleration()
            return [list(a) for a in accel] if accel else None
        except Exception:
            return None

    def get_motion_tracker_serial_numbers(self) -> List[str]:
        """获取 Motion Tracker 序列号"""
        if not self._connected or not _USE_PYBIND:
            return []
        try:
            return list(xrt.get_motion_tracker_serial_numbers())
        except Exception:
            return []

    # ==================== 手柄输入 API ====================

    def get_left_trigger(self) -> float:
        """获取左扳机值 [0, 1]"""
        if not self._connected or not _USE_PYBIND:
            return 0.0
        try:
            return xrt.get_left_trigger()
        except Exception:
            return 0.0

    def get_right_trigger(self) -> float:
        """获取右扳机值 [0, 1]"""
        if not self._connected or not _USE_PYBIND:
            return 0.0
        try:
            return xrt.get_right_trigger()
        except Exception:
            return 0.0

    def get_left_grip(self) -> float:
        """获取左握把值 [0, 1]"""
        if not self._connected or not _USE_PYBIND:
            return 0.0
        try:
            return xrt.get_left_grip()
        except Exception:
            return 0.0

    def get_right_grip(self) -> float:
        """获取右握把值 [0, 1]"""
        if not self._connected or not _USE_PYBIND:
            return 0.0
        try:
            return xrt.get_right_grip()
        except Exception:
            return 0.0

    def get_left_axis(self) -> List[float]:
        """获取左摇杆 [x, y]"""
        if not self._connected or not _USE_PYBIND:
            return [0.0, 0.0]
        try:
            return list(xrt.get_left_axis())
        except Exception:
            return [0.0, 0.0]

    def get_right_axis(self) -> List[float]:
        """获取右摇杆 [x, y]"""
        if not self._connected or not _USE_PYBIND:
            return [0.0, 0.0]
        try:
            return list(xrt.get_right_axis())
        except Exception:
            return [0.0, 0.0]

    def get_time_stamp_ns(self) -> int:
        """获取时间戳 (纳秒)"""
        if not self._connected or not _USE_PYBIND:
            return 0
        try:
            return xrt.get_motion_timestamp_ns()
        except Exception:
            return 0


# ==================== 模块级 API (兼容 xrobotoolkit_sdk) ====================

_client: Optional[XRoboToolkitClient] = None


def init(host: str = "127.0.0.1", port: int = 60061) -> bool:
    """初始化 SDK"""
    global _client
    if _client is not None:
        return True
    _client = XRoboToolkitClient(host, port)
    return _client.init()


def close():
    """关闭 SDK"""
    global _client
    if _client is not None:
        _client.close()
        _client = None


def get_headset_pose() -> Optional[List[float]]:
    """获取 HMD 位姿"""
    return _client.get_headset_pose() if _client else None


def get_left_controller_pose() -> Optional[List[float]]:
    """获取左手柄位姿"""
    return _client.get_left_controller_pose() if _client else None


def get_right_controller_pose() -> Optional[List[float]]:
    """获取右手柄位姿"""
    return _client.get_right_controller_pose() if _client else None


def num_motion_data_available() -> int:
    """获取可用 Motion Tracker 数量"""
    return _client.num_motion_data_available() if _client else 0


def get_motion_tracker_pose() -> Optional[List[List[float]]]:
    """获取所有 Motion Tracker 位姿"""
    return _client.get_motion_tracker_pose() if _client else None


def get_motion_tracker_velocity() -> Optional[List[List[float]]]:
    """获取所有 Motion Tracker 速度"""
    return _client.get_motion_tracker_velocity() if _client else None


def get_motion_tracker_serial_numbers() -> List[str]:
    """获取 Motion Tracker 序列号"""
    return _client.get_motion_tracker_serial_numbers() if _client else []


def get_left_trigger() -> float:
    """获取左扳机值"""
    return _client.get_left_trigger() if _client else 0.0


def get_right_trigger() -> float:
    """获取右扳机值"""
    return _client.get_right_trigger() if _client else 0.0


def get_left_grip() -> float:
    """获取左握把值"""
    return _client.get_left_grip() if _client else 0.0


def get_right_grip() -> float:
    """获取右握把值"""
    return _client.get_right_grip() if _client else 0.0


def get_left_axis() -> List[float]:
    """获取左摇杆"""
    return _client.get_left_axis() if _client else [0.0, 0.0]


def get_right_axis() -> List[float]:
    """获取右摇杆"""
    return _client.get_right_axis() if _client else [0.0, 0.0]


def get_time_stamp_ns() -> int:
    """获取时间戳"""
    return _client.get_time_stamp_ns() if _client else 0

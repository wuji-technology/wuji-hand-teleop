#!/usr/bin/env python3
"""
Wuji 灵巧手硬件接口封装 / Wuji Hand Hardware Interface

通过 wujihandros2 驱动 (C++ wujihandcpp SDK) 进行 ROS2 通信
"""
import threading
from typing import Optional
import numpy as np

# ROS2 依赖
try:
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import JointState
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    Node = None


def get_sensor_data_qos() -> 'QoSProfile':
    """获取传感器数据 QoS 配置 (与 wujihandros2 驱动一致)"""
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=10
    )


class WujiHandROS2:
    """
    Wuji 灵巧手 ROS2 接口 (通过 wujihandros2 驱动)

    通过 ROS2 Topics 与 wujihandros2 C++ 驱动通信：
    - 发布: /{hand_name}/joint_commands
    - 订阅: /{hand_name}/joint_states

    wujihandros2 使用 C++ wujihandcpp SDK，支持 1000Hz 控制频率
    """

    NUM_JOINTS = 20  # 5 手指 × 4 关节

    def __init__(
        self,
        hand_name: str,
        side: str,
        node: 'Node',
        logger=None
    ):
        """
        初始化 ROS2 接口

        Args:
            hand_name: wujihandros2 驱动的命名空间 (如 "left_hand", "right_hand")
            side: 手的位置 "left" 或 "right" (用于日志)
            node: ROS2 节点实例
            logger: 外部 logger (可选)
        """
        if not ROS2_AVAILABLE:
            raise ImportError("ROS2 依赖未安装，无法使用 WujiHandROS2")

        self.hand_name = hand_name
        self.side = side
        self.node = node
        self.logger = logger or node.get_logger()

        # 状态缓存
        self._latest_positions: Optional[np.ndarray] = None
        self._connected = False
        self._lock = threading.Lock()

        # QoS 配置
        qos = get_sensor_data_qos()

        # 创建发布者: 发送关节命令到 wujihandros2 驱动
        self._cmd_pub = node.create_publisher(
            JointState,
            f"/{hand_name}/joint_commands",
            qos
        )

        # 创建订阅者: 接收 wujihandros2 驱动的关节状态
        self._state_sub = node.create_subscription(
            JointState,
            f"/{hand_name}/joint_states",
            self._state_callback,
            qos
        )

        self.logger.info(
            f"WujiHandROS2 已初始化: {side} 手 -> /{hand_name}"
        )

    def _state_callback(self, msg: 'JointState') -> None:
        """处理来自 wujihandros2 驱动的状态消息"""
        with self._lock:
            if msg.position and len(msg.position) == self.NUM_JOINTS:
                self._latest_positions = np.array(msg.position, dtype=np.float32)
                if not self._connected:
                    self._connected = True
                    self.logger.info(f"{self.side.title()} 手已连接 (通过 wujihandros2)")

    def connect(self) -> bool:
        """
        连接检查 (ROS2 模式下始终返回 True，实际连接由驱动节点管理)

        Returns:
            bool: True
        """
        self.logger.info(f"{self.side.title()} 手等待 wujihandros2 驱动连接...")
        return True

    def is_connected(self) -> bool:
        """
        检查是否已收到驱动状态 (表示驱动已连接硬件)

        Returns:
            bool: 是否收到过状态消息
        """
        with self._lock:
            return self._connected

    def set_joint_positions(self, positions: np.ndarray) -> bool:
        """
        设置关节角度 (发布到 wujihandros2 驱动)

        Args:
            positions: 关节角度数组，形状为 (20,) 或 (5, 4)

        Returns:
            bool: 发布是否成功
        """
        try:
            positions = np.asarray(positions, dtype=np.float32)
            if positions.shape == (5, 4):
                positions = positions.flatten()
            elif positions.shape != (self.NUM_JOINTS,):
                self.logger.error(
                    f"无效的关节角度形状: {positions.shape}，期望 (20,) 或 (5, 4)"
                )
                return False

            # 创建 JointState 消息 (仅使用 position，不设置 name)
            # wujihandros2 驱动支持 position-only 模式，按索引顺序解析
            msg = JointState()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.position = positions.tolist()

            self._cmd_pub.publish(msg)
            return True

        except Exception as e:
            self.logger.error(f"发布关节命令失败: {e}")
            return False

    def get_joint_positions(self) -> Optional[np.ndarray]:
        """
        获取当前关节角度 (从订阅缓存读取)

        Returns:
            np.ndarray: 关节角度数组 (20,)，未连接时返回 None
        """
        with self._lock:
            if self._latest_positions is not None:
                return self._latest_positions.copy()
            return None

    def disable(self) -> None:
        """
        下使能灵巧手

        注意: wujihandros2 驱动会在节点关闭时自动下使能
        """
        self.logger.info(f"{self.side.title()} 手将由 wujihandros2 驱动管理使能状态")

    def release(self) -> None:
        """释放资源"""
        self.disable()
        with self._lock:
            self._connected = False
            self._latest_positions = None
        self.logger.info(f"{self.side.title()} 手 ROS2 接口已释放")

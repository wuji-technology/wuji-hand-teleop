#!/usr/bin/env python3
"""
Wuji 灵巧手统一控制器 / Wuji Hand Unified Controller

通过 wujihandros2 驱动 (C++ wujihandcpp SDK) 进行 ROS2 通信
支持关节角度控制和 IK 重定向控制
"""
import logging
from pathlib import Path
from typing import Optional, List, Tuple
import numpy as np

try:
    from wujihand_output._internal.hand_interface import WujiHandROS2
except ImportError:
    from ._internal.hand_interface import WujiHandROS2

# IK 重定向器（可选）
try:
    from wuji_retargeting import Retargeter
    RETARGETER_AVAILABLE = True
except ImportError:
    RETARGETER_AVAILABLE = False


class WujiHandController:
    """Wuji 灵巧手统一控制器 / Wuji Hand Unified Controller

    通过 wujihandros2 驱动进行硬件通信 (1000Hz 控制频率)

    支持两种控制模式：
    1. 关节角度控制：直接设置 20 个关节角度
    2. IK 控制：使用手部关键点（21 个）进行逆运动学重定向
    """

    NUM_JOINTS = 20  # 5 手指 × 4 关节
    JOINT_NAMES = [
        "thumb_joint_0", "thumb_joint_1", "thumb_joint_2", "thumb_joint_3",
        "index_joint_0", "index_joint_1", "index_joint_2", "index_joint_3",
        "middle_joint_0", "middle_joint_1", "middle_joint_2", "middle_joint_3",
        "ring_joint_0", "ring_joint_1", "ring_joint_2", "ring_joint_3",
        "pinky_joint_0", "pinky_joint_1", "pinky_joint_2", "pinky_joint_3",
    ]

    def __init__(
        self,
        input_source: str = "manus",
        left_retarget_config: Optional[str] = None,
        right_retarget_config: Optional[str] = None,
        enable_ik: bool = True,
        logger=None,
        # ROS2 参数 (wujihandros2)
        node=None,
        left_hand_name: Optional[str] = None,
        right_hand_name: Optional[str] = None,
    ):
        """
        初始化 Wuji 灵巧手控制器

        Args:
            input_source: 输入源类型 "manus", 用于选择 IK 重定向配置
            left_retarget_config: 左手重定向配置文件路径（可选，用于 IK 控制）
            right_retarget_config: 右手重定向配置文件路径（可选，用于 IK 控制）
            enable_ik: 是否启用 IK 控制功能（默认 True）
            logger: 外部传入的 logger（可选）
            node: ROS2 节点实例（必需）
            left_hand_name: 左手 wujihandros2 驱动命名空间（如 "left_hand"）
            right_hand_name: 右手 wujihandros2 驱动命名空间（如 "right_hand"）
        """
        # 日志
        if logger is not None:
            self.logger = logger
        else:
            self.logger = logging.getLogger('WujiHandController')
            self.logger.setLevel(logging.INFO)
            if not self.logger.handlers:
                handler = logging.StreamHandler()
                handler.setFormatter(logging.Formatter('[%(name)s] %(message)s'))
                self.logger.addHandler(handler)

        self.input_source = input_source

        # ROS2 参数
        self.node = node
        self.left_hand_name = left_hand_name
        self.right_hand_name = right_hand_name

        # 硬件接口 (WujiHandROS2)
        self.left_hand = None
        self.right_hand = None

        # IK 重定向器
        self.left_retargeter: Optional['Retargeter'] = None
        self.right_retargeter: Optional['Retargeter'] = None
        self._ik_enabled = False

        # 初始化 IK 重定向器（如果启用且可用）
        if enable_ik and RETARGETER_AVAILABLE:
            self._init_retargeters(left_retarget_config, right_retarget_config)
        elif enable_ik and not RETARGETER_AVAILABLE:
            self.logger.warning("wuji_retargeting 未安装，IK 控制不可用")

        # 初始化硬件 (通过 wujihandros2)
        self._init_hands()

        self.logger.info("Wuji 灵巧手控制器初始化完成 (wujihandros2 模式)")

    def _resolve_retarget_config(self, side: str) -> Optional[str]:
        """Resolve retarget config path for a given input source and side.

        Lookup order:
          1. retarget_{input_source}_{side}.yaml  (per-side config)
          2. retarget_{input_source}.yaml          (shared config)
        """
        try:
            from ament_index_python.packages import get_package_share_directory
            config_dir = Path(get_package_share_directory("wujihand_output")) / "config"
        except Exception:
            return None

        per_side = config_dir / f"retarget_{self.input_source}_{side}.yaml"
        if per_side.exists():
            return str(per_side)

        shared = config_dir / f"retarget_{self.input_source}.yaml"
        if shared.exists():
            return str(shared)

        return None

    def _init_retargeters(
        self,
        left_config: Optional[str],
        right_config: Optional[str]
    ) -> None:
        """初始化 IK 重定向器"""
        default_left_config = self._resolve_retarget_config("left")
        default_right_config = self._resolve_retarget_config("right")

        # 使用用户提供的配置或默认配置
        right_config_path = right_config or default_right_config
        left_config_path = left_config or default_left_config

        # 初始化右手重定向器
        if right_config_path and Path(right_config_path).exists():
            self.right_retargeter = Retargeter.from_yaml(right_config_path, "right")
            self.logger.info(f"右手 IK 重定向配置: {Path(right_config_path).name}")
        else:
            self.logger.warning("未找到右手 IK 重定向配置文件")

        # 初始化左手重定向器
        if left_config_path and Path(left_config_path).exists():
            self.left_retargeter = Retargeter.from_yaml(left_config_path, "left")
            self.logger.info(f"左手 IK 重定向配置: {Path(left_config_path).name}")
        else:
            self.logger.warning("未找到左手 IK 重定向配置文件")

        # 标记 IK 是否可用
        self._ik_enabled = self.left_retargeter is not None or self.right_retargeter is not None

    def _init_hands(self) -> None:
        """初始化灵巧手硬件 (通过 wujihandros2 驱动)"""
        if self.node is None:
            raise RuntimeError("ROS2 节点未提供，无法初始化 wujihandros2 接口")

        self.logger.info("通过 wujihandros2 驱动连接灵巧手 (1000Hz)")

        if self.left_hand_name:
            self.left_hand = WujiHandROS2(
                hand_name=self.left_hand_name,
                side="left",
                node=self.node,
                logger=self.logger
            )
            self.left_hand.connect()
            self.logger.info(f"左手 ROS2 接口已创建 -> /{self.left_hand_name}")

        if self.right_hand_name:
            self.right_hand = WujiHandROS2(
                hand_name=self.right_hand_name,
                side="right",
                node=self.node,
                logger=self.logger
            )
            self.right_hand.connect()
            self.logger.info(f"右手 ROS2 接口已创建 -> /{self.right_hand_name}")

    # ==================== 关节角度控制 ====================

    def set_joint_positions(
        self,
        left_positions: Optional[np.ndarray] = None,
        right_positions: Optional[np.ndarray] = None
    ) -> Tuple[bool, bool]:
        """
        设置双手关节角度（非阻塞，用于实时控制）

        Args:
            left_positions: 左手关节角度，形状 (20,) 或 (5, 4)，None 表示不控制
            right_positions: 右手关节角度，形状 (20,) 或 (5, 4)，None 表示不控制

        Returns:
            Tuple[bool, bool]: (左手成功, 右手成功)
        """
        left_success = False
        right_success = False

        if left_positions is not None and self.left_hand is not None:
            left_success = self.left_hand.set_joint_positions(left_positions)

        if right_positions is not None and self.right_hand is not None:
            right_success = self.right_hand.set_joint_positions(right_positions)

        return left_success, right_success

    def set_joint_positions_from_flat(
        self,
        flat_positions: np.ndarray
    ) -> Tuple[bool, bool]:
        """
        从扁平数组设置双手关节角度

        Args:
            flat_positions: 扁平化关节角度数组
                - 20 个值: 单手（根据启用的手分配）
                - 40 个值: 双手（右手在前，左手在后）

        Returns:
            Tuple[bool, bool]: (左手成功, 右手成功)
        """
        flat_positions = np.asarray(flat_positions, dtype=np.float32)

        if flat_positions.size == 40:
            # 双手: 右手(0:20) + 左手(20:40)
            right_pos = flat_positions[:20]
            left_pos = flat_positions[20:]
            return self.set_joint_positions(left_pos, right_pos)
        elif flat_positions.size == 20:
            # 单手: 根据启用的手分配
            if self.left_hand is not None and self.right_hand is None:
                return self.set_joint_positions(flat_positions, None)
            else:
                return self.set_joint_positions(None, flat_positions)
        else:
            self.logger.error(f"无效的关节角度数量: {flat_positions.size}，期望 20 或 40")
            return False, False

    def get_joint_positions(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        获取当前双手关节角度

        Returns:
            Tuple: (左手关节角度, 右手关节角度)，各为 (20,) 数组或 None
        """
        left_pos = None
        right_pos = None

        if self.left_hand is not None:
            left_pos = self.left_hand.get_joint_positions()

        if self.right_hand is not None:
            right_pos = self.right_hand.get_joint_positions()

        return left_pos, right_pos

    # ==================== IK 控制 ====================

    def is_ik_available(self) -> bool:
        """检查 IK 控制是否可用"""
        return self._ik_enabled

    def retarget(
        self,
        left_keypoints: Optional[np.ndarray] = None,
        right_keypoints: Optional[np.ndarray] = None
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        对手部关键点进行重定向，计算关节角度

        Args:
            left_keypoints: 左手关键点，形状 (21, 3)，None 表示不处理
            right_keypoints: 右手关键点，形状 (21, 3)，None 表示不处理

        Returns:
            Tuple: (左手关节角度, 右手关节角度)，各为 (20,) 数组或 None
        """
        left_angles = None
        right_angles = None

        if left_keypoints is not None and self.left_retargeter is not None:
            try:
                left_keypoints = np.asarray(left_keypoints, dtype=np.float32)
                if left_keypoints.shape == (63,):
                    left_keypoints = left_keypoints.reshape(21, 3)
                left_angles = self.left_retargeter.retarget(left_keypoints)
            except Exception as e:
                self.logger.error(f"左手 IK 重定向异常: {e}")

        if right_keypoints is not None and self.right_retargeter is not None:
            try:
                right_keypoints = np.asarray(right_keypoints, dtype=np.float32)
                if right_keypoints.shape == (63,):
                    right_keypoints = right_keypoints.reshape(21, 3)
                right_angles = self.right_retargeter.retarget(right_keypoints)
            except Exception as e:
                self.logger.error(f"右手 IK 重定向异常: {e}")

        return left_angles, right_angles

    def set_keypoints(
        self,
        left_keypoints: Optional[np.ndarray] = None,
        right_keypoints: Optional[np.ndarray] = None
    ) -> Tuple[bool, bool, Optional[np.ndarray], Optional[np.ndarray]]:
        """
        使用手部关键点控制灵巧手（IK 重定向 + 硬件控制）

        Args:
            left_keypoints: 左手关键点，形状 (21, 3) 或 (63,)，None 表示不控制
            right_keypoints: 右手关键点，形状 (21, 3) 或 (63,)，None 表示不控制

        Returns:
            Tuple: (左手成功, 右手成功, 左手关节角度, 右手关节角度)
        """
        # 重定向计算关节角度
        left_angles, right_angles = self.retarget(left_keypoints, right_keypoints)

        left_success = False
        right_success = False

        # 控制左手
        if left_angles is not None and self.left_hand is not None:
            left_success = self.left_hand.set_joint_positions(left_angles)

        # 控制右手
        if right_angles is not None and self.right_hand is not None:
            right_success = self.right_hand.set_joint_positions(right_angles)

        return left_success, right_success, left_angles, right_angles

    def set_keypoints_from_flat(
        self,
        flat_keypoints: np.ndarray
    ) -> Tuple[bool, bool, Optional[np.ndarray], Optional[np.ndarray]]:
        """
        从扁平数组设置手部关键点

        Args:
            flat_keypoints: 扁平化关键点数组
                - 63 个值: 单手 (21 * 3)，根据启用的手分配
                - 126 个值: 双手（右手在前，左手在后）

        Returns:
            Tuple: (左手成功, 右手成功, 左手关节角度, 右手关节角度)
        """
        flat_keypoints = np.asarray(flat_keypoints, dtype=np.float32)
        single_len = 21 * 3  # 63
        double_len = single_len * 2  # 126

        if flat_keypoints.size == double_len:
            # 双手: 右手(0:63) + 左手(63:126)
            right_kp = flat_keypoints[:single_len].reshape(21, 3)
            left_kp = flat_keypoints[single_len:].reshape(21, 3)
            return self.set_keypoints(left_kp, right_kp)
        elif flat_keypoints.size == single_len:
            # 单手: 根据启用的手分配
            kp = flat_keypoints.reshape(21, 3)
            if self.left_hand is not None and self.right_hand is None:
                return self.set_keypoints(kp, None)
            else:
                return self.set_keypoints(None, kp)
        else:
            self.logger.error(f"无效的关键点数量: {flat_keypoints.size}，期望 63 或 126")
            return False, False, None, None

    # ==================== 状态检查与释放 ====================

    def is_left_connected(self) -> bool:
        """检查左手是否已连接"""
        return self.left_hand is not None and self.left_hand.is_connected()

    def is_right_connected(self) -> bool:
        """检查右手是否已连接"""
        return self.right_hand is not None and self.right_hand.is_connected()

    def get_enabled_hands(self) -> List[str]:
        """获取已启用的手列表"""
        hands = []
        if self.is_left_connected():
            hands.append("left")
        if self.is_right_connected():
            hands.append("right")
        return hands

    def disable_and_release(self) -> None:
        """下使能并释放双手"""
        self.logger.info("灵巧手下使能...")

        if self.left_hand is not None:
            self.left_hand.release()
            self.left_hand = None

        if self.right_hand is not None:
            self.right_hand.release()
            self.right_hand = None

        self.logger.info("已安全退出")

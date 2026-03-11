"""灵巧手控制器节点

通过 wujihandros2 驱动 (C++ wujihandcpp SDK) 控制 Wuji 灵巧手

两种控制模式：
1. TELEOP 模式：hand_input → IK 逆重定向 → 灵巧手
2. INFERENCE 模式：joint_command → 灵巧手

控制架构 (定时器驱动, 固定频率):
  订阅回调仅缓存数据 → 100Hz 定时器 consume-once → retarget → 硬件
  保证输出时序均匀, 避免 CPU 满载时序抖动

  注: driver (C++) 以 1000Hz 将最新指令转发到固件,
  因此 controller 端重复发送不增加硬件实际更新率。
  提高平滑度的正确路径是提升 retarget 频率 (需 C++ 重写)。

模式切换服务：/wuji_hand/switch_mode
"""
from __future__ import annotations

import argparse
import sys
from typing import Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, Trigger
from std_msgs.msg import Float32MultiArray, Header

from wujihand_output import WujiHandController
from .common import (
    ControlMode,
    ROS2LoggerAdapter,
    get_default_qos,
    load_yaml_config,
    get_package_config_path,
)

# 话题名称
LEFT_HAND_CMD_TOPIC = "/wuji_hand/left/joint_command"
RIGHT_HAND_CMD_TOPIC = "/wuji_hand/right/joint_command"
HAND_INPUT_TOPIC = "/hand_input"

# 控制频率 (Hz)
# 100Hz: retarget ~6.6ms/帧 → CPU ~66%, 留余量给 GC/调度
# lp_alpha=0.3 @ 100Hz → 截止 5.7Hz, 覆盖人手动态
# driver (C++ 1000Hz) 自动以 1000Hz 重复转发最新指令到固件
CONTROL_RATE_HZ = 100.0

# 关节名称
JOINT_NAMES = [
    "thumb_joint_0", "thumb_joint_1", "thumb_joint_2", "thumb_joint_3",
    "index_joint_0", "index_joint_1", "index_joint_2", "index_joint_3",
    "middle_joint_0", "middle_joint_1", "middle_joint_2", "middle_joint_3",
    "ring_joint_0", "ring_joint_1", "ring_joint_2", "ring_joint_3",
    "pinky_joint_0", "pinky_joint_1", "pinky_joint_2", "pinky_joint_3",
]


class WujiHandControllerNode(Node):
    """灵巧手控制器节点 (通过 wujihandros2 驱动)

    控制架构 (定时器驱动):
      订阅回调仅缓存最新数据 (零计算) → 100Hz 定时器 consume-once → retarget → 硬件
      - 固定 10ms 间隔, 输出时序均匀
      - CPU ~66%, 留余量给 Python GIL/GC
      - 输入 120Hz 中的重复帧自动跳过
      - driver (C++ 1000Hz) 自动重复转发, 无需 controller 高频发送

    关节状态由 wujihandros2 driver 直接发布 (1000Hz):
      /{hand_name}/joint_states — 数据录制/监控请订阅此话题
    """

    def __init__(
        self,
        input_source: str = "manus",
        left_hand_name: Optional[str] = None,
        right_hand_name: Optional[str] = None,
    ):
        super().__init__("wujihand_controller")

        self._mode = ControlMode.TELEOP
        self._input_source = input_source
        self._left_hand_name = left_hand_name
        self._right_hand_name = right_hand_name
        self._logger_adapter = ROS2LoggerAdapter(self.get_logger())

        # 初始化控制器 (通过 wujihandros2)
        self.get_logger().info("正在初始化灵巧手控制器 (wujihandros2)...")
        self.controller = WujiHandController(
            input_source=input_source,
            logger=self._logger_adapter,
            node=self,
            left_hand_name=left_hand_name,
            right_hand_name=right_hand_name,
        )
        self.get_logger().info("控制器初始化完成")

        # TELEOP: 缓存最新手部输入 (回调写入, 定时器消费)
        self._latest_raw: Optional[np.ndarray] = None

        # INFERENCE: 关节指令缓存
        self._left_inference_angles: Optional[np.ndarray] = None
        self._right_inference_angles: Optional[np.ndarray] = None

        qos = get_default_qos()

        # 发布者 (指令话题, 状态由 wujihandros2 driver 1000Hz 发布)
        self.left_cmd_pub = self.create_publisher(JointState, LEFT_HAND_CMD_TOPIC, qos)
        self.right_cmd_pub = self.create_publisher(JointState, RIGHT_HAND_CMD_TOPIC, qos)

        # 订阅者 (仅缓存数据, 不做 retarget)
        self.hand_input_sub = self.create_subscription(
            Float32MultiArray, HAND_INPUT_TOPIC, self._hand_input_callback, qos)
        self.left_cmd_sub = self.create_subscription(
            JointState, LEFT_HAND_CMD_TOPIC, self._left_cmd_callback, qos)
        self.right_cmd_sub = self.create_subscription(
            JointState, RIGHT_HAND_CMD_TOPIC, self._right_cmd_callback, qos)

        # 服务
        self.create_service(SetBool, '/wuji_hand/switch_mode', self._switch_mode_callback)
        self.create_service(Trigger, '/wuji_hand/get_mode', self._get_mode_callback)

        # 定时器 (固定频率, 保证输出时序均匀)
        control_period = 1.0 / CONTROL_RATE_HZ
        self.create_timer(control_period, self._teleop_loop)
        self.create_timer(control_period, self._inference_loop)

        self.get_logger().info(
            f"初始化完成，模式: {self._mode.value.upper()}，"
            f"控制频率: {CONTROL_RATE_HZ}Hz"
        )

    @property
    def mode(self) -> ControlMode:
        return self._mode

    # -------------------- 服务回调 --------------------

    def _switch_mode_callback(self, request: SetBool.Request, response: SetBool.Response):
        new_mode = ControlMode.INFERENCE if request.data else ControlMode.TELEOP
        if self._mode != new_mode:
            self._mode = new_mode
            self._latest_raw = None
            self.get_logger().info(f"切换到 {new_mode.value} 模式")
        response.success = True
        response.message = f"当前模式: {new_mode.value}"
        return response

    def _get_mode_callback(self, request: Trigger.Request, response: Trigger.Response):
        response.success = True
        response.message = self._mode.value
        return response

    # -------------------- 订阅回调 (仅缓存, 零计算) --------------------

    def _left_cmd_callback(self, msg: JointState):
        if self._mode == ControlMode.INFERENCE and msg.position:
            self._left_inference_angles = np.array(msg.position, dtype=np.float32)

    def _right_cmd_callback(self, msg: JointState):
        if self._mode == ControlMode.INFERENCE and msg.position:
            self._right_inference_angles = np.array(msg.position, dtype=np.float32)

    def _hand_input_callback(self, msg: Float32MultiArray):
        """缓存最新手部输入 (零计算), 由 _teleop_loop 定时器消费"""
        if self._mode != ControlMode.TELEOP:
            return
        raw = np.array(msg.data, dtype=np.float32)
        if raw.size > 0:
            self._latest_raw = raw

    # -------------------- 控制循环 (100Hz 定时器驱动) --------------------

    def _teleop_loop(self):
        """TELEOP 控制循环: consume-once → retarget → 硬件

        固定 100Hz (10ms) 间隔, 保证:
        - 输出时序均匀 (无 CPU 满载抖动)
        - 每帧数据只 retarget 一次 (无重复计算)
        - CPU ~66% (留余量给 Python GIL/GC)
        - driver 以 1000Hz 自动重复转发最新指令到固件
        """
        if self._mode != ControlMode.TELEOP:
            return

        # Consume-once: 取出并清空缓存
        raw = self._latest_raw
        if raw is None:
            return
        self._latest_raw = None

        try:
            right_kp, left_kp = self._split_keypoints(raw)
        except ValueError as e:
            self.get_logger().error(f"无效数据: {e}")
            return

        _, _, left_angles, right_angles = self.controller.set_keypoints(
            left_keypoints=left_kp, right_keypoints=right_kp)
        self._publish_command(left_angles, right_angles)

    def _inference_loop(self):
        """INFERENCE 控制循环: 直接转发关节指令"""
        if self._mode != ControlMode.INFERENCE:
            return
        left = self._left_inference_angles
        right = self._right_inference_angles
        if left is not None and self.controller.left_hand:
            self.controller.left_hand.set_joint_positions(left)
        if right is not None and self.controller.right_hand:
            self.controller.right_hand.set_joint_positions(right)

    # -------------------- 发布 --------------------

    def _publish_command(self, left: Optional[np.ndarray], right: Optional[np.ndarray]):
        stamp = self.get_clock().now().to_msg()
        if left is not None:
            msg = JointState()
            msg.header = Header(stamp=stamp, frame_id="left_hand")
            msg.name = JOINT_NAMES
            msg.position = left.tolist()
            self.left_cmd_pub.publish(msg)
        if right is not None:
            msg = JointState()
            msg.header = Header(stamp=stamp, frame_id="right_hand")
            msg.name = JOINT_NAMES
            msg.position = right.tolist()
            self.right_cmd_pub.publish(msg)

    # -------------------- 工具方法 --------------------

    def _split_keypoints(self, raw: np.ndarray) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """分割关键点数据为左右手"""
        single, double = 63, 126
        if raw.size == double:
            return raw[:single].reshape(21, 3), raw[single:].reshape(21, 3)
        if raw.size == single:
            kp = raw.reshape(21, 3)
            if self.controller.is_left_connected() and not self.controller.is_right_connected():
                return None, kp
            return kp, None
        raise ValueError(f"期望 {single} 或 {double}，得到 {raw.size}")

    def shutdown(self):
        self.get_logger().info("正在关闭...")
        self.controller.disable_and_release()
        self.get_logger().info("已安全退出")


# -------------------- 入口函数 --------------------

def _parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="灵巧手控制器")
    parser.add_argument("-c", "--config", help="配置文件路径")
    parser.add_argument("-i", "--input-source", choices=["manus"], help="输入源")
    parser.add_argument("--left-hand", help="左手 wujihandros2 驱动命名空间")
    parser.add_argument("--right-hand", help="右手 wujihandros2 驱动命名空间")
    return parser.parse_args(argv)


def main(argv: Optional[list[str]] = None):
    program_name = sys.argv[0] if sys.argv else "wujihand_controller"
    raw_argv = sys.argv if argv is None else [program_name, *argv]
    cli_argv = remove_ros_args(raw_argv)[1:]
    args = _parse_args(cli_argv)

    # 加载配置 (CLI 参数优先于配置文件)
    config_path = args.config or get_package_config_path("wujihand_output", "wujihand_ik.yaml")
    config = load_yaml_config(config_path)

    rclpy.init(args=raw_argv)
    left_hand_cfg = config.get('left_hand', {})
    right_hand_cfg = config.get('right_hand', {})
    node = WujiHandControllerNode(
        input_source=args.input_source or config.get('input_source', 'manus'),
        left_hand_name=args.left_hand or left_hand_cfg.get('name'),
        right_hand_name=args.right_hand or right_hand_cfg.get('name'),
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

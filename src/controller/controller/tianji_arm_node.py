"""天机臂控制器节点

两种控制模式：
1. TELEOP 模式：TF → IK → 机器人
2. INFERENCE 模式：joint_command → 机器人

模式切换服务：/tianji_arm/switch_mode
"""
from __future__ import annotations

import argparse
import sys
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from tf2_ros import Buffer, TransformListener
import tf2_ros
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool, Trigger

from tianji_output import TianjiArmController
from .common import (
    ControlMode,
    ROS2LoggerAdapter,
    get_default_qos,
    load_yaml_config,
    get_package_config_path,
)

# 话题名称
LEFT_ARM_CMD_TOPIC = "/tianji_arm/left/joint_command"
RIGHT_ARM_CMD_TOPIC = "/tianji_arm/right/joint_command"
LEFT_ARM_STATE_TOPIC = "/tianji_arm/left/joint_state"
RIGHT_ARM_STATE_TOPIC = "/tianji_arm/right/joint_state"


class TianjiArmControllerNode(Node):
    """天机臂控制器节点"""

    def __init__(self, robot_ip: str = '192.168.1.190'):
        super().__init__("tianji_arm_controller")

        self._mode = ControlMode.TELEOP
        self._logger_adapter = ROS2LoggerAdapter(self.get_logger())
        self._log_counter = 0

        # 初始化控制器
        self.get_logger().info(f"正在连接机器人 {robot_ip}...")
        self.controller = TianjiArmController(robot_ip=robot_ip, logger=self._logger_adapter)
        self.controller.set_impedance_mode(mode='joint')

        # 移动到初始位置
        self.get_logger().info("机械臂移动到初始位置...")
        self.controller.move_to_init(wait=True, timeout=1)
        self.get_logger().info("控制器初始化完成")

        # TF 监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 位姿和关节缓存
        self.left_pose = None
        self.right_pose = None
        self.left_y_axis = None
        self.right_y_axis = None
        self.left_inference_joints = None
        self.right_inference_joints = None

        qos = get_default_qos()

        # 发布者
        self.left_cmd_pub = self.create_publisher(JointState, LEFT_ARM_CMD_TOPIC, qos)
        self.right_cmd_pub = self.create_publisher(JointState, RIGHT_ARM_CMD_TOPIC, qos)
        self.left_state_pub = self.create_publisher(JointState, LEFT_ARM_STATE_TOPIC, qos)
        self.right_state_pub = self.create_publisher(JointState, RIGHT_ARM_STATE_TOPIC, qos)

        # zsp_para 和 pose 发布者
        self.left_zsp_para_pub = self.create_publisher(Float64MultiArray, '/tianji_arm/left/left_zsp_para', qos)
        self.right_zsp_para_pub = self.create_publisher(Float64MultiArray, '/tianji_arm/right/right_zsp_para', qos)
        self.left_ee_pose_pub = self.create_publisher(Float64MultiArray, '/tianji_arm/left/left_ee_pose', qos)
        self.right_ee_pose_pub = self.create_publisher(Float64MultiArray, '/tianji_arm/right/right_ee_pose', qos)

        # 订阅者
        self.left_cmd_sub = self.create_subscription(
            JointState, LEFT_ARM_CMD_TOPIC, self._left_cmd_callback, qos)
        self.right_cmd_sub = self.create_subscription(
            JointState, RIGHT_ARM_CMD_TOPIC, self._right_cmd_callback, qos)

        # 服务
        self.create_service(SetBool, '/tianji_arm/switch_mode', self._switch_mode_callback)
        self.create_service(Trigger, '/tianji_arm/get_mode', self._get_mode_callback)

        # 定时器 (100Hz)
        self.create_timer(0.01, self._control_loop)

        self.get_logger().info(f"初始化完成，模式: {self._mode.value.upper()}")

    @property
    def mode(self) -> ControlMode:
        return self._mode

    # -------------------- 服务回调 --------------------

    def _switch_mode_callback(self, request: SetBool.Request, response: SetBool.Response):
        new_mode = ControlMode.INFERENCE if request.data else ControlMode.TELEOP
        if self._mode != new_mode:
            self._mode = new_mode
            self.get_logger().info(f"切换到 {new_mode.value} 模式")
        response.success = True
        response.message = f"当前模式: {new_mode.value}"
        return response

    def _get_mode_callback(self, request: Trigger.Request, response: Trigger.Response):
        response.success = True
        response.message = self._mode.value
        return response

    # -------------------- 订阅回调 --------------------

    def _left_cmd_callback(self, msg: JointState):
        if self._mode == ControlMode.INFERENCE and msg.position:
            self.left_inference_joints = list(msg.position)

    def _right_cmd_callback(self, msg: JointState):
        if self._mode == ControlMode.INFERENCE and msg.position:
            self.right_inference_joints = list(msg.position)

    # -------------------- 控制循环 --------------------

    def _control_loop(self):
        self._publish_state()

        if self._mode == ControlMode.TELEOP:
            self._teleop_control()
        else:
            self._inference_control()

    def _teleop_control(self):
        """遥操作控制：TF → IK → 机器人"""
        # 查询 TF
        left_tf = self._lookup_transform("left_chest", "tianji_left")
        if left_tf is not None:
            self.left_pose = self._matrix_to_pose(left_tf)

        right_tf = self._lookup_transform("right_chest", "tianji_right")
        if right_tf is not None:
            self.right_pose = self._matrix_to_pose(right_tf)

        # 查询大臂方向
        left_arm_tf = self._lookup_transform("left_chest", "left_arm")
        if left_arm_tf is not None:
            self.left_y_axis = left_arm_tf[:3, 1]

        right_arm_tf = self._lookup_transform("right_chest", "right_arm")
        if right_arm_tf is not None:
            self.right_y_axis = right_arm_tf[:3, 1]

        # 日志（每3秒）
        self._log_counter += 1
        if self._log_counter >= 300:
            self._log_counter = 0
            self.get_logger().info(
                f"TF: left={'OK' if self.left_pose is not None else 'None'}, "
                f"right={'OK' if self.right_pose is not None else 'None'}")

        # 执行 IK 控制
        if self.left_pose is not None or self.right_pose is not None:
            # 更新零空间参数
            if self.left_y_axis is not None:
                self.controller.left_zsp_para = [*self.left_y_axis, 0, 0, 0]
            if self.right_y_axis is not None:
                self.controller.right_zsp_para = [*self.right_y_axis, 0, 0, 0]

            _, _, l_joints, r_joints = self.controller.move_to_pose_direct(
                left_pose=self.left_pose, right_pose=self.right_pose, unit='m')
            self._publish_command(l_joints, r_joints)

        # 发布零空间参数和末端位姿
        if self.left_pose is not None and self.right_pose is not None and self.controller.left_zsp_para is not None and self.controller.right_zsp_para is not None: 
            self._publish_zsp_para_and_pose()

    def _inference_control(self):
        """推理控制：joint_command → 机器人"""
        left, right = self.left_inference_joints, self.right_inference_joints
        if left is not None or right is not None:
            self.controller.move_to_joints_direct(left_joints=left, right_joints=right)

    # -------------------- 发布 --------------------

    def _publish_command(self, left: Optional[list], right: Optional[list]):
        stamp = self.get_clock().now().to_msg()
        if left is not None:
            msg = JointState()
            msg.header.stamp = stamp
            msg.header.frame_id = "left_base_cmd"
            msg.name = [f'left_joint_{i+1}' for i in range(7)]
            msg.position = list(left)
            self.left_cmd_pub.publish(msg)
        if right is not None:
            msg = JointState()
            msg.header.stamp = stamp
            msg.header.frame_id = "right_base_cmd"
            msg.name = [f'right_joint_{i+1}' for i in range(7)]
            msg.position = list(right)
            self.right_cmd_pub.publish(msg)

    def _publish_state(self):
        try:
            left_joints, right_joints = self.controller.get_current_joints()
            stamp = self.get_clock().now().to_msg()

            if left_joints is not None:
                msg = JointState()
                msg.header.stamp = stamp
                msg.header.frame_id = "left_base_state"
                msg.name = [f'left_joint_{i+1}' for i in range(7)]
                msg.position = list(left_joints)
                self.left_state_pub.publish(msg)

            if right_joints is not None:
                msg = JointState()
                msg.header.stamp = stamp
                msg.header.frame_id = "right_base_state"
                msg.name = [f'right_joint_{i+1}' for i in range(7)]
                msg.position = list(right_joints)
                self.right_state_pub.publish(msg)
        except Exception as e:
            self.get_logger().debug(f"发布状态异常: {e}")

    def _publish_zsp_para_and_pose(self):
        """发布零空间参数和末端位姿"""
        # 这一行其实没用到 stamp，如果 msg 不需要 header 可以删掉，或者给 pose 加上 header
        # stamp = self.get_clock().now().to_msg() 

        try:
            # --- 发布 left_zsp_para ---
            # 增加 try-except 是为了防止在读取属性的一瞬间 controller 被销毁
            if hasattr(self.controller, 'left_zsp_para') and self.controller.left_zsp_para is not None:
                raw_data = self.controller.left_zsp_para
                # 确保数据非空
                if len(raw_data) > 0:
                    msg = Float64MultiArray()
                    # 强制转换为 list 且元素为 float，确保兼容性
                    msg.data = [float(x) for x in raw_data]
                    self.left_zsp_para_pub.publish(msg)

            # --- 发布 right_zsp_para ---
            if hasattr(self.controller, 'right_zsp_para') and self.controller.right_zsp_para is not None:
                raw_data = self.controller.right_zsp_para
                if len(raw_data) > 0:
                    msg = Float64MultiArray()
                    msg.data = [float(x) for x in raw_data]
                    self.right_zsp_para_pub.publish(msg)

            # --- 发布 left_pose ---
            if self.left_pose is not None:
                msg = Float64MultiArray()
                # 同理，确保 pose 数据也是纯 float 列表
                msg.data = [float(x) for x in self.left_pose]
                self.left_ee_pose_pub.publish(msg)

            # --- 发布 right_pose ---
            if self.right_pose is not None:
                msg = Float64MultiArray()
                msg.data = [float(x) for x in self.right_pose]
                self.right_ee_pose_pub.publish(msg)

        except Exception as e:
            # 如果在关闭过程中发生任何奇怪的错误（如 C++ 对象已释放），仅打印日志而不让节点崩溃
            self.get_logger().warn(f"Failed to publish debug info during shutdown: {e}")

    # -------------------- TF 工具 --------------------

    def _lookup_transform(self, from_frame: str, to_frame: str) -> Optional[np.ndarray]:
        try:
            tf = self.tf_buffer.lookup_transform(from_frame, to_frame, rclpy.time.Time())
            return self._transform_to_matrix(tf)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None

    @staticmethod
    def _transform_to_matrix(transform) -> np.ndarray:
        t = transform.transform
        quat = [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]
        T = np.eye(4, dtype=np.float32)
        T[:3, :3] = R.from_quat(quat).as_matrix()
        T[:3, 3] = [t.translation.x, t.translation.y, t.translation.z]
        return T

    @staticmethod
    def _matrix_to_pose(matrix: np.ndarray) -> np.ndarray:
        """4x4 矩阵 → [x, y, z, RX, RY, RZ]（度）"""
        xyz = matrix[:3, 3]
        rpy = R.from_matrix(matrix[:3, :3]).as_euler('ZYX', degrees=True)[::-1]
        return np.array([xyz[0], xyz[1], xyz[2], rpy[0], rpy[1], rpy[2]])

    def shutdown(self):
        self.get_logger().info("正在关闭...")
        self.controller.disable_and_release()
        self.get_logger().info("已安全退出")


# -------------------- 入口函数 --------------------

def _parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="天机臂控制器")
    parser.add_argument("-c", "--config", help="配置文件路径")
    return parser.parse_args(argv)


def main(argv: Optional[list[str]] = None):
    program_name = sys.argv[0] if sys.argv else "tianji_arm_controller"
    raw_argv = sys.argv if argv is None else [program_name, *argv]
    cli_argv = remove_ros_args(raw_argv)[1:]
    args = _parse_args(cli_argv)

    # 加载配置
    config_path = args.config or get_package_config_path("tianji_output", "tianji_output.yaml")
    config = load_yaml_config(config_path)

    rclpy.init(args=raw_argv)
    node = TianjiArmControllerNode(robot_ip=config.get("robot_ip", "192.168.1.190"))

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

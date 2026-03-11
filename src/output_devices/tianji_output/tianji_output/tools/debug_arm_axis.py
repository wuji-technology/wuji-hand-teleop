#!/usr/bin/env python3
"""调试脚本：查看大臂 Y 轴在 chest 坐标系下的投影"""

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
import tf2_ros
import numpy as np
from scipy.spatial.transform import Rotation as R


class DebugArmAxisNode(Node):
    def __init__(self):
        super().__init__("debug_arm_axis")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # IK 参数（仅用于显示）
        self.right_zsp_para = [0, 0, -1, 0, 0, 0]

        # 每 3 秒打印一次
        self.timer = self.create_timer(3.0, self.print_debug_info)

        self.get_logger().info("Debug node started. Waiting for TF...")

    def print_debug_info(self):
        # 查询 right_chest -> right_arm (大臂方向)
        right_arm_transform = self._lookup_transform("right_chest", "right_arm")
        if right_arm_transform is not None:
            y_axis = right_arm_transform[:3, 1]
            # 转换为 Python float，避免 numpy 类型问题
            self.right_zsp_para = [float(y_axis[0]), float(y_axis[1]), float(y_axis[2]), 0.0, 0.0, 0.0]
            self.get_logger().info(
                f"Right arm Y-axis in right_chest: [{y_axis[0]:.3f}, {y_axis[1]:.3f}, {y_axis[2]:.3f}]"
            )
            self.get_logger().info(
                f"  -> zsp_para: {self.right_zsp_para}"
            )
        else:
            self.get_logger().warn("right_chest -> right_arm TF not available")

        # 查询 right_chest -> tianji_right (末端位姿)
        tianji_transform = self._lookup_transform("right_chest", "tianji_right")
        if tianji_transform is not None:
            pose = self._matrix_to_pose_xyzrpy(tianji_transform)
            # 转换为 Python float 列表
            pose_mm = [float(pose[0]) * 1000, float(pose[1]) * 1000, float(pose[2]) * 1000,
                       float(pose[3]), float(pose[4]), float(pose[5])]

            self.get_logger().info(
                f"Tianji right pose in right_chest: "
                f"[{pose_mm[0]:.1f}, {pose_mm[1]:.1f}, {pose_mm[2]:.1f}] mm, "
                f"[{pose_mm[3]:.1f}, {pose_mm[4]:.1f}, {pose_mm[5]:.1f}] deg"
            )
        else:
            self.get_logger().warn("right_chest -> tianji_right TF not available")

    def _lookup_transform(self, from_frame: str, to_frame: str):
        try:
            transform = self.tf_buffer.lookup_transform(
                from_frame, to_frame, rclpy.time.Time()
            )
            return self._transform_to_matrix(transform)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None

    @staticmethod
    def _transform_to_matrix(transform):
        t = transform.transform
        quat = [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]
        T = np.eye(4, dtype=np.float32)
        T[:3, :3] = R.from_quat(quat).as_matrix()
        T[:3, 3] = [t.translation.x, t.translation.y, t.translation.z]
        return T

    @staticmethod
    def _matrix_to_pose_xyzrpy(matrix: np.ndarray) -> np.ndarray:
        """Convert 4x4 transformation matrix to [x, y, z, RX, RY, RZ] in degrees."""
        xyz = matrix[:3, 3]
        rpy_deg = R.from_matrix(matrix[:3, :3]).as_euler('ZYX', degrees=True)[::-1]
        return np.array([xyz[0], xyz[1], xyz[2], rpy_deg[0], rpy_deg[1], rpy_deg[2]])


def main():
    rclpy.init()
    node = DebugArmAxisNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

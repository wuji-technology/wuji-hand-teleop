#!/usr/bin/env python3
"""
三相机统一查看器
在一个窗口中显示三个相机画面（立体相机、左腕、右腕）
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
from cv_bridge import CvBridge
import threading


class TripleCameraViewer(Node):
    def __init__(self):
        super().__init__('triple_camera_viewer')

        self.bridge = CvBridge()

        # 图像缓存
        self.stereo_img = None
        self.left_wrist_img = None
        self.right_wrist_img = None

        # 锁
        self.lock = threading.Lock()

        # 接收计数（用于调试）
        self.stereo_count = 0
        self.left_wrist_count = 0
        self.right_wrist_count = 0

        # 创建适合图像传输的 QoS 配置 (BEST_EFFORT)
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 订阅三个相机话题
        self.stereo_sub = self.create_subscription(
            CompressedImage,
            '/stereo/left/compressed',
            self.stereo_callback,
            image_qos  # 使用 BEST_EFFORT QoS
        )

        self.left_wrist_sub = self.create_subscription(
            CompressedImage,
            '/cam_left_wrist/color/image_rect_raw/compressed',
            self.left_wrist_callback,
            image_qos  # 使用 BEST_EFFORT QoS
        )

        self.right_wrist_sub = self.create_subscription(
            CompressedImage,
            '/cam_right_wrist/color/image_rect_raw/compressed',
            self.right_wrist_callback,
            image_qos  # 使用 BEST_EFFORT QoS
        )

        self.get_logger().info('三相机查看器已启动')
        self.get_logger().info('订阅话题:')
        self.get_logger().info('  - /stereo/left/compressed')
        self.get_logger().info('  - /cam_left_wrist/color/image_rect_raw/compressed')
        self.get_logger().info('  - /cam_right_wrist/color/image_rect_raw/compressed')

    def stereo_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if img is None:
                self.get_logger().warning(f'立体相机解码失败: 图像为 None (数据大小: {len(msg.data)})')
                return

            with self.lock:
                self.stereo_img = img
                self.stereo_count += 1
                if self.stereo_count == 1:
                    self.get_logger().info(f'✓ 立体相机首帧已接收 (分辨率: {img.shape[1]}x{img.shape[0]}, 格式: {msg.format})')
                elif self.stereo_count % 100 == 0:
                    self.get_logger().info(f'立体相机已接收 {self.stereo_count} 帧')
        except Exception as e:
            self.get_logger().error(f'立体相机解码错误: {e}')

    def left_wrist_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            # 旋转180度
            img = cv2.rotate(img, cv2.ROTATE_180)
            with self.lock:
                self.left_wrist_img = img
                self.left_wrist_count += 1
                if self.left_wrist_count == 1:
                    self.get_logger().info(f'✓ 左腕相机首帧已接收 (分辨率: {img.shape[1]}x{img.shape[0]})')
        except Exception as e:
            self.get_logger().error(f'左腕相机解码错误: {e}')

    def right_wrist_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            # 旋转180度
            img = cv2.rotate(img, cv2.ROTATE_180)
            with self.lock:
                self.right_wrist_img = img
                self.right_wrist_count += 1
                if self.right_wrist_count == 1:
                    self.get_logger().info(f'✓ 右腕相机首帧已接收 (分辨率: {img.shape[1]}x{img.shape[0]})')
        except Exception as e:
            self.get_logger().error(f'右腕相机解码错误: {e}')

    def get_combined_image(self, target_height=480):
        """
        将三个相机画面合并为一个图像
        布局：立体相机在上，左右腕相机在下
        """
        with self.lock:
            images = []
            labels = []

            # 立体相机
            if self.stereo_img is not None:
                images.append(self.stereo_img.copy())
                labels.append('Stereo (L+R)')
            else:
                images.append(self._create_placeholder(640, target_height, 'Stereo Camera'))
                labels.append('Stereo (L+R)')

            # 左腕相机
            if self.left_wrist_img is not None:
                images.append(self.left_wrist_img.copy())
                labels.append('Left Wrist')
            else:
                images.append(self._create_placeholder(640, target_height, 'Left Wrist'))
                labels.append('Left Wrist')

            # 右腕相机
            if self.right_wrist_img is not None:
                images.append(self.right_wrist_img.copy())
                labels.append('Right Wrist')
            else:
                images.append(self._create_placeholder(640, target_height, 'Right Wrist'))
                labels.append('Right Wrist')

        # 调整所有图像到相同高度
        resized_images = []
        for img, label in zip(images, labels):
            h, w = img.shape[:2]
            scale = target_height / h
            new_w = int(w * scale)
            new_h = target_height
            resized = cv2.resize(img, (new_w, new_h))

            # 添加标签
            cv2.putText(resized, label, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            resized_images.append(resized)

        # 上下布局：立体相机在上，左右腕并排在下
        top_row = resized_images[0]
        bottom_row = cv2.hconcat([resized_images[1], resized_images[2]])

        # 调整宽度使上下对齐
        top_width = top_row.shape[1]
        bottom_width = bottom_row.shape[1]

        if top_width > bottom_width:
            # 底部需要填充
            padding = top_width - bottom_width
            bottom_row = cv2.copyMakeBorder(
                bottom_row, 0, 0, 0, padding,
                cv2.BORDER_CONSTANT, value=(0, 0, 0)
            )
        elif bottom_width > top_width:
            # 顶部需要填充
            padding = bottom_width - top_width
            top_row = cv2.copyMakeBorder(
                top_row, 0, 0, 0, padding,
                cv2.BORDER_CONSTANT, value=(0, 0, 0)
            )

        # 上下拼接
        combined = cv2.vconcat([top_row, bottom_row])

        return combined

    def _create_placeholder(self, width, height, text):
        """创建占位图像"""
        img = np.zeros((height, width, 3), dtype=np.uint8)
        cv2.putText(img, f'Waiting for {text}...',
                   (width//2 - 200, height//2),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (100, 100, 100), 2)
        return img


def main(args=None):
    rclpy.init(args=args)

    viewer = TripleCameraViewer()

    # ROS2 spin 在单独线程
    spin_thread = threading.Thread(target=rclpy.spin, args=(viewer,), daemon=True)
    spin_thread.start()

    # OpenCV 显示窗口
    window_name = 'Camera Monitor'
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 1920, 960)

    print("\n" + "="*60)
    print("相机监控已启动")
    print("="*60)
    print("布局: 立体相机在上，左右腕相机在下")
    print("按 'q' 或 ESC 键退出，或直接关闭窗口")
    print("="*60 + "\n")

    try:
        while rclpy.ok():
            combined_img = viewer.get_combined_image()
            cv2.imshow(window_name, combined_img)

            # 检查窗口是否被关闭
            if cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) < 1:
                print("\n检测到窗口关闭")
                break

            key = cv2.waitKey(30) & 0xFF
            if key == ord('q') or key == 27:  # q 或 ESC
                print("\n用户按键退出")
                break

    except KeyboardInterrupt:
        print("\n检测到 Ctrl+C")
        pass

    finally:
        print("关闭相机监控...")
        cv2.destroyAllWindows()
        viewer.destroy_node()
        rclpy.shutdown()
        print("已退出")


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
Triple Camera Unified Viewer
Displays three camera feeds in one window (stereo camera, left wrist, right wrist)
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

        # Image cache
        self.stereo_img = None
        self.left_wrist_img = None
        self.right_wrist_img = None

        # Lock
        self.lock = threading.Lock()

        # Receive count (for debugging)
        self.stereo_count = 0
        self.left_wrist_count = 0
        self.right_wrist_count = 0

        # Create QoS config suitable for image transport (BEST_EFFORT)
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribe to three camera topics
        self.stereo_sub = self.create_subscription(
            CompressedImage,
            '/stereo/left/compressed',
            self.stereo_callback,
            image_qos  # Use BEST_EFFORT QoS
        )

        self.left_wrist_sub = self.create_subscription(
            CompressedImage,
            '/cam_left_wrist/color/image_rect_raw/compressed',
            self.left_wrist_callback,
            image_qos  # Use BEST_EFFORT QoS
        )

        self.right_wrist_sub = self.create_subscription(
            CompressedImage,
            '/cam_right_wrist/color/image_rect_raw/compressed',
            self.right_wrist_callback,
            image_qos  # Use BEST_EFFORT QoS
        )

        self.get_logger().info('Triple camera viewer started')
        self.get_logger().info('Subscribed topics:')
        self.get_logger().info('  - /stereo/left/compressed')
        self.get_logger().info('  - /cam_left_wrist/color/image_rect_raw/compressed')
        self.get_logger().info('  - /cam_right_wrist/color/image_rect_raw/compressed')

    def stereo_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if img is None:
                self.get_logger().warning(f'Stereo camera decode failed: image is None (data size: {len(msg.data)})')
                return

            with self.lock:
                self.stereo_img = img
                self.stereo_count += 1
                if self.stereo_count == 1:
                    self.get_logger().info(f'Stereo camera first frame received (resolution: {img.shape[1]}x{img.shape[0]}, format: {msg.format})')
                elif self.stereo_count % 100 == 0:
                    self.get_logger().info(f'Stereo camera received {self.stereo_count} frames')
        except Exception as e:
            self.get_logger().error(f'Stereo camera decode error: {e}')

    def left_wrist_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            # Rotate 180 degrees
            img = cv2.rotate(img, cv2.ROTATE_180)
            with self.lock:
                self.left_wrist_img = img
                self.left_wrist_count += 1
                if self.left_wrist_count == 1:
                    self.get_logger().info(f'Left wrist camera first frame received (resolution: {img.shape[1]}x{img.shape[0]})')
        except Exception as e:
            self.get_logger().error(f'Left wrist camera decode error: {e}')

    def right_wrist_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            # Rotate 180 degrees
            img = cv2.rotate(img, cv2.ROTATE_180)
            with self.lock:
                self.right_wrist_img = img
                self.right_wrist_count += 1
                if self.right_wrist_count == 1:
                    self.get_logger().info(f'Right wrist camera first frame received (resolution: {img.shape[1]}x{img.shape[0]})')
        except Exception as e:
            self.get_logger().error(f'Right wrist camera decode error: {e}')

    def get_combined_image(self, target_height=480):
        """
        Combine three camera feeds into one image
        Layout: stereo camera on top, left/right wrist cameras on bottom
        """
        with self.lock:
            images = []
            labels = []

            # Stereo camera
            if self.stereo_img is not None:
                images.append(self.stereo_img.copy())
                labels.append('Stereo (L+R)')
            else:
                images.append(self._create_placeholder(640, target_height, 'Stereo Camera'))
                labels.append('Stereo (L+R)')

            # Left wrist camera
            if self.left_wrist_img is not None:
                images.append(self.left_wrist_img.copy())
                labels.append('Left Wrist')
            else:
                images.append(self._create_placeholder(640, target_height, 'Left Wrist'))
                labels.append('Left Wrist')

            # Right wrist camera
            if self.right_wrist_img is not None:
                images.append(self.right_wrist_img.copy())
                labels.append('Right Wrist')
            else:
                images.append(self._create_placeholder(640, target_height, 'Right Wrist'))
                labels.append('Right Wrist')

        # Resize all images to the same height
        resized_images = []
        for img, label in zip(images, labels):
            h, w = img.shape[:2]
            scale = target_height / h
            new_w = int(w * scale)
            new_h = target_height
            resized = cv2.resize(img, (new_w, new_h))

            # Add label
            cv2.putText(resized, label, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            resized_images.append(resized)

        # Vertical layout: stereo camera on top, left/right wrist side by side on bottom
        top_row = resized_images[0]
        bottom_row = cv2.hconcat([resized_images[1], resized_images[2]])

        # Adjust widths so top and bottom align
        top_width = top_row.shape[1]
        bottom_width = bottom_row.shape[1]

        if top_width > bottom_width:
            # Bottom needs padding
            padding = top_width - bottom_width
            bottom_row = cv2.copyMakeBorder(
                bottom_row, 0, 0, 0, padding,
                cv2.BORDER_CONSTANT, value=(0, 0, 0)
            )
        elif bottom_width > top_width:
            # Top needs padding
            padding = bottom_width - top_width
            top_row = cv2.copyMakeBorder(
                top_row, 0, 0, 0, padding,
                cv2.BORDER_CONSTANT, value=(0, 0, 0)
            )

        # Stack vertically
        combined = cv2.vconcat([top_row, bottom_row])

        return combined

    def _create_placeholder(self, width, height, text):
        """Create placeholder image"""
        img = np.zeros((height, width, 3), dtype=np.uint8)
        cv2.putText(img, f'Waiting for {text}...',
                   (width//2 - 200, height//2),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (100, 100, 100), 2)
        return img


def main(args=None):
    rclpy.init(args=args)

    viewer = TripleCameraViewer()

    # ROS2 spin in a separate thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(viewer,), daemon=True)
    spin_thread.start()

    # OpenCV display window
    window_name = 'Camera Monitor'
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 1920, 960)

    print("\n" + "="*60)
    print("Camera monitor started")
    print("="*60)
    print("Layout: stereo camera on top, left/right wrist cameras on bottom")
    print("Press 'q' or ESC to exit, or close the window directly")
    print("="*60 + "\n")

    try:
        while rclpy.ok():
            combined_img = viewer.get_combined_image()
            cv2.imshow(window_name, combined_img)

            # Check if window was closed
            if cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) < 1:
                print("\nWindow close detected")
                break

            key = cv2.waitKey(30) & 0xFF
            if key == ord('q') or key == 27:  # q or ESC
                print("\nUser key exit")
                break

    except KeyboardInterrupt:
        print("\nCtrl+C detected")
        pass

    finally:
        print("Closing camera monitor...")
        cv2.destroyAllWindows()
        viewer.destroy_node()
        rclpy.shutdown()
        print("Exited")


if __name__ == '__main__':
    main()

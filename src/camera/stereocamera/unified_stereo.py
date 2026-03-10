#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
统一双目相机节点入口
Unified Stereo Camera Node Entry Point

替代 stereo_pico_server + stereo_publisher 的两进程架构。
单进程处理 ROS2 发布 + PICO H.264 串流，无需 v4l2loopback。

Usage:
    ros2 run camera unified_stereo
    ros2 run camera unified_stereo --device /dev/stereo_camera --fps 30 --quality 70
"""

import argparse
import logging
import signal
import sys
import traceback

try:
    from stereocamera.config_loader import load_stereo_head_config
except ImportError as e:
    print(f"[ERROR] Cannot import config_loader: {e}")
    sys.exit(1)

try:
    from stereocamera.teleopVision.unified_stereo_node import UnifiedStereoNode
except ImportError as e:
    print(f"[ERROR] Cannot import UnifiedStereoNode: {e}")
    sys.exit(1)


def main():
    logging.basicConfig(
        level=logging.INFO,
        format='[%(name)s] %(levelname)s: %(message)s',
    )

    parser = argparse.ArgumentParser(
        description='Unified stereo camera node (ROS2 + PICO H.264, no v4l2loopback)'
    )
    parser.add_argument('--device', default=None, help='Camera device path')
    parser.add_argument('--fps', default=None, type=float, help='ROS2 publish FPS')
    parser.add_argument('--quality', default=None, type=int, help='JPEG quality 1-100')
    parser.add_argument('--width', default=None, type=int, help='Stereo frame width')
    parser.add_argument('--height', default=None, type=int, help='Stereo frame height')
    parser.add_argument('--bitrate', default=None, type=int, help='H.264 bitrate in bps')
    parser.add_argument('--config', default=None, help='Config file path')

    args, _ = parser.parse_known_args()

    # Load config
    head_cfg = load_stereo_head_config(args.config)
    resolution = head_cfg.get('resolution', {})

    # CLI overrides config
    device = args.device or head_cfg.get('camera_device', '/dev/stereo_camera')
    fps = args.fps or resolution.get('fps', 30)
    quality = args.quality or 70
    width = args.width or resolution.get('width', 2560)
    height = args.height or resolution.get('height', 720)
    bitrate = args.bitrate or 30_000_000

    node = UnifiedStereoNode(
        camera_device=device,
        width=width,
        height=height,
        ros2_fps=fps,
        jpeg_quality=quality,
        h264_bitrate=bitrate
    )

    def signal_handler(sig, frame):
        print(f"\n[INFO] Received signal ({sig}), shutting down...")
        node.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        node.start()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"\n[ERROR] {e}")
        traceback.print_exc()
        sys.exit(1)
    finally:
        node.stop()


if __name__ == '__main__':
    main()

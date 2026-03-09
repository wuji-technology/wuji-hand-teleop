#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
统一双目相机节点 - 单进程处理 ROS2 发布 + PICO H.264 串流

架构:
    /dev/stereo_camera → OpenCV (MJPEG 60fps)
      ├── ROS2: split L/R → JPEG → /stereo/{left,right}/compressed (30fps)
      └── PICO: BGR24 → FFmpeg stdin → H.264 stdout → TCP (60fps, on-demand)
          └── XRobo TCP 13579: OPEN_CAMERA → MEDIA_DECODER_READY → streaming

消除 v4l2loopback 依赖。单进程内完成所有相机输出。

作者: Liang ZHU
"""

import cv2
import os
import socket
import struct
import subprocess
import threading
import time
import logging
from typing import Optional
from enum import Enum

from .ffmpeg_utils import check_nvenc_available, get_encoder_args, read_h264_frames, terminate_ffmpeg
from .xrobo_protocol import PacketParser, ProtocolConstants

logger = logging.getLogger(__name__)



class StreamState(Enum):
    IDLE = "idle"
    STARTING = "starting"
    STREAMING = "streaming"


class UnifiedStereoNode:
    """
    统一双目相机节点

    单进程处理:
    1. OpenCV 采集 (MJPEG, camera native fps)
    2. ROS2 JPEG 发布 (/stereo/{left,right}/compressed)
    3. XRobo 协议服务器 (TCP 13579, PICO 连接管理)
    4. H.264 编码发送 (FFmpeg rawvideo→H.264, TCP → PICO, on-demand)

    线程模型:
    - capture_thread: OpenCV read → shared frame buffer (Condition notify)
    - ros2_thread: wait frame → split → JPEG encode → publish (rate limited)
    - tcp_accept (main): listen 13579, handle PICO protocol
    - h264_feed_thread: wait frame → write to FFmpeg stdin (when PICO connected)
    - h264_send_thread: read FFmpeg stdout → TCP sendall (when PICO connected)
    """

    def __init__(self,
                 camera_device: str = "/dev/stereo_camera",
                 width: int = 2560,
                 height: int = 720,
                 ros2_fps: float = 30.0,
                 jpeg_quality: int = 70,
                 h264_bitrate: int = 30_000_000):
        # Config
        self.camera_device = camera_device
        self.width = width
        self.height = height
        self.ros2_fps = ros2_fps
        self.jpeg_quality = jpeg_quality
        self.h264_bitrate = h264_bitrate

        # Camera
        self.cap: Optional[cv2.VideoCapture] = None
        self.actual_fps = 60.0

        # Frame sharing: capture thread writes, consumers wait via Condition
        self._frame: Optional[object] = None
        self._frame_id: int = 0
        self._frame_cond = threading.Condition()

        # ROS2
        self._ros2_node = None
        self._pub_left = None
        self._pub_right = None

        # TCP server (XRobo protocol)
        self._tcp_server: Optional[socket.socket] = None
        self._tcp_available = False

        # H.264 streaming state
        self._stream_state = StreamState.IDLE
        self._stream_lock = threading.RLock()
        self._ffmpeg_process: Optional[subprocess.Popen] = None
        self._pico_socket: Optional[socket.socket] = None
        self._generation: int = 0
        self._pending_params: Optional[dict] = None
        self._pending_client: Optional[socket.socket] = None
        self._command_client: Optional[socket.socket] = None  # 推流期间的命令通道

        # ADB
        self._adb_connected = False

        # Control
        self.is_running = False

        # Stats
        self._capture_count = 0
        self._ros2_count = 0
        self._h264_count = 0

    # ==================== Camera ====================

    def _open_camera(self) -> bool:
        """Open camera with OpenCV V4L2 backend (MJPEG mode)"""
        device = self.camera_device
        if os.path.islink(device):
            device = os.path.realpath(device)

        logger.info(f"Opening camera: {device}")

        for attempt in range(15):
            cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
            if not cap.isOpened():
                logger.debug(f"Open attempt {attempt+1}/15 failed")
                time.sleep(1.0)
                continue

            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            cap.set(cv2.CAP_PROP_FPS, 60)  # Request max camera native fps

            ret, frame = cap.read()
            if ret and frame is not None:
                h, w = frame.shape[:2]
                self.actual_fps = cap.get(cv2.CAP_PROP_FPS)
                logger.info(f"Camera opened: {w}x{h} @ {self.actual_fps:.0f}fps (MJPEG)")
                self.cap = cap
                return True

            cap.release()
            time.sleep(1.0)

        logger.error(f"Failed to open camera: {self.camera_device}")
        return False

    def _capture_loop(self):
        """Camera capture thread - reads frames and notifies all consumers"""
        logger.info("Capture thread started")
        consecutive_failures = 0

        while self.is_running:
            ret, frame = self.cap.read()
            if not ret or frame is None:
                consecutive_failures += 1
                if consecutive_failures >= 60:
                    logger.error("Camera read failed 60 consecutive times, stopping")
                    self.is_running = False
                    break
                time.sleep(0.01)
                continue

            consecutive_failures = 0

            with self._frame_cond:
                self._frame = frame
                self._frame_id += 1
                self._capture_count += 1
                self._frame_cond.notify_all()

        logger.info(f"Capture thread ended ({self._capture_count} frames)")

    def _wait_frame(self, last_id: int, timeout: float = 1.0):
        """Wait for a new frame. Returns (frame, frame_id) or (None, last_id)."""
        with self._frame_cond:
            deadline = time.monotonic() + timeout
            while self._frame_id == last_id and self.is_running:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None, last_id
                self._frame_cond.wait(timeout=remaining)
            return self._frame, self._frame_id

    # ==================== ROS2 ====================

    def _init_ros2(self) -> bool:
        """Initialize ROS2 node and publishers"""
        try:
            import rclpy
            from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

            if not rclpy.ok():
                rclpy.init()

            self._ros2_node = rclpy.create_node('stereo_camera_publisher')

            from sensor_msgs.msg import CompressedImage
            qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=1
            )
            self._pub_left = self._ros2_node.create_publisher(
                CompressedImage, '/stereo/left/compressed', qos)
            self._pub_right = self._ros2_node.create_publisher(
                CompressedImage, '/stereo/right/compressed', qos)

            logger.info("ROS2: /stereo/{left,right}/compressed (BEST_EFFORT)")
            return True
        except Exception as e:
            logger.error(f"ROS2 init failed: {e}")
            return False

    def _ros2_loop(self):
        """ROS2 publish thread - rate-limited JPEG publishing"""
        import rclpy
        from sensor_msgs.msg import CompressedImage

        logger.info(f"ROS2 thread started ({self.ros2_fps}fps, JPEG Q{self.jpeg_quality})")

        last_id = 0
        frame_interval = 1.0 / self.ros2_fps
        last_publish = 0.0
        last_log = time.time()
        log_frame_count = 0
        encode_params = [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]

        while self.is_running and rclpy.ok():
            try:
                frame, last_id = self._wait_frame(last_id)
                if frame is None:
                    continue

                # Rate limit
                now = time.time()
                if now - last_publish < frame_interval * 0.9:
                    continue
                last_publish = now

                h, w = frame.shape[:2]
                half_w = w // 2
                if half_w < 100:
                    continue

                # Split left/right and encode JPEG
                _, left_jpg = cv2.imencode('.jpg', frame[:, :half_w], encode_params)
                _, right_jpg = cv2.imencode('.jpg', frame[:, half_w:], encode_params)

                stamp = self._ros2_node.get_clock().now().to_msg()

                msg_l = CompressedImage()
                msg_l.header.stamp = stamp
                msg_l.header.frame_id = 'stereo_left'
                msg_l.format = 'jpeg'
                msg_l.data = left_jpg.tobytes()
                self._pub_left.publish(msg_l)

                msg_r = CompressedImage()
                msg_r.header.stamp = stamp
                msg_r.header.frame_id = 'stereo_right'
                msg_r.format = 'jpeg'
                msg_r.data = right_jpg.tobytes()
                self._pub_right.publish(msg_r)

                self._ros2_count += 1
                log_frame_count += 1

                if now - last_log >= 1.0:
                    logger.info(f"ROS2: {log_frame_count}fps (total {self._ros2_count}) | "
                               f"L:{len(left_jpg)}B R:{len(right_jpg)}B")
                    last_log = now
                    log_frame_count = 0

                rclpy.spin_once(self._ros2_node, timeout_sec=0)

            except Exception as e:
                if self.is_running:
                    logger.warning(f"ROS2 publish error: {e}")
                break

        logger.info(f"ROS2 thread ended ({self._ros2_count} frames published)")

    # ==================== ADB ====================

    def _check_adb(self):
        """检测 PICO 是否通过 USB 连接 (决定视频数据走 ADB forward 还是直连 IP)

        ADB reverse 端口由 docker/scripts/adb_watchdog.sh 统一管理,
        此处只做连接检测, 不设置 reverse 端口。
        """
        try:
            result = subprocess.run(['adb', 'devices'],
                                    capture_output=True, text=True, timeout=5)
            lines = result.stdout.strip().split('\n')[1:]
            devices = [l.split('\t')[0] for l in lines if '\tdevice' in l]
            self._adb_connected = len(devices) > 0
            if self._adb_connected:
                logger.info(f"ADB: PICO USB detected ({devices[0]})")
            else:
                logger.info("ADB: No USB device, WiFi mode")
        except (FileNotFoundError, subprocess.TimeoutExpired):
            self._adb_connected = False

    def _setup_adb_forward(self, port: int):
        """设置 ADB forward (PC→PICO) 用于 H.264 视频推流

        与 watchdog 管理的 reverse 端口不同, forward 是每次视频连接动态创建的,
        端口号由 PICO MEDIA_DECODER_READY 指定。
        """
        try:
            result = subprocess.run(['adb', 'forward', f'tcp:{port}', f'tcp:{port}'],
                                    capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                logger.info(f"ADB forward tcp:{port} → PICO:{port}")
            else:
                logger.warning(f"ADB forward failed: {result.stderr.strip()}")
        except Exception as e:
            logger.warning(f"ADB forward error: {e}")

    # ==================== TCP Server (XRobo Protocol) ====================

    def _start_tcp_server(self):
        """Try to start TCP server on port 13579"""
        try:
            self._tcp_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._tcp_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._tcp_server.bind(('0.0.0.0', ProtocolConstants.TCP_PORT))
            self._tcp_server.listen(1)
            self._tcp_available = True
            logger.info(f"XRobo TCP server listening on :{ProtocolConstants.TCP_PORT}")
        except OSError as e:
            logger.warning(f"Port {ProtocolConstants.TCP_PORT} taken (PC-Service?): {e}")
            logger.warning("PICO H.264 streaming disabled, ROS2-only mode")
            self._tcp_server = None
            self._tcp_available = False

    def _tcp_accept_loop(self):
        """Accept PICO connections (runs in main thread)"""
        logger.info("Waiting for PICO connections...")
        while self.is_running:
            try:
                self._tcp_server.settimeout(1.0)
                client, addr = self._tcp_server.accept()
                client_id = f"{addr[0]}:{addr[1]}"
                logger.info(f"PICO client connected: {client_id}")

                t = threading.Thread(target=self._handle_client,
                                     args=(client, client_id), daemon=True)
                t.start()
            except socket.timeout:
                continue
            except Exception as e:
                if self.is_running:
                    logger.error(f"Accept error: {e}")

    def _handle_client(self, client: socket.socket, client_id: str):
        """Handle a PICO client connection (protocol parsing)

        命令通道生命周期:
        - 协商阶段: 5s timeout (快速检测死连接)
        - 推流阶段: 阻塞等待 (推流结束时由外部 shutdown 唤醒)
        """
        buffer = b''
        try:
            client.settimeout(5.0)
            while self.is_running:
                try:
                    data = client.recv(4096)
                    if not data:
                        break

                    buffer += data
                    _, json_data = PacketParser.parse(buffer)

                    if json_data:
                        buffer = b''
                        func = json_data.get('functionName', '')
                        value = json_data.get('value', {})

                        if func in ('OpenCamera', 'StartReceivePcCamera'):
                            self._handle_open_camera(value, client)
                        elif func == 'MediaDecoderReady':
                            self._handle_media_decoder_ready(value, client)
                            # 推流开始: 切换阻塞模式, 等待 CLOSE_CAMERA 或外部 shutdown
                            client.settimeout(None)
                            with self._stream_lock:
                                self._command_client = client
                        elif func == 'StopReceivePcCamera':
                            self._stop_h264_streaming()
                        elif func == 'requestCameraList':
                            self._send_camera_list(client)
                        else:
                            logger.debug(f"Unknown command: {func}")

                except socket.timeout:
                    break
                except (ConnectionResetError, OSError):
                    break
        except Exception as e:
            logger.warning(f"Client handler error: {e}")
        finally:
            with self._stream_lock:
                if self._command_client is client:
                    self._command_client = None
            client.close()
            logger.debug(f"Client disconnected: {client_id}")

    def _handle_open_camera(self, params: dict, client: socket.socket):
        """Handle OPEN_CAMERA - save params, wait for MEDIA_DECODER_READY"""
        logger.info(f"OPEN_CAMERA: {params.get('width', 2560)}x{params.get('height', 720)} "
                     f"@ {params.get('fps', 60)}fps, "
                     f"{params.get('bitrate', 30_000_000)//1000000}Mbps")
        with self._stream_lock:
            self._pending_params = params
            self._pending_client = client

    def _handle_media_decoder_ready(self, value, client: socket.socket):
        """Handle MEDIA_DECODER_READY - start H.264 streaming"""
        port = 12345
        if isinstance(value, dict):
            port = value.get('port', 12345)
        elif isinstance(value, bytes) and len(value) >= 4:
            port = struct.unpack('<I', value[:4])[0]

        logger.info(f"MEDIA_DECODER_READY, video port={port}")

        with self._stream_lock:
            if self._pending_params is None:
                logger.warning("No pending OPEN_CAMERA params")
                return
            params = self._pending_params
            self._pending_params = None
            self._pending_client = None

        self._start_h264_streaming(params, client)

    def _send_camera_list(self, client: socket.socket):
        """Send camera list response"""
        response = PacketParser.build_response(
            ProtocolConstants.CMD_FUNCTION,
            {"functionName": "onCameraList", "cameras": [
                {"name": "ADB", "type": "USB"},
                {"name": "WIFI", "type": "USB"}
            ]}
        )
        try:
            client.send(response)
        except Exception:
            pass

    # ==================== H.264 Streaming ====================

    def _start_h264_streaming(self, params: dict, client: socket.socket):
        """Start H.264 encoding + TCP streaming to PICO"""
        with self._stream_lock:
            if self._stream_state == StreamState.STARTING:
                logger.warning("H.264 streaming already starting, ignoring")
                return

            # Stop existing stream
            if self._stream_state == StreamState.STREAMING:
                logger.info("Stopping existing H.264 stream for new connection")
                self._stop_h264_internal()
                time.sleep(0.3)

            self._stream_state = StreamState.STARTING
            self._generation += 1
            gen = self._generation

        target_ip = params.get('clientIp', params.get('ip', ''))
        target_port = params.get('clientPort', params.get('port', 12345))
        bitrate = params.get('bitrate', self.h264_bitrate)

        if self._adb_connected:
            logger.info(f"ADB mode: target IP → 127.0.0.1")
            target_ip = "127.0.0.1"
            self._setup_adb_forward(target_port)

        logger.info(f"Starting H.264: {target_ip}:{target_port} "
                     f"({bitrate//1000000}Mbps, gen={gen})")

        try:
            # 1. Start FFmpeg (rawvideo stdin → H.264 stdout)
            ffmpeg = self._start_ffmpeg_encoder(bitrate)
            if not ffmpeg:
                with self._stream_lock:
                    self._stream_state = StreamState.IDLE
                return

            # 2. Start feed thread (camera frames → FFmpeg stdin)
            feed_t = threading.Thread(target=self._h264_feed_loop,
                                      args=(ffmpeg, gen), daemon=True)
            feed_t.start()

            # 3. Connect TCP to PICO
            #    connect() 返回即表示 PICO 已 accept (WiFi 直连 / ADB 端到端代理)
            #    无需额外 ACK 握手，立即启动 send 线程
            tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            tcp_sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            tcp_sock.settimeout(5.0)

            try:
                tcp_sock.connect((target_ip, target_port))
                logger.info(f"TCP connected: {target_ip}:{target_port}")
            except Exception as e:
                logger.error(f"TCP connect failed: {e}")
                tcp_sock.close()
                self.is_running and terminate_ffmpeg(ffmpeg, "FFmpeg")
                with self._stream_lock:
                    self._stream_state = StreamState.IDLE
                return

            with self._stream_lock:
                self._ffmpeg_process = ffmpeg
                self._pico_socket = tcp_sock
                self._stream_state = StreamState.STREAMING

            # 4. Start send thread (H.264 stdout → TCP → PICO)
            send_t = threading.Thread(target=self._h264_send_loop,
                                      args=(ffmpeg, tcp_sock, gen,
                                            target_ip, target_port),
                                      daemon=True)
            send_t.start()

            logger.info("H.264 streaming active")

        except Exception as e:
            logger.error(f"Start H.264 streaming failed: {e}")
            with self._stream_lock:
                self._stream_state = StreamState.IDLE

    def _start_ffmpeg_encoder(self, bitrate: int) -> Optional[subprocess.Popen]:
        """Start FFmpeg: rawvideo stdin → H.264 stdout"""
        use_nvenc = check_nvenc_available()
        encoder_args = get_encoder_args(bitrate // 1000, use_nvenc)

        ffmpeg_cmd = [
            'ffmpeg', '-y',
            '-f', 'rawvideo',
            '-pixel_format', 'bgr24',
            '-video_size', f'{self.width}x{self.height}',
            '-framerate', str(int(self.actual_fps)),
            '-i', 'pipe:0',
        ] + encoder_args + [
            '-f', 'h264', 'pipe:1'
        ]

        try:
            logger.info(f"FFmpeg cmd: {' '.join(ffmpeg_cmd)}")
            process = subprocess.Popen(
                ffmpeg_cmd,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            threading.Thread(target=self._log_ffmpeg, args=(process,), daemon=True).start()
            return process
        except Exception as e:
            logger.error(f"FFmpeg start failed: {e}")
            return None

    def _log_ffmpeg(self, process: subprocess.Popen):
        """Log FFmpeg stderr in background"""
        try:
            for line in process.stderr:
                text = line.decode('utf-8', errors='ignore').strip()
                if text and 'frame=' not in text:
                    logger.debug(f"[FFmpeg] {text}")
        except Exception:
            pass

    def _h264_feed_loop(self, ffmpeg: subprocess.Popen, generation: int):
        """Feed camera frames (BGR24) to FFmpeg stdin"""
        logger.info(f"H.264 feed thread started (gen={generation})")
        last_id = 0
        count = 0
        expected_shape = (self.height, self.width, 3)

        try:
            # NOTE: Do NOT check _stream_state here. The feed thread starts while
            # state is STARTING (before TCP connects). Only check generation and
            # is_running. The generation check handles "replaced by new connection".
            while self.is_running and self._generation == generation:

                frame, last_id = self._wait_frame(last_id, timeout=0.5)
                if frame is None:
                    continue

                # Verify frame dimensions match FFmpeg expectation
                if frame.shape != expected_shape:
                    logger.warning(f"Frame shape {frame.shape} != expected {expected_shape}")
                    continue

                try:
                    ffmpeg.stdin.write(frame.tobytes())
                    count += 1
                except (BrokenPipeError, OSError):
                    logger.warning("FFmpeg stdin broken")
                    break
        except Exception as e:
            logger.error(f"Feed loop error: {e}")
        finally:
            try:
                ffmpeg.stdin.close()
            except Exception:
                pass
            logger.info(f"Feed thread ended: {count} frames (gen={generation})")

    def _h264_send_loop(self, ffmpeg: subprocess.Popen,
                         tcp_sock: socket.socket, generation: int,
                         target_ip: str, target_port: int):
        """Read H.264 from FFmpeg stdout and send to PICO via TCP."""
        logger.info(f"H.264 send thread started (gen={generation})")
        total_count = 0
        start = time.time()
        last_log = start

        try:
            tcp_sock.settimeout(5.0)

            for frame_data in read_h264_frames(
                ffmpeg.stdout,
                is_running=lambda: (self.is_running and
                                    self._stream_state == StreamState.STREAMING and
                                    self._generation == generation),
                poll_interval=0.1
            ):
                try:
                    tcp_sock.sendall(struct.pack('>I', len(frame_data)) + frame_data)
                    total_count += 1
                except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError) as e:
                    logger.warning(f"PICO disconnected after {total_count} frames: {e}")
                    break
                except socket.timeout:
                    logger.warning("TCP send timeout, PICO may be disconnected")
                    break

                now = time.time()
                if now - last_log >= 1.0:
                    elapsed = now - start
                    fps = total_count / elapsed if elapsed > 0 else 0
                    logger.info(f"PICO H.264: {total_count} frames | {fps:.1f}fps")
                    last_log = now

        except Exception as e:
            logger.error(f"Send loop error: {e}")
        finally:
            logger.info(f"Send thread ended: {total_count} frames (gen={generation})")

            # Close TCP
            try:
                tcp_sock.shutdown(socket.SHUT_RDWR)
            except Exception:
                pass
            try:
                tcp_sock.close()
            except Exception:
                pass

            # Clean up if still current generation
            with self._stream_lock:
                if self._generation == generation:
                    logger.info("Reverting to ROS2-only mode, waiting for PICO reconnect...")
                    terminate_ffmpeg(ffmpeg, "FFmpeg (H.264)")
                    self._ffmpeg_process = None
                    self._pico_socket = None
                    self._stream_state = StreamState.IDLE
                    # 唤醒命令通道 handler (阻塞在 recv 上)
                    self._shutdown_command_client()
                else:
                    logger.info(f"New connection active (gen={self._generation}), skip cleanup")

            self._h264_count += total_count

    def _stop_h264_streaming(self):
        """Stop H.264 streaming (thread-safe)"""
        logger.info("Stopping H.264 streaming...")
        with self._stream_lock:
            self._stop_h264_internal()

    def _stop_h264_internal(self):
        """Internal stop (caller must hold _stream_lock)"""
        if self._stream_state == StreamState.IDLE:
            return

        self._stream_state = StreamState.IDLE

        if self._ffmpeg_process:
            terminate_ffmpeg(self._ffmpeg_process, "FFmpeg")
            self._ffmpeg_process = None

        if self._pico_socket:
            try:
                self._pico_socket.close()
            except Exception:
                pass
            self._pico_socket = None

        self._shutdown_command_client()

    def _shutdown_command_client(self):
        """唤醒阻塞在 recv 上的命令通道 handler (caller must hold _stream_lock)"""
        if self._command_client:
            try:
                self._command_client.shutdown(socket.SHUT_RDWR)
            except Exception:
                pass
            self._command_client = None

    # ==================== Lifecycle ====================

    def start(self) -> bool:
        """Start the unified node"""
        logger.info("=" * 60)
        logger.info("Unified Stereo Camera Node")
        logger.info("=" * 60)
        logger.info(f"  Device: {self.camera_device}")
        logger.info(f"  Resolution: {self.width}x{self.height}")
        logger.info(f"  ROS2: {self.ros2_fps}fps, JPEG Q{self.jpeg_quality}")
        logger.info(f"  H.264: {self.h264_bitrate//1000000}Mbps (on PICO connect)")
        logger.info(f"  No v4l2loopback required")
        logger.info("=" * 60)

        if not self._open_camera():
            return False

        ros2_ok = self._init_ros2()
        self._check_adb()
        self._start_tcp_server()

        self.is_running = True

        # Start threads
        threads = [
            threading.Thread(target=self._capture_loop, daemon=True, name="capture"),
        ]
        if ros2_ok:
            threads.append(
                threading.Thread(target=self._ros2_loop, daemon=True, name="ros2"))

        for t in threads:
            t.start()

        logger.info("=" * 60)
        if self._tcp_available:
            logger.info("Mode: ROS2 + PICO (XRobo TCP active)")
        else:
            logger.info("Mode: ROS2 only (port 13579 taken)")
        logger.info("=" * 60)

        # Main loop: TCP accept or idle
        try:
            if self._tcp_available:
                self._tcp_accept_loop()
            else:
                while self.is_running:
                    time.sleep(1.0)
        except KeyboardInterrupt:
            pass

        return True

    def stop(self):
        """Clean shutdown"""
        logger.info("Shutting down unified stereo node...")
        self.is_running = False

        # Stop H.264
        self._stop_h264_streaming()

        # Stop TCP server
        if self._tcp_server:
            try:
                self._tcp_server.close()
            except Exception:
                pass

        # Release camera
        if self.cap:
            self.cap.release()
            self.cap = None

        # Shutdown ROS2
        if self._ros2_node:
            try:
                self._ros2_node.destroy_node()
            except Exception:
                pass
        try:
            import rclpy
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

        logger.info(f"Stats: capture={self._capture_count} "
                     f"ros2={self._ros2_count} h264={self._h264_count}")
        logger.info("Unified stereo node stopped")

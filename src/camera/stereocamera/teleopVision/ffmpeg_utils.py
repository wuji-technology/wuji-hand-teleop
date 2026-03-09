#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FFmpeg 编码工具函数

提供:
1. NVENC/libx264 编码器检测与参数生成
2. H.264 NAL 单元解析
3. 跨平台相机输入参数构建
4. H.264 帧读取生成器 (SPS 分帧)

被 unified_stereo_node.py 使用。

作者: Liang ZHU
"""

import sys
import subprocess
import logging

logger = logging.getLogger(__name__)


# ============== 进程管理 ==============

def terminate_ffmpeg(process: subprocess.Popen, label: str = "FFmpeg"):
    """
    安全终止 FFmpeg 进程

    步骤: SIGTERM → wait(2s) → SIGKILL → wait(1s)
    确保无论何种情况都释放相机资源。
    """
    if process is None:
        return
    try:
        process.terminate()
        try:
            process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            logger.warning(f"{label} 未响应 terminate，强制 kill...")
            process.kill()
            process.wait(timeout=1)
    except Exception as e:
        logger.warning(f"终止 {label} 时出错: {e}")
        try:
            process.kill()
        except Exception:
            pass


# ============== 流媒体常量 ==============

BUFFER_READ_SIZE = 32768        # FFmpeg stdout 读取缓冲 (32KB)
MAX_BUFFER_SIZE = 512 * 1024    # 帧缓冲区溢出阈值 (512KB)


def check_nvenc_available() -> bool:
    """
    检测 NVIDIA NVENC 硬件编码器是否真正可用

    仅检查 FFmpeg 编码器列表不够 — 还需要 libcuda.so.1 (NVIDIA 驱动)。
    Docker 容器内如果没有 --gpus all，NVENC 会列出但无法使用。
    因此实际尝试编码一帧来验证。
    """
    try:
        result = subprocess.run(
            ['ffmpeg', '-hide_banner', '-encoders'],
            capture_output=True, text=True, timeout=5
        )
        if 'h264_nvenc' not in result.stdout:
            return False

        # 实际测试 NVENC 能否编码 (检测 libcuda.so.1 是否可用)
        test = subprocess.run(
            ['ffmpeg', '-hide_banner', '-f', 'lavfi', '-i', 'nullsrc=s=64x64:d=0.1',
             '-c:v', 'h264_nvenc', '-f', 'null', '-'],
            capture_output=True, text=True, timeout=5
        )
        if test.returncode == 0:
            logger.info("检测到 NVIDIA NVENC 硬件编码器 (已验证可用)")
            return True
        else:
            logger.info("NVENC 已列出但不可用 (缺少 libcuda.so.1 或 GPU 访问)，回退到 libx264")
            return False
    except Exception as e:
        logger.debug(f"检测 NVENC 失败: {e}")
    return False


def get_encoder_args(bitrate_k: int, use_nvenc: bool) -> list:
    """
    获取 H.264 编码器参数 (低延迟优先)

    参数:
        bitrate_k: 码率 (kbps)
        use_nvenc: 是否使用 NVENC 硬件编码

    返回:
        FFmpeg 编码器参数列表
    """
    if use_nvenc:
        logger.info(f"使用 NVENC 硬件编码器 (低延迟), 码率: {bitrate_k} kbps")
        return [
            '-pix_fmt', 'yuv420p',
            '-c:v', 'h264_nvenc',
            '-preset', 'p1',
            '-tune', 'll',
            '-profile:v', 'baseline',
            '-level', '4.2',
            '-rc', 'cbr',
            '-b:v', f'{bitrate_k}k',
            '-maxrate', f'{bitrate_k}k',
            '-bufsize', f'{bitrate_k // 20}k',
            '-g', '1',
            '-keyint_min', '1',
            '-delay', '0',
            '-zerolatency', '1',
        ]
    else:
        logger.info(f"使用 libx264 软件编码器 (低延迟), 码率: {bitrate_k} kbps")
        return [
            '-pix_fmt', 'yuv420p',
            '-c:v', 'libx264',
            '-preset', 'ultrafast',
            '-tune', 'zerolatency',
            '-profile:v', 'baseline',
            '-level', '4.2',
            '-b:v', f'{bitrate_k}k',
            '-maxrate', f'{bitrate_k}k',
            '-bufsize', f'{bitrate_k // 20}k',
            '-g', '1',
            '-keyint_min', '1',
        ]


def find_nal_type(data: bytes, nal_type: int, start_pos: int = 0) -> int:
    """
    查找特定类型的 H.264 NAL 单元

    NAL 类型:
        7: SPS (Sequence Parameter Set)
        8: PPS (Picture Parameter Set)
        5: IDR slice (关键帧)
        1: P slice (预测帧)

    参数:
        data: H.264 Annexb 数据
        nal_type: 目标 NAL 类型
        start_pos: 搜索起始位置

    返回:
        NAL startcode 位置，未找到返回 -1
    """
    NAL_STARTCODE_4 = b'\x00\x00\x00\x01'
    NAL_STARTCODE_3 = b'\x00\x00\x01'

    pos = start_pos
    while pos < len(data) - 4:
        if data[pos:pos + 4] == NAL_STARTCODE_4:
            if (data[pos + 4] & 0x1F) == nal_type:
                return pos
            pos += 4
        elif data[pos:pos + 3] == NAL_STARTCODE_3:
            if (data[pos + 3] & 0x1F) == nal_type:
                return pos
            pos += 3
        else:
            pos += 1
    return -1


# ============== 相机输入参数 ==============

def build_camera_input_args(device_path: str, width: int, height: int, fps: int) -> list:
    """
    构建 FFmpeg 相机输入参数 (跨平台)

    参数:
        device_path: 设备路径 (Linux: /dev/video0, Windows: video=Name)
        width: 视频宽度
        height: 视频高度
        fps: 帧率

    返回:
        FFmpeg 输入参数列表
    """
    if sys.platform == 'win32':
        return [
            '-f', 'dshow',
            '-video_size', f'{width}x{height}',
            '-framerate', str(fps),
            '-vcodec', 'mjpeg',
            '-i', device_path,
        ]
    else:
        return [
            '-f', 'v4l2',
            '-input_format', 'mjpeg',
            '-video_size', f'{width}x{height}',
            '-framerate', str(fps),
            '-i', device_path,
        ]


# ============== H.264 帧读取生成器 ==============

def read_h264_frames(stdout, is_running=None, poll_interval=None):
    """
    从 FFmpeg stdout 读取 H.264 帧的生成器

    按 SPS NAL (type 7) 分割帧，适用于 GOP=1 模式。
    封装了缓冲区管理、NAL 解析和溢出保护。

    参数:
        stdout: FFmpeg 进程的 stdout 管道
        is_running: 可选的状态检查函数，返回 False 时停止。
                    如果为 None，仅在 EOF 时停止。
        poll_interval: 如果设置，使用 select() 进行非阻塞读取 (秒)。
                       适用于需要快速响应停止信号的场景。
                       如果为 None，使用阻塞读取 (需要外部终止 FFmpeg 来退出)。

    Yields:
        bytes: 完整的 H.264 帧数据 (从一个 SPS 到下一个 SPS)
    """
    buffer = b''
    first_sps_found = False

    while is_running is None or is_running():
        # 非阻塞读取模式 (用于需要快速响应停止信号的场景)
        if poll_interval is not None:
            import select
            readable, _, _ = select.select([stdout], [], [], poll_interval)
            if not readable:
                continue

        chunk = stdout.read(BUFFER_READ_SIZE)
        if not chunk:
            break

        buffer += chunk

        # 按 SPS NAL 分割帧
        while len(buffer) > 5:
            if not first_sps_found:
                sps_idx = find_nal_type(buffer, 7, 0)
                if sps_idx < 0:
                    break
                if sps_idx > 0:
                    buffer = buffer[sps_idx:]
                first_sps_found = True
                continue

            next_sps_idx = find_nal_type(buffer, 7, 5)
            if next_sps_idx > 0:
                yield buffer[:next_sps_idx]
                buffer = buffer[next_sps_idx:]
            else:
                # 缓冲区溢出保护
                if len(buffer) > MAX_BUFFER_SIZE:
                    logger.warning("H.264 缓冲区溢出，重置帧同步")
                    buffer = b''
                    first_sps_found = False
                break

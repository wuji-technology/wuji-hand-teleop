#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FFmpeg encoding utility functions

Provides:
1. NVENC/libx264 encoder detection and parameter generation
2. H.264 NAL unit parsing
3. Cross-platform camera input parameter building
4. H.264 frame reader generator (SPS-based frame splitting)

Used by unified_stereo_node.py.

Author: Liang ZHU
"""

import sys
import subprocess
import logging

logger = logging.getLogger(__name__)


# ============== Process Management ==============

def terminate_ffmpeg(process: subprocess.Popen, label: str = "FFmpeg"):
    """
    Safely terminate FFmpeg process

    Steps: SIGTERM → wait(2s) → SIGKILL → wait(1s)
    Ensures camera resources are released in all cases.
    """
    if process is None:
        return
    try:
        process.terminate()
        try:
            process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            logger.warning(f"{label} did not respond to terminate, force killing...")
            process.kill()
            process.wait(timeout=1)
    except Exception as e:
        logger.warning(f"Error terminating {label}: {e}")
        try:
            process.kill()
        except Exception:
            pass


# ============== Streaming Constants ==============

BUFFER_READ_SIZE = 32768        # FFmpeg stdout read buffer (32KB)
MAX_BUFFER_SIZE = 512 * 1024    # Frame buffer overflow threshold (512KB)


def check_nvenc_available() -> bool:
    """
    Detect whether NVIDIA NVENC hardware encoder is actually available

    Checking the FFmpeg encoder list alone is not enough — libcuda.so.1 (NVIDIA driver) is also required.
    Inside a Docker container without --gpus all, NVENC will be listed but unusable.
    Therefore, actually try encoding a frame to verify.
    """
    try:
        result = subprocess.run(
            ['ffmpeg', '-hide_banner', '-encoders'],
            capture_output=True, text=True, timeout=5
        )
        if 'h264_nvenc' not in result.stdout:
            return False

        # Actually test if NVENC can encode (check if libcuda.so.1 is available)
        test = subprocess.run(
            ['ffmpeg', '-hide_banner', '-f', 'lavfi', '-i', 'nullsrc=s=64x64:d=0.1',
             '-c:v', 'h264_nvenc', '-f', 'null', '-'],
            capture_output=True, text=True, timeout=5
        )
        if test.returncode == 0:
            logger.info("NVIDIA NVENC hardware encoder detected (verified available)")
            return True
        else:
            logger.info("NVENC listed but not available (missing libcuda.so.1 or GPU access), falling back to libx264")
            return False
    except Exception as e:
        logger.debug(f"NVENC detection failed: {e}")
    return False


def get_encoder_args(bitrate_k: int, use_nvenc: bool) -> list:
    """
    Get H.264 encoder arguments (low latency priority)

    Args:
        bitrate_k: Bitrate (kbps)
        use_nvenc: Whether to use NVENC hardware encoding

    Returns:
        FFmpeg encoder argument list
    """
    if use_nvenc:
        logger.info(f"Using NVENC hardware encoder (low latency), bitrate: {bitrate_k} kbps")
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
        logger.info(f"Using libx264 software encoder (low latency), bitrate: {bitrate_k} kbps")
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
    Find a specific type of H.264 NAL unit

    NAL types:
        7: SPS (Sequence Parameter Set)
        8: PPS (Picture Parameter Set)
        5: IDR slice (keyframe)
        1: P slice (predicted frame)

    Args:
        data: H.264 Annexb data
        nal_type: Target NAL type
        start_pos: Search start position

    Returns:
        NAL startcode position, returns -1 if not found
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


# ============== Camera Input Arguments ==============

def build_camera_input_args(device_path: str, width: int, height: int, fps: int) -> list:
    """
    Build FFmpeg camera input arguments (cross-platform)

    Args:
        device_path: Device path (Linux: /dev/video0, Windows: video=Name)
        width: Video width
        height: Video height
        fps: Frame rate

    Returns:
        FFmpeg input argument list
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


# ============== H.264 Frame Reader Generator ==============

def read_h264_frames(stdout, is_running=None, poll_interval=None):
    """
    Generator that reads H.264 frames from FFmpeg stdout

    Splits frames by SPS NAL (type 7), suitable for GOP=1 mode.
    Encapsulates buffer management, NAL parsing, and overflow protection.

    Args:
        stdout: FFmpeg process stdout pipe
        is_running: Optional status check function, stops when returning False.
                    If None, only stops on EOF.
        poll_interval: If set, uses select() for non-blocking reads (seconds).
                       Suitable for scenarios requiring fast response to stop signals.
                       If None, uses blocking reads (requires external FFmpeg termination to exit).

    Yields:
        bytes: Complete H.264 frame data (from one SPS to the next SPS)
    """
    buffer = b''
    first_sps_found = False

    while is_running is None or is_running():
        # Non-blocking read mode (for scenarios requiring fast response to stop signals)
        if poll_interval is not None:
            import select
            readable, _, _ = select.select([stdout], [], [], poll_interval)
            if not readable:
                continue

        chunk = stdout.read(BUFFER_READ_SIZE)
        if not chunk:
            break

        buffer += chunk

        # Split frames by SPS NAL
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
                # Buffer overflow protection
                if len(buffer) > MAX_BUFFER_SIZE:
                    logger.warning("H.264 buffer overflow, resetting frame sync")
                    buffer = b''
                    first_sps_found = False
                break

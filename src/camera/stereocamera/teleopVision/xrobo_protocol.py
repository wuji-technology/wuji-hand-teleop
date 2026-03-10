#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
XRoboToolkit 协议解析模块 (最小依赖, 无遗留代码)

从 xrobo_compat_server.py 提取的纯协议解析逻辑:
- ProtocolConstants: XRobo 协议常量 (端口/包头/命令码)
- PacketParser: 协议包解析 + 构建 (支持 NetworkDataProtocol / XRobo / JSON)

使用方:
- unified_stereo_node.py (活跃): PICO H.264 单进程架构

作者: Liang ZHU
"""

import json
import struct
import time
import traceback
import logging

logger = logging.getLogger(__name__)


class ProtocolConstants:
    """协议常量 (参考 XRoboToolkit-Unity-Client 的 TcpManager.cs)"""

    # 端口 (Unity Client 使用 13579)
    TCP_PORT = 13579

    # 包头包尾
    PACKET_HEAD_SEND = 0x3F      # 发送包头
    PACKET_HEAD_RECV = 0xCF      # 接收包头
    PACKET_END = 0xA5            # 包尾

    # 命令码
    CMD_CONNECT = 0x19           # 连接
    CMD_FUNCTION = 0x6D          # 函数调用
    CMD_HEARTBEAT = 0x23         # 心跳
    CMD_VERSION = 0x6C           # 版本


class PacketParser:
    """
    协议包解析器
    支持两种格式:
    1. XRoboToolkit 格式: [Head:1][Cmd:1][Length:4][Data:n][Timestamp:8][End:1]
    2. NetworkDataProtocol 格式: [cmd_len:4][command:str][data_len:4][data:bytes]
    """

    @staticmethod
    def parse(data: bytes) -> tuple:
        """
        解析数据包
        返回: (command, json_data) 或 (None, None)
        """
        if len(data) < 8:
            return None, None

        # 首先尝试 NetworkDataProtocol 格式 (Unity Client 使用)
        result = PacketParser._try_parse_network_data_protocol(data)
        if result[0] is not None:
            return result

        # 然后尝试 XRoboToolkit 格式
        result = PacketParser._try_parse_xrobo_protocol(data)
        if result[0] is not None:
            return result

        # 最后尝试直接解析 JSON
        return PacketParser._try_parse_json(data)

    @staticmethod
    def _try_parse_network_data_protocol(data: bytes) -> tuple:
        """
        解析 NetworkDataProtocol 格式

        Unity Client 发送格式:
        [total_length: 4 bytes Big-Endian][cmd_len: 4 bytes Little-Endian][command][data_len: 4 bytes Little-Endian][data]
        """
        try:
            hex_preview = data[:min(64, len(data))].hex()
            logger.debug(f"[DEBUG] 收到数据 ({len(data)} bytes): {hex_preview}...")

            if len(data) < 12:
                return None, None

            offset = 0

            # 检查是否有 Big-Endian 长度前缀
            potential_total_len = struct.unpack('>I', data[0:4])[0]
            if 10 < potential_total_len < 1000 and potential_total_len <= len(data):
                offset = 4
                logger.debug(f"[DEBUG] 检测到长度前缀: {potential_total_len} bytes")

            # 读取命令长度 (Little-Endian)
            cmd_len = struct.unpack('<I', data[offset:offset+4])[0]
            offset += 4

            # 验证命令长度合理性
            if cmd_len <= 0 or cmd_len > 100:
                return None, None

            if offset + cmd_len > len(data):
                return None, None

            # 读取命令字符串
            command = data[offset:offset+cmd_len].decode('utf-8')
            offset += cmd_len

            if offset + 4 > len(data):
                return None, None

            # 读取数据长度
            data_len = struct.unpack('<I', data[offset:offset+4])[0]
            offset += 4

            if data_len < 0 or offset + data_len > len(data):
                return None, None

            # 读取数据
            payload = data[offset:offset+data_len]

            logger.info(f"解析命令: {command}, 数据长度: {data_len}")

            # 根据命令类型处理
            if command == "OPEN_CAMERA":
                camera_config = PacketParser._parse_camera_request(payload)
                if camera_config:
                    return ProtocolConstants.CMD_FUNCTION, {
                        "functionName": "OpenCamera",
                        "value": camera_config
                    }

            elif command == "CLOSE_CAMERA":
                return ProtocolConstants.CMD_FUNCTION, {
                    "functionName": "StopReceivePcCamera"
                }

            elif command == "MEDIA_DECODER_READY":
                return ProtocolConstants.CMD_FUNCTION, {
                    "functionName": "MediaDecoderReady",
                    "value": payload
                }

            return None, None

        except Exception as e:
            logger.debug(f"NetworkDataProtocol 解析失败: {e}")
            return None, None

    @staticmethod
    def _parse_camera_request(data: bytes) -> dict:
        """解析 CameraRequest 二进制数据"""
        try:
            if len(data) < 10:
                logger.warning(f"CameraRequest 数据太短: {len(data)} bytes")
                return None

            offset = 0

            # 检查魔数 0xCA 0xFE
            if data[0] == 0xCA and data[1] == 0xFE:
                offset = 2
                version = data[offset]
                offset += 1
                logger.debug(f"CameraRequest 协议版本: {version}")
            else:
                logger.debug("无魔数头，尝试直接解析")

            # 读取整数字段 (小端序)
            width = struct.unpack('<I', data[offset:offset+4])[0]
            offset += 4

            height = struct.unpack('<I', data[offset:offset+4])[0]
            offset += 4

            fps = struct.unpack('<I', data[offset:offset+4])[0]
            offset += 4

            bitrate = struct.unpack('<I', data[offset:offset+4])[0]
            offset += 4

            enable_hevc = struct.unpack('<I', data[offset:offset+4])[0]
            offset += 4

            render_mode = struct.unpack('<I', data[offset:offset+4])[0]
            offset += 4

            port = struct.unpack('<I', data[offset:offset+4])[0]
            offset += 4

            # 读取 camera 字符串
            camera_len = data[offset]
            offset += 1
            camera = data[offset:offset+camera_len].decode('utf-8') if camera_len > 0 else "USB"
            offset += camera_len

            # 读取 ip 字符串
            ip_len = data[offset]
            offset += 1
            ip = data[offset:offset+ip_len].decode('utf-8') if ip_len > 0 else ""
            offset += ip_len

            logger.info(f"CameraRequest: {width}x{height}@{fps}fps, {bitrate//1000000}Mbps, ip={ip}:{port}")

            return {
                "width": width,
                "height": height,
                "fps": fps,
                "bitrate": bitrate,
                "ip": ip,
                "port": port,
                "cameraType": camera
            }

        except Exception as e:
            logger.error(f"解析 CameraRequest 失败: {e}")
            traceback.print_exc()
            return None

    @staticmethod
    def _try_parse_xrobo_protocol(data: bytes) -> tuple:
        """尝试解析 XRoboToolkit 标准协议格式"""
        head_idx = -1
        for i in range(len(data)):
            if data[i] == ProtocolConstants.PACKET_HEAD_RECV or data[i] == ProtocolConstants.PACKET_HEAD_SEND:
                head_idx = i
                break

        if head_idx < 0:
            return None, None

        try:
            cmd = data[head_idx + 1]
            length = struct.unpack('<I', data[head_idx + 2:head_idx + 6])[0]
            data_start = head_idx + 6
            data_end = data_start + length

            if data_end > len(data):
                return None, None

            payload = data[data_start:data_end]

            try:
                json_str = payload.decode('utf-8')
                json_data = json.loads(json_str)
                return cmd, json_data
            except Exception:
                pass

        except Exception as e:
            logger.debug(f"XRobo 协议解析失败: {e}")

        return None, None

    @staticmethod
    def _try_parse_json(data: bytes) -> tuple:
        """尝试直接从数据中提取 JSON"""
        try:
            text = data.decode('utf-8', errors='ignore')
            json_start = text.find('{')
            json_end = text.rfind('}')

            if json_start >= 0 and json_end > json_start:
                json_str = text[json_start:json_end + 1]
                json_data = json.loads(json_str)
                return ProtocolConstants.CMD_FUNCTION, json_data

        except Exception as e:
            logger.debug(f"JSON 解析失败: {e}")

        return None, None

    @staticmethod
    def build_response(cmd: int, data: dict) -> bytes:
        """构建响应包"""
        json_str = json.dumps(data)
        json_bytes = json_str.encode('utf-8')

        packet = bytearray()
        packet.append(ProtocolConstants.PACKET_HEAD_SEND)
        packet.append(cmd)
        packet.extend(struct.pack('<I', len(json_bytes)))
        packet.extend(json_bytes)
        packet.extend(struct.pack('<Q', int(time.time() * 1000)))
        packet.append(ProtocolConstants.PACKET_END)

        return bytes(packet)

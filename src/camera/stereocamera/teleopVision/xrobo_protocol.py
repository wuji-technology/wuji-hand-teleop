#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
XRoboToolkit protocol parsing module (minimal dependencies, no legacy code)

Pure protocol parsing logic extracted from xrobo_compat_server.py:
- ProtocolConstants: XRobo protocol constants (ports/headers/command codes)
- PacketParser: Protocol packet parsing + building (supports NetworkDataProtocol / XRobo / JSON)

Used by:
- unified_stereo_node.py (active): PICO H.264 single-process architecture

Author: Liang ZHU
"""

import json
import struct
import time
import traceback
import logging

logger = logging.getLogger(__name__)


class ProtocolConstants:
    """Protocol constants (reference: XRoboToolkit-Unity-Client TcpManager.cs)"""

    # Port (Unity Client uses 13579)
    TCP_PORT = 13579

    # Packet header and footer
    PACKET_HEAD_SEND = 0x3F      # Send packet header
    PACKET_HEAD_RECV = 0xCF      # Receive packet header
    PACKET_END = 0xA5            # Packet footer

    # Command codes
    CMD_CONNECT = 0x19           # Connect
    CMD_FUNCTION = 0x6D          # Function call
    CMD_HEARTBEAT = 0x23         # Heartbeat
    CMD_VERSION = 0x6C           # Version


class PacketParser:
    """
    Protocol packet parser
    Supports two formats:
    1. XRoboToolkit format: [Head:1][Cmd:1][Length:4][Data:n][Timestamp:8][End:1]
    2. NetworkDataProtocol format: [cmd_len:4][command:str][data_len:4][data:bytes]
    """

    @staticmethod
    def parse(data: bytes) -> tuple:
        """
        Parse data packet
        Returns: (command, json_data) or (None, None)
        """
        if len(data) < 8:
            return None, None

        # First try NetworkDataProtocol format (used by Unity Client)
        result = PacketParser._try_parse_network_data_protocol(data)
        if result[0] is not None:
            return result

        # Then try XRoboToolkit format
        result = PacketParser._try_parse_xrobo_protocol(data)
        if result[0] is not None:
            return result

        # Finally try direct JSON parsing
        return PacketParser._try_parse_json(data)

    @staticmethod
    def _try_parse_network_data_protocol(data: bytes) -> tuple:
        """
        Parse NetworkDataProtocol format

        Unity Client send format:
        [total_length: 4 bytes Big-Endian][cmd_len: 4 bytes Little-Endian][command][data_len: 4 bytes Little-Endian][data]
        """
        try:
            hex_preview = data[:min(64, len(data))].hex()
            logger.debug(f"[DEBUG] Received data ({len(data)} bytes): {hex_preview}...")

            if len(data) < 12:
                return None, None

            offset = 0

            # Check for Big-Endian length prefix
            potential_total_len = struct.unpack('>I', data[0:4])[0]
            if 10 < potential_total_len < 1000 and potential_total_len <= len(data):
                offset = 4
                logger.debug(f"[DEBUG] Length prefix detected: {potential_total_len} bytes")

            # Read command length (Little-Endian)
            cmd_len = struct.unpack('<I', data[offset:offset+4])[0]
            offset += 4

            # Validate command length
            if cmd_len <= 0 or cmd_len > 100:
                return None, None

            if offset + cmd_len > len(data):
                return None, None

            # Read command string
            command = data[offset:offset+cmd_len].decode('utf-8')
            offset += cmd_len

            if offset + 4 > len(data):
                return None, None

            # Read data length
            data_len = struct.unpack('<I', data[offset:offset+4])[0]
            offset += 4

            if data_len < 0 or offset + data_len > len(data):
                return None, None

            # Read data
            payload = data[offset:offset+data_len]

            logger.info(f"Parsed command: {command}, data length: {data_len}")

            # Handle by command type
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
            logger.debug(f"NetworkDataProtocol parsing failed: {e}")
            return None, None

    @staticmethod
    def _parse_camera_request(data: bytes) -> dict:
        """Parse CameraRequest binary data"""
        try:
            if len(data) < 10:
                logger.warning(f"CameraRequest data too short: {len(data)} bytes")
                return None

            offset = 0

            # Check magic number 0xCA 0xFE
            if data[0] == 0xCA and data[1] == 0xFE:
                offset = 2
                version = data[offset]
                offset += 1
                logger.debug(f"CameraRequest protocol version: {version}")
            else:
                logger.debug("No magic header, trying direct parsing")

            # Read integer fields (little-endian)
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

            # Read camera string
            camera_len = data[offset]
            offset += 1
            camera = data[offset:offset+camera_len].decode('utf-8') if camera_len > 0 else "USB"
            offset += camera_len

            # Read ip string
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
            logger.error(f"Failed to parse CameraRequest: {e}")
            traceback.print_exc()
            return None

    @staticmethod
    def _try_parse_xrobo_protocol(data: bytes) -> tuple:
        """Try to parse XRoboToolkit standard protocol format"""
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
            logger.debug(f"XRobo protocol parsing failed: {e}")

        return None, None

    @staticmethod
    def _try_parse_json(data: bytes) -> tuple:
        """Try to extract JSON directly from data"""
        try:
            text = data.decode('utf-8', errors='ignore')
            json_start = text.find('{')
            json_end = text.rfind('}')

            if json_start >= 0 and json_end > json_start:
                json_str = text[json_start:json_end + 1]
                json_data = json.loads(json_str)
                return ProtocolConstants.CMD_FUNCTION, json_data

        except Exception as e:
            logger.debug(f"JSON parsing failed: {e}")

        return None, None

    @staticmethod
    def build_response(cmd: int, data: dict) -> bytes:
        """Build response packet"""
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

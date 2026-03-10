"""
USB device scanner for Wuji Hand and Manus Glove devices.
"""

from __future__ import annotations

import subprocess
from pathlib import Path
from typing import Optional, Set

from .logger import get_logger

# Setup module logger
logger = get_logger("teleop_monitor.scanner")

try:
    import yaml
except ImportError:
    yaml = None
    logger.warning("yaml 模块未安装，配置文件读取功能将不可用")

try:
    import openvr
    OPENVR_AVAILABLE = True
    logger.debug("OpenVR 模块已加载")
except ImportError:
    openvr = None
    OPENVR_AVAILABLE = False
    logger.debug("OpenVR 模块不可用")


class WujiHandScanner:
    """Scan for Wuji Hand devices via USB."""

    USB_VID = "0483"
    USB_PID = "2000"

    @classmethod
    def scan_devices(cls) -> Set[str]:
        """
        Scan for connected Wuji Hand devices and return their serial numbers.

        Returns:
            Set of serial numbers found
        """
        try:
            logger.debug(f"扫描 Wuji 灵巧手设备 (VID:PID = {cls.USB_VID}:{cls.USB_PID})")
            result = subprocess.run(
                ["lsusb", "-v", "-d", f"{cls.USB_VID}:{cls.USB_PID}"],
                capture_output=True,
                text=True,
                timeout=5
            )

            serials = set()
            for line in result.stdout.split('\n'):
                if 'iSerial' in line:
                    parts = line.strip().split()
                    if len(parts) >= 3:
                        serial = parts[-1]
                        if serial and serial != '0':
                            serials.add(serial)

            logger.debug(f"找到 {len(serials)} 个 Wuji 灵巧手设备: {serials}")
            return serials

        except subprocess.TimeoutExpired:
            logger.error("扫描 Wuji 灵巧手设备超时 (5秒)")
            return set()
        except FileNotFoundError:
            logger.error("lsusb 命令不存在，请安装 usbutils 包")
            return set()
        except Exception as e:
            logger.error(f"扫描 Wuji 灵巧手设备异常: {e}", exc_info=True)
            return set()


class ManusGloveScanner:
    """Scan for Manus Glove devices via USB."""

    USB_VID = "3325"
    USB_PID = "00ca"

    @classmethod
    def scan_devices(cls) -> Set[str]:
        """
        Scan for connected Manus Glove devices and return their serial numbers.

        Returns:
            Set of serial numbers found
        """
        try:
            logger.debug(f"扫描 Manus 手套设备 (VID:PID = {cls.USB_VID}:{cls.USB_PID})")
            result = subprocess.run(
                ["lsusb", "-v", "-d", f"{cls.USB_VID}:{cls.USB_PID}"],
                capture_output=True,
                text=True,
                timeout=5
            )

            serials = set()
            for line in result.stdout.split('\n'):
                if 'iSerial' in line:
                    parts = line.strip().split()
                    if len(parts) >= 3:
                        serial = parts[-1]
                        if serial and serial != '0':
                            serials.add(serial)

            logger.debug(f"找到 {len(serials)} 个 Manus 手套设备: {serials}")
            return serials

        except subprocess.TimeoutExpired:
            logger.error("扫描 Manus 手套设备超时 (5秒)")
            return set()
        except FileNotFoundError:
            logger.error("lsusb 命令不存在，请安装 usbutils 包")
            return set()
        except Exception as e:
            logger.error(f"扫描 Manus 手套设备异常: {e}", exc_info=True)
            return set()


class TianjiArmScanner:
    """Scan for Tianji Arm network connectivity."""

    DEFAULT_IP = "192.168.1.190"
    CONFIG_FILENAME = "tianji_output.yaml"

    _cached_ip: Optional[str] = None

    @classmethod
    def get_robot_ip(cls) -> str:
        """
        Get robot IP from tianji_output config file.

        Returns:
            IP address string
        """
        if cls._cached_ip is not None:
            return cls._cached_ip

        # Try to find config file using ament_index
        try:
            from ament_index_python.packages import get_package_share_directory
            pkg_share = Path(get_package_share_directory('tianji_output'))
            search_paths = [
                pkg_share / "config" / cls.CONFIG_FILENAME,
            ]
            logger.debug(f"使用 ament_index 查找配置文件: {search_paths}")
        except Exception as e:
            logger.debug(f"ament_index 不可用: {e}")
            # Fallback to install directory if ament_index fails
            search_paths = [
                Path.home() / "ros2_ws/install/tianji_output/share/tianji_output/config" / cls.CONFIG_FILENAME,
            ]
            logger.debug(f"使用后备路径查找配置文件: {search_paths}")

        for config_path in search_paths:
            if config_path.exists() and yaml is not None:
                try:
                    logger.debug(f"读取配置文件: {config_path}")
                    with open(config_path, 'r', encoding='utf-8') as f:
                        config = yaml.safe_load(f)
                    if config and 'robot_ip' in config:
                        cls._cached_ip = config['robot_ip']
                        logger.info(f"从配置文件读取到机器人 IP: {cls._cached_ip}")
                        return cls._cached_ip
                except Exception as e:
                    logger.error(f"读取配置文件失败: {e}", exc_info=True)

        cls._cached_ip = cls.DEFAULT_IP
        logger.info(f"使用默认机器人 IP: {cls._cached_ip}")
        return cls._cached_ip

    @classmethod
    def check_reachable(cls) -> bool:
        """
        Check if Tianji Arm is reachable via ping.

        Returns:
            True if reachable, False otherwise
        """
        ip = cls.get_robot_ip()
        try:
            logger.debug(f"检查机器人网络连接: {ip}")
            result = subprocess.run(
                ["ping", "-c", "1", "-W", "1", ip],
                capture_output=True,
                timeout=2
            )
            reachable = result.returncode == 0
            if reachable:
                logger.debug(f"机器人 {ip} 可达")
            else:
                logger.debug(f"机器人 {ip} 不可达")
            return reachable
        except subprocess.TimeoutExpired:
            logger.error(f"Ping 机器人 {ip} 超时 (2秒)")
            return False
        except Exception as e:
            logger.error(f"检查机器人连接异常: {e}", exc_info=True)
            return False


class StereoHeadScanner:
    """Scan for stereo head camera and v4l2loopback status."""

    STEREO_CAMERA_DEVICE = "/dev/stereo_camera"  # udev 符号链接
    LOOPBACK_DEVICE = "/dev/video99"             # v4l2loopback 虚拟设备

    @classmethod
    def _get_camera_package_path(cls) -> Optional[Path]:
        """
        Dynamically get camera package path using ament_index.

        Returns:
            Path to camera package, or None if not found
        """
        try:
            from ament_index_python.packages import get_package_share_directory
            return Path(get_package_share_directory('camera'))
        except Exception:
            return None

    @classmethod
    def check_stereo_camera(cls) -> dict:
        """
        Check if stereo camera is available.

        Returns:
            Dict with camera info: {'available': bool, 'device': str, 'info': str}
        """
        import os

        result = {'available': False, 'device': '', 'info': ''}
        logger.debug("检查头部双目相机")

        # 检查 udev 符号链接
        if os.path.exists(cls.STEREO_CAMERA_DEVICE):
            try:
                real_path = os.path.realpath(cls.STEREO_CAMERA_DEVICE)
                result['available'] = True
                result['device'] = real_path
                result['info'] = f"→ {real_path}"
                logger.debug(f"找到双目相机: {cls.STEREO_CAMERA_DEVICE} -> {real_path}")
                return result
            except Exception as e:
                logger.error(f"读取相机设备链接失败: {e}", exc_info=True)

        # 回退: 检查常见的 video 设备
        logger.debug("检查常见的 video 设备")
        for i in [0, 1, 2, 3, 4]:
            device = f"/dev/video{i}"
            if os.path.exists(device):
                # 尝试用 v4l2-ctl 检查是否是双目相机
                try:
                    check_result = subprocess.run(
                        ["v4l2-ctl", "-d", device, "--all"],
                        capture_output=True,
                        text=True,
                        timeout=2
                    )
                    output = check_result.stdout.lower()
                    # 检查是否是双目相机 (通常分辨率为 2560x720 或类似)
                    if "2560" in output or "stereo" in output or "dual" in output:
                        result['available'] = True
                        result['device'] = device
                        result['info'] = "检测到双目相机"
                        logger.info(f"在 {device} 发现双目相机")
                        return result
                except subprocess.TimeoutExpired:
                    logger.warning(f"检查设备 {device} 超时")
                except FileNotFoundError:
                    logger.error("v4l2-ctl 命令不存在，请安装 v4l-utils 包")
                    break
                except Exception as e:
                    logger.debug(f"检查设备 {device} 失败: {e}")

        result['info'] = "未找到双目相机"
        logger.warning("未找到头部双目相机")
        return result

    @classmethod
    def check_loopback_device(cls) -> dict:
        """
        Check if v4l2loopback device is available.

        Returns:
            Dict with loopback info: {'available': bool, 'loaded': bool, 'info': str}
        """
        import os

        result = {'available': False, 'loaded': False, 'info': ''}
        logger.debug("检查 v4l2loopback 设备")

        # 检查 v4l2loopback 模块是否已加载
        try:
            lsmod_result = subprocess.run(
                ["lsmod"],
                capture_output=True,
                text=True,
                timeout=2
            )
            if "v4l2loopback" in lsmod_result.stdout:
                result['loaded'] = True
                logger.debug("v4l2loopback 模块已加载")
            else:
                logger.debug("v4l2loopback 模块未加载")
        except Exception as e:
            logger.error(f"检查 lsmod 失败: {e}", exc_info=True)

        # 检查 /dev/video99 是否存在
        if os.path.exists(cls.LOOPBACK_DEVICE):
            result['available'] = True
            result['info'] = "v4l2loopback 已就绪"
            logger.debug(f"Loopback 设备 {cls.LOOPBACK_DEVICE} 已就绪")
        elif result['loaded']:
            result['info'] = "模块已加载，设备未创建"
            logger.warning("v4l2loopback 模块已加载，但设备未创建")
        else:
            result['info'] = "需要加载 v4l2loopback"
            logger.warning("v4l2loopback 未加载")

        return result

    @classmethod
    def get_setup_command(cls) -> str:
        """
        Get the command to setup v4l2loopback.

        Returns:
            Shell command string
        """
        return 'sudo modprobe v4l2loopback devices=1 video_nr=99 card_label="StereoVR_ROS2"'

    @classmethod
    def check_camera_package(cls) -> bool:
        """
        Check if camera package is installed.

        Returns:
            True if camera package is available
        """
        camera_path = cls._get_camera_package_path()
        return camera_path is not None and camera_path.exists()


class ViveTrackerScanner:
    """Scan for SteamVR/Vive Tracker status."""

    OPENVR_PATH_REGISTRY = Path.home() / ".config/openvr/openvrpaths.vrpath"

    BODY_PART_NAMES = {
        'chest': '胸部',
        'right_wrist': '右腕',
        'left_wrist': '左腕',
        'left_arm': '左臂',
        'right_arm': '右臂',
    }

    @classmethod
    def _get_config_path(cls) -> Optional[Path]:
        """
        Dynamically get OpenVR input config path using ament_index.

        Returns:
            Path to openvr_input config, or None if not found
        """
        try:
            from ament_index_python.packages import get_package_share_directory
            pkg_share = Path(get_package_share_directory('openvr_input'))
            config_path = pkg_share / "config" / "openvr_input.yaml"
            if config_path.exists():
                return config_path
        except Exception:
            pass

        # Fallback to install directory
        fallback = Path.home() / "ros2_ws/install/openvr_input/share/openvr_input/config/openvr_input.yaml"
        if fallback.exists():
            return fallback

        return None

    @classmethod
    def _is_openvr_configured(cls) -> bool:
        """Check if OpenVR is properly configured on the system."""
        return cls.OPENVR_PATH_REGISTRY.exists()

    @classmethod
    def get_tracker_config(cls) -> dict:
        """读取配置文件中的 Tracker 部位-序列号映射（每次重新读取）"""
        config_path = cls._get_config_path()
        if config_path and yaml is not None:
            try:
                with open(config_path, 'r', encoding='utf-8') as f:
                    config = yaml.safe_load(f)
                if config and 'tracker_serials' in config:
                    return config['tracker_serials']
            except Exception:
                pass
        return {}

    @classmethod
    def scan_trackers(cls) -> Set[str]:
        """
        Get connected Vive Tracker serial numbers via OpenVR API.

        Returns:
            Set of tracker serial numbers (only currently connected devices)
        """
        if not OPENVR_AVAILABLE or openvr is None:
            logger.debug("OpenVR 不可用，跳过 Tracker 扫描")
            return set()

        # Check if OpenVR is configured before attempting to initialize
        if not cls._is_openvr_configured():
            logger.debug("OpenVR 未配置，跳过 Tracker 扫描")
            return set()

        try:
            logger.debug("初始化 OpenVR API")
            openvr.init(openvr.VRApplication_Other)
            vr = openvr.VRSystem()

            serials = set()
            logger.debug("扫描 Vive Tracker 设备")
            for i in range(64):
                device_class = vr.getTrackedDeviceClass(i)
                if device_class == openvr.TrackedDeviceClass_GenericTracker:
                    # 检查设备是否真正连接（排除缓存的离线设备）
                    if not vr.isTrackedDeviceConnected(i):
                        continue
                    serial = vr.getStringTrackedDeviceProperty(
                        i, openvr.Prop_SerialNumber_String
                    )
                    if serial:
                        serials.add(serial)

            logger.debug(f"关闭 OpenVR API，找到 {len(serials)} 个 Tracker")
            openvr.shutdown()
            return serials
        except Exception as e:
            logger.error(f"扫描 Vive Tracker 异常: {e}", exc_info=True)
            try:
                openvr.shutdown()
            except Exception:
                pass
            return set()

    @classmethod
    def scan_base_stations(cls) -> dict:
        """
        Get connected base station status via OpenVR API.

        Returns:
            Dict with base station info: {serial: {'mode': 'A/B/C', 'connected': True/False}}
        """
        if not OPENVR_AVAILABLE or openvr is None:
            logger.debug("OpenVR 不可用，跳过基站扫描")
            return {}

        # Check if OpenVR is configured before attempting to initialize
        if not cls._is_openvr_configured():
            logger.debug("OpenVR 未配置，跳过基站扫描")
            return {}

        try:
            logger.debug("初始化 OpenVR API (基站扫描)")
            openvr.init(openvr.VRApplication_Other)
            vr = openvr.VRSystem()

            stations = {}
            logger.debug("扫描 Vive 基站设备")
            for i in range(64):
                device_class = vr.getTrackedDeviceClass(i)
                if device_class == openvr.TrackedDeviceClass_TrackingReference:
                    serial = vr.getStringTrackedDeviceProperty(
                        i, openvr.Prop_SerialNumber_String
                    )
                    connected = vr.isTrackedDeviceConnected(i)
                    # 尝试获取基站模式 (A/B/C)
                    try:
                        mode = vr.getStringTrackedDeviceProperty(
                            i, openvr.Prop_ModeLabel_String
                        )
                    except Exception:
                        mode = "?"
                    if serial:
                        stations[serial] = {'mode': mode or "?", 'connected': connected}

            logger.debug(f"关闭 OpenVR API，找到 {len(stations)} 个基站")
            openvr.shutdown()
            return stations
        except Exception as e:
            logger.error(f"扫描 Vive 基站异常: {e}", exc_info=True)
            try:
                openvr.shutdown()
            except Exception:
                pass
            return {}

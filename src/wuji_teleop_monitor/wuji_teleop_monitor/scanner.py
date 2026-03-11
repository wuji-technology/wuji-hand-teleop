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
    logger.warning("yaml module not installed, config file loading will be unavailable")

try:
    import openvr
    OPENVR_AVAILABLE = True
    logger.debug("OpenVR module loaded")
except ImportError:
    openvr = None
    OPENVR_AVAILABLE = False
    logger.debug("OpenVR module not available")


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
            logger.debug(f"Scanning Wuji Hand devices (VID:PID = {cls.USB_VID}:{cls.USB_PID})")
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

            logger.debug(f"Found {len(serials)} Wuji Hand device(s): {serials}")
            return serials

        except subprocess.TimeoutExpired:
            logger.error("Wuji Hand device scan timed out (5s)")
            return set()
        except FileNotFoundError:
            logger.error("lsusb command not found, please install usbutils")
            return set()
        except Exception as e:
            logger.error(f"Wuji Hand device scan error: {e}", exc_info=True)
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
            logger.debug(f"Scanning Manus Glove devices (VID:PID = {cls.USB_VID}:{cls.USB_PID})")
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

            logger.debug(f"Found {len(serials)} Manus Glove device(s): {serials}")
            return serials

        except subprocess.TimeoutExpired:
            logger.error("Manus Glove device scan timed out (5s)")
            return set()
        except FileNotFoundError:
            logger.error("lsusb command not found, please install usbutils")
            return set()
        except Exception as e:
            logger.error(f"Manus Glove device scan error: {e}", exc_info=True)
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
            logger.debug(f"Using ament_index to find config file: {search_paths}")
        except Exception as e:
            logger.debug(f"ament_index not available: {e}")
            # Fallback to install directory if ament_index fails
            search_paths = [
                Path.home() / "ros2_ws/install/tianji_output/share/tianji_output/config" / cls.CONFIG_FILENAME,
            ]
            logger.debug(f"Using fallback path to find config file: {search_paths}")

        for config_path in search_paths:
            if config_path.exists() and yaml is not None:
                try:
                    logger.debug(f"Reading config file: {config_path}")
                    with open(config_path, 'r', encoding='utf-8') as f:
                        config = yaml.safe_load(f)
                    if config and 'robot_ip' in config:
                        cls._cached_ip = config['robot_ip']
                        logger.info(f"Robot IP loaded from config: {cls._cached_ip}")
                        return cls._cached_ip
                except Exception as e:
                    logger.error(f"Failed to read config file: {e}", exc_info=True)

        cls._cached_ip = cls.DEFAULT_IP
        logger.info(f"Using default robot IP: {cls._cached_ip}")
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
            logger.debug(f"Checking robot network connectivity: {ip}")
            result = subprocess.run(
                ["ping", "-c", "1", "-W", "1", ip],
                capture_output=True,
                timeout=2
            )
            reachable = result.returncode == 0
            if reachable:
                logger.debug(f"Robot {ip} is reachable")
            else:
                logger.debug(f"Robot {ip} is unreachable")
            return reachable
        except subprocess.TimeoutExpired:
            logger.error(f"Ping robot {ip} timed out (2s)")
            return False
        except Exception as e:
            logger.error(f"Robot connectivity check error: {e}", exc_info=True)
            return False


class StereoHeadScanner:
    """Scan for stereo head camera and v4l2loopback status."""

    STEREO_CAMERA_DEVICE = "/dev/stereo_camera"  # udev symlink
    LOOPBACK_DEVICE = "/dev/video99"             # v4l2loopback virtual device

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
        logger.debug("Checking stereo head camera")

        # Check udev symlink
        if os.path.exists(cls.STEREO_CAMERA_DEVICE):
            try:
                real_path = os.path.realpath(cls.STEREO_CAMERA_DEVICE)
                result['available'] = True
                result['device'] = real_path
                result['info'] = f"→ {real_path}"
                logger.debug(f"Stereo camera found: {cls.STEREO_CAMERA_DEVICE} -> {real_path}")
                return result
            except Exception as e:
                logger.error(f"Failed to read camera device link: {e}", exc_info=True)

        # Fallback: check common video devices
        logger.debug("Checking common video devices")
        for i in [0, 1, 2, 3, 4]:
            device = f"/dev/video{i}"
            if os.path.exists(device):
                # Try using v4l2-ctl to check if it's a stereo camera
                try:
                    check_result = subprocess.run(
                        ["v4l2-ctl", "-d", device, "--all"],
                        capture_output=True,
                        text=True,
                        timeout=2
                    )
                    output = check_result.stdout.lower()
                    # Check if it's a stereo camera (typically 2560x720 or similar resolution)
                    if "2560" in output or "stereo" in output or "dual" in output:
                        result['available'] = True
                        result['device'] = device
                        result['info'] = "Stereo camera detected"
                        logger.info(f"Stereo camera found at {device}")
                        return result
                except subprocess.TimeoutExpired:
                    logger.warning(f"Device {device} check timed out")
                except FileNotFoundError:
                    logger.error("v4l2-ctl command not found, please install v4l-utils")
                    break
                except Exception as e:
                    logger.debug(f"Device {device} check failed: {e}")

        result['info'] = "Stereo camera not found"
        logger.warning("Stereo head camera not found")
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
        logger.debug("Checking v4l2loopback device")

        # Check if v4l2loopback module is loaded
        try:
            lsmod_result = subprocess.run(
                ["lsmod"],
                capture_output=True,
                text=True,
                timeout=2
            )
            if "v4l2loopback" in lsmod_result.stdout:
                result['loaded'] = True
                logger.debug("v4l2loopback module loaded")
            else:
                logger.debug("v4l2loopback module not loaded")
        except Exception as e:
            logger.error(f"lsmod check failed: {e}", exc_info=True)

        # Check if /dev/video99 exists
        if os.path.exists(cls.LOOPBACK_DEVICE):
            result['available'] = True
            result['info'] = "v4l2loopback ready"
            logger.debug(f"Loopback device {cls.LOOPBACK_DEVICE} ready")
        elif result['loaded']:
            result['info'] = "Module loaded, device not created"
            logger.warning("v4l2loopback module loaded but device not created")
        else:
            result['info'] = "v4l2loopback not loaded"
            logger.warning("v4l2loopback not loaded")

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
        'chest': 'Chest',
        'right_wrist': 'R Wrist',
        'left_wrist': 'L Wrist',
        'left_arm': 'L Arm',
        'right_arm': 'R Arm',
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
        """Read tracker body-part to serial-number mapping from config (re-read each time)."""
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
            logger.debug("OpenVR not available, skipping tracker scan")
            return set()

        # Check if OpenVR is configured before attempting to initialize
        if not cls._is_openvr_configured():
            logger.debug("OpenVR not configured, skipping tracker scan")
            return set()

        try:
            logger.debug("Initializing OpenVR API")
            openvr.init(openvr.VRApplication_Other)
            vr = openvr.VRSystem()

            serials = set()
            logger.debug("Scanning Vive Tracker devices")
            for i in range(64):
                device_class = vr.getTrackedDeviceClass(i)
                if device_class == openvr.TrackedDeviceClass_GenericTracker:
                    # Check if device is actually connected (exclude cached offline devices)
                    if not vr.isTrackedDeviceConnected(i):
                        continue
                    serial = vr.getStringTrackedDeviceProperty(
                        i, openvr.Prop_SerialNumber_String
                    )
                    if serial:
                        serials.add(serial)

            logger.debug(f"Shutting down OpenVR API, found {len(serials)} tracker(s)")
            openvr.shutdown()
            return serials
        except Exception as e:
            logger.error(f"Vive Tracker scan error: {e}", exc_info=True)
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
            logger.debug("OpenVR not available, skipping base station scan")
            return {}

        # Check if OpenVR is configured before attempting to initialize
        if not cls._is_openvr_configured():
            logger.debug("OpenVR not configured, skipping base station scan")
            return {}

        try:
            logger.debug("Initializing OpenVR API (base station scan)")
            openvr.init(openvr.VRApplication_Other)
            vr = openvr.VRSystem()

            stations = {}
            logger.debug("Scanning Vive base stations")
            for i in range(64):
                device_class = vr.getTrackedDeviceClass(i)
                if device_class == openvr.TrackedDeviceClass_TrackingReference:
                    serial = vr.getStringTrackedDeviceProperty(
                        i, openvr.Prop_SerialNumber_String
                    )
                    connected = vr.isTrackedDeviceConnected(i)
                    # Try to get base station mode (A/B/C)
                    try:
                        mode = vr.getStringTrackedDeviceProperty(
                            i, openvr.Prop_ModeLabel_String
                        )
                    except Exception:
                        mode = "?"
                    if serial:
                        stations[serial] = {'mode': mode or "?", 'connected': connected}

            logger.debug(f"Shutting down OpenVR API, found {len(stations)} base station(s)")
            openvr.shutdown()
            return stations
        except Exception as e:
            logger.error(f"Vive base station scan error: {e}", exc_info=True)
            try:
                openvr.shutdown()
            except Exception:
                pass
            return {}

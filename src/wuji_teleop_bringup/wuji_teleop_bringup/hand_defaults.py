"""灵巧手硬件默认参数 — 从 wujihand_ik.yaml 统一加载

所有 launch 文件通过此模块读取灵巧手配置。
实际参数定义在: wujihand_output/config/wujihand_ik.yaml

Usage:
    from wuji_teleop_bringup.hand_defaults import (
        LEFT_HAND_SERIAL, RIGHT_HAND_SERIAL,
        LEFT_HAND_NAME, RIGHT_HAND_NAME,
        DRIVER_PUBLISH_RATE, DRIVER_FILTER_CUTOFF_FREQ, DRIVER_DIAGNOSTICS_RATE,
    )
"""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory


def _load_hand_config() -> dict:
    """Load hand config from wujihand_output package share directory."""
    config_path = (
        Path(get_package_share_directory("wujihand_output"))
        / "config"
        / "wujihand_ik.yaml"
    )
    with open(config_path) as f:
        return yaml.safe_load(f)


_config = _load_hand_config()

# ===== 导出常量 (向后兼容, launch 文件无需修改) =====

LEFT_HAND_SERIAL: str = _config["left_hand"]["serial_number"]
RIGHT_HAND_SERIAL: str = _config["right_hand"]["serial_number"]

LEFT_HAND_NAME: str = _config["left_hand"]["name"]
RIGHT_HAND_NAME: str = _config["right_hand"]["name"]

DRIVER_PUBLISH_RATE: float = float(_config["driver"]["publish_rate"])
DRIVER_FILTER_CUTOFF_FREQ: float = float(_config["driver"]["filter_cutoff_freq"])
DRIVER_DIAGNOSTICS_RATE: float = float(_config["driver"]["diagnostics_rate"])

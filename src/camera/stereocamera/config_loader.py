#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
头部双目相机配置加载工具

被 unified_stereo_node.py 使用。

作者: Liang ZHU
"""

import sys
from pathlib import Path

try:
    import yaml
except ImportError:
    print("[ERROR] PyYAML not installed. Install: pip install pyyaml")
    sys.exit(1)

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    print("[ERROR] ament_index_python not installed")
    sys.exit(1)


def load_stereo_head_config(config_path=None) -> dict:
    """
    加载头部双目相机配置

    参数:
        config_path: 可选的配置文件路径，为 None 时使用包内默认路径

    返回:
        stereo_head 配置字典
    """
    if config_path is None:
        pkg_share = get_package_share_directory('camera')
        config_path = Path(pkg_share) / 'config' / 'stereo_head' / 'stereo_head_config.yaml'
    else:
        config_path = Path(config_path)

    if not config_path.exists():
        print(f"[ERROR] Config file not found: {config_path}")
        sys.exit(1)

    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    return config.get('stereo_head', {})

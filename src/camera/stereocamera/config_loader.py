#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Head stereo camera configuration loader utility

Used by unified_stereo_node.py.

Author: Liang ZHU
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
    Load head stereo camera configuration

    Args:
        config_path: Optional config file path, uses package default path when None

    Returns:
        stereo_head configuration dictionary
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

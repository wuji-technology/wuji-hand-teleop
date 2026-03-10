"""Shared TF utilities for launch files."""

from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def load_tf_config() -> dict:
    """Load static transforms config."""
    share_dir = Path(get_package_share_directory("wuji_teleop_bringup"))
    with open(share_dir / "config" / "static_transforms.yaml") as f:
        return yaml.safe_load(f)


def create_chest_tf_nodes(config: dict = None, sides: list = None) -> list:
    """
    Create chest mount TF nodes.

    Args:
        config: Config dict, loads from yaml if None
        sides: ["left", "right"] or subset, defaults to both
    """
    if config is None:
        config = load_tf_config()
    if sides is None:
        sides = ["left", "right"]

    chest = config["chest_mount"]
    shared = chest["shared"]
    nodes = []

    for side in sides:
        s = chest[side]
        # Layer 1: chest -> {side}_chest_base
        nodes.append(Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=f"{side}_chest_base_tf",
            arguments=[
                str(shared["x"]), str(shared["y"]), str(shared["z"]),
                str(s["roll"]), str(s["pitch"]), str(s["yaw"]),
                "chest", f"{side}_chest_base"
            ],
        ))
        # Layer 2: {side}_chest_base -> {side}_chest
        nodes.append(Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=f"{side}_chest_tf",
            arguments=[
                str(s["local_x"]), str(s["local_y"]), str(s["local_z"]),
                "0", "0", "0",
                f"{side}_chest_base", f"{side}_chest"
            ],
        ))

    return nodes


def create_tianji_tf_nodes(config: dict = None, sides: list = None) -> list:
    """Create wrist to tianji TF nodes."""
    if config is None:
        config = load_tf_config()
    if sides is None:
        sides = ["left", "right"]

    tianji = config["wrist_to_tianji"]
    nodes = []

    for side in sides:
        t = tianji[side]
        nodes.append(Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=f"tianji_{side}_tf",
            arguments=[
                str(t["x"]), str(t["y"]), str(t["z"]),
                str(t["qx"]), str(t["qy"]), str(t["qz"]), str(t["qw"]),
                f"{side}_wrist", f"tianji_{side}"
            ],
        ))

    return nodes

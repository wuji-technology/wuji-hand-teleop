"""Shared launch-time helpers for wuji_teleop_bringup."""
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory


VALID_INPUT_SOURCES = ("wuji_glove", "manus")


def resolve_config_path(package: str, config_file: str) -> str:
    """Return the ament share-dir path to a config yaml.

    Repo tracks only `<config_file>.template` (placeholders); the operator's
    live `<config_file>` (real SNs / IPs) is gitignored and seeded from the
    template by `docker/entrypoint.sh` on container start. By the time any
    launch helper calls this, the live yaml exists at the returned path.
    Single resolution path — no fallback / no special-case override layer.
    """
    return str(Path(get_package_share_directory(package)) / "config" / config_file)


def read_input_source() -> str:
    """Read `input_source` from wujihand_ik.yaml at launch evaluation.

    Resolves via the same template-seeded path as resolve_config_path.
    Raises FileNotFoundError if the yaml is missing (colcon build skipped or
    wujihand_output not installed) — no silent fallback to a default source.
    """
    yaml_path = Path(resolve_config_path("wujihand_output", "wujihand_ik.yaml"))
    with open(yaml_path) as f:
        cfg = yaml.safe_load(f) or {}
    src = cfg.get("input_source", "wuji_glove")
    if src not in VALID_INPUT_SOURCES:
        raise ValueError(
            f"wujihand_ik.yaml::input_source = {src!r} is not one of {VALID_INPUT_SOURCES}"
        )
    return src

"""Tianji servo fault-code helpers maintained outside the official SDK files.

The vendor may replace ``_internal/fx_robot.py`` and ``_internal/fx_kine.py``
when the controller SDK is upgraded. Keep Wuji-side normalization, lookup, and
raw servo-error reading here so SDK refreshes only need dictionary updates or a
thin import check instead of re-applying local patches to vendor files.
"""

from __future__ import annotations

import ctypes
from typing import Any, Iterable

try:
    from ._internal.fx_robot import fault_code_dict
except ImportError:  # pragma: no cover - installed package fallback
    from tianji_output._internal.fx_robot import fault_code_dict


def normalize_fault_code(code: Any) -> str:
    """Return SDK fault-code key format, for example ``0xFF26``.

    Tianji's dictionary uses mixed-case hex keys. Normalize raw int/string
    values before lookup so SDK/user output like ``0xff26`` does not miss
    ``fault_code_dict["0xFF26"]``.
    """
    if code is None:
        return "0x0000"
    if isinstance(code, int):
        return f"0x{(code & 0xFFFF):04X}"
    text = str(code).strip()
    if not text:
        return "0x0000"
    if text.lower().startswith("0x"):
        text = text[2:]
    try:
        value = int(text, 16)
    except ValueError:
        return str(code).strip()
    return f"0x{(value & 0xFFFF):04X}"


def describe_fault_code(code: Any, empty: str = "") -> str:
    """Look up one servo fault code in the central Tianji SDK dictionary."""
    normalized = normalize_fault_code(code)
    if normalized == "0x0000":
        return empty
    return fault_code_dict.get(normalized, f"unknown({normalized})")


def describe_fault_codes(codes: Iterable[Any], empty: str = "") -> list[str]:
    """Return one description per joint, preserving the input list shape."""
    return [describe_fault_code(code, empty=empty) for code in codes]


def format_fault_descriptions(codes: Iterable[Any]) -> str:
    """Return the vendor-style multi-line string for CLI diagnostics."""
    lines = []
    for joint_idx, code in enumerate(codes, 1):
        normalized = normalize_fault_code(code)
        if normalized == "0x0000":
            continue
        lines.append(
            f"joint {joint_idx}: {normalized} - "
            f"{describe_fault_code(normalized, empty='')}"
        )
    if not lines:
        return "None"
    if len(lines) == 1:
        return lines[0]
    return "Fault information for each joint:\n" + "\n".join(
        f"{idx}. {line}" for idx, line in enumerate(lines, 1)
    )


def read_servo_fault_codes(robot_handle: Any, arm: str) -> list[str]:
    """Read raw 7-axis servo fault codes from a Tianji ctypes handle.

    Args:
        robot_handle: the underlying ``ctypes.CDLL`` handle (``Marvin_Robot.robot``).
        arm: ``'A'`` for left arm or ``'B'`` for right arm.
    """
    if arm not in ("A", "B"):
        raise ValueError(f"arm must be 'A' or 'B', got {arm!r}")
    err_buf = (ctypes.c_long * 7)()
    fn = robot_handle.OnGetServoErr_A if arm == "A" else robot_handle.OnGetServoErr_B
    fn.argtypes = [ctypes.POINTER(ctypes.c_long * 7)]
    fn(ctypes.byref(err_buf))
    return [normalize_fault_code(err_buf[i]) for i in range(7)]

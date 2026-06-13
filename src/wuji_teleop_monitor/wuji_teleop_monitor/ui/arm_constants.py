# -*- coding: utf-8 -*-
"""Arm constants — init_joints, state codes, and error-code mapping.

Source: Tianji SDK documentation
https://github.com/cynthia-you/TJ_FX_ROBOT_CONTRL_SDK/blob/master/README.md
"""

# Init joints reference (from tianji_chest.yaml / tianji_world.yaml)
INIT_LEFT = [55.0, -65.0, -70.0, -60.0, 60.0, 0.0, 0.0]
INIT_RIGHT = [-55.0, -65.0, 70.0, -60.0, -60.0, 0.0, 0.0]

# State codes (SDK cur_state, sub_data["states"][i]["cur_state"]).
#   0-3: terminal | 100: error terminal
#   101-103: instantaneous transition states (auto-advance to a terminal state)
STATE_NAMES = {
    0: "servo off",
    1: "position following",
    2: "PVT",
    3: "impedance",
    100: "switch failed (clear error required)",
    101: "switching -> position",
    102: "switching -> PVT",
    103: "switching -> impedance",
}

# Error codes (SDK err_code, sub_data["states"][i]["err_code"]).
# The upstream SDK header's #define suffixes (PositionModeOK/SensorModeOK/
# EnableServoOK/DisableServoOK) contradict the inline comments ("...failed"),
# so we follow the public SDK README's descriptions.
ERR_DESCRIPTIONS = {
    0: "ok",
    1: "bus topology fault",
    2: "servo fault",
    3: "PVT fault",
    4: "request-enter-position failed",
    5: "enter-position failed",
    6: "request-enter-torque failed (common after e-stop: joint at its limit)",
    7: "enter-torque failed",
    8: "request-enable-servo",
    9: "enable-servo failed",
    10: "request-disable-servo failed",
    11: "disable-servo failed",
    12: "internal error",
    13: "e-stop",
    14: "floating-base config error (UMI not enabled)",
}

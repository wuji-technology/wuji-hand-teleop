# -*- coding: utf-8 -*-
"""Qt plugin-path setup — works inside containers and on the host."""

import os
import sys


def setup_qt_plugins():
    """Set QT_QPA_PLATFORM_PLUGIN_PATH so Qt can locate the xcb plugin.

    Must be called before any PyQt5 import.
    """
    candidates = [
        "/usr/lib/x86_64-linux-gnu/qt5/plugins",
        "/usr/lib/aarch64-linux-gnu/qt5/plugins",
        os.path.join(sys.prefix, "lib", "qt5", "plugins"),
    ]
    for path in candidates:
        if os.path.isdir(path):
            os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = path
            break
    os.environ.pop("QT_PLUGIN_PATH", None)

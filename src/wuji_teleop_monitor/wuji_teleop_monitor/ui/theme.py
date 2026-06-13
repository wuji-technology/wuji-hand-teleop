# -*- coding: utf-8 -*-
"""Dark-theme CSS — shared by every UI."""

# Global dark theme (applied to QMainWindow).
DARK_THEME_CSS = """
    QMainWindow { background-color: #2b2b2b; }
    QGroupBox {
        font-weight: bold; border: 1px solid #555;
        border-radius: 5px; margin-top: 10px; padding-top: 10px;
        color: #ddd;
    }
    QGroupBox::title {
        subcontrol-origin: margin; left: 10px; padding: 0 5px;
    }
    QLabel { color: #ddd; }
    QMessageBox { background-color: #ffffff; }
    QMessageBox QLabel { color: #000000; font-size: 13px; }
    QMessageBox QPushButton {
        background-color: #e0e0e0; color: #000000;
        border: 1px solid #999; border-radius: 4px;
        padding: 6px 20px; min-width: 80px;
    }
"""

# Release button (orange — destructive action).
RELEASE_BUTTON_CSS = """
    QPushButton {
        background-color: #E65100; color: white;
        border: 1px solid #BF360C; border-radius: 6px;
        padding: 8px 20px; font-size: 13px; font-weight: bold;
    }
    QPushButton:hover { background-color: #F57C00; }
    QPushButton:disabled { background-color: #555; color: #888; }
"""

# Hold button (grey).
HOLD_BUTTON_CSS = """
    QPushButton {
        background-color: #37474F; color: #ccc;
        border: 1px solid #546E7A; border-radius: 6px;
        padding: 8px 20px; font-size: 13px;
    }
    QPushButton:hover { background-color: #455A64; color: white; }
    QPushButton:disabled { background-color: #555; color: #888; }
"""

# Read-status button (blue).
READ_BUTTON_CSS = """
    QPushButton {
        background-color: #1565C0; color: white;
        border: 1px solid #0D47A1; border-radius: 6px;
        padding: 6px 20px; font-size: 12px;
    }
    QPushButton:hover { background-color: #1976D2; }
    QPushButton:disabled { background-color: #555; color: #888; }
"""

# Clear-error button (amber).
CLEAR_ERROR_BUTTON_CSS = """
    QPushButton {
        background-color: #F9A825; color: #212121;
        border: 1px solid #F57F17; border-radius: 6px;
        padding: 8px 20px; font-size: 13px; font-weight: bold;
    }
    QPushButton:hover { background-color: #FBC02D; }
    QPushButton:disabled { background-color: #555; color: #888; }
"""

# Log QTextEdit / QPlainTextEdit + scrollbar.
LOG_TEXTEDIT_CSS = """
    QPlainTextEdit, QTextEdit {
        background-color: #1e1e1e; color: #ddd;
        border: 1px solid #555; border-radius: 5px;
        padding: 5px;
    }
    QScrollBar:vertical {
        background: #2a2a2a; width: 12px; border-radius: 6px;
    }
    QScrollBar::handle:vertical {
        background: #666; min-height: 30px; border-radius: 6px;
    }
    QScrollBar::handle:vertical:hover {
        background: #888;
    }
    QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
        height: 0px;
    }
"""

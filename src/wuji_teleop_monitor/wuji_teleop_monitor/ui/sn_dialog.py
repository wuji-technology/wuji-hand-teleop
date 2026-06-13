#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Scan-SN dialog: enumerate Wuji Hand (USB) + Wuji Glove (SDK) and write
their serial numbers back to wujihand_ik.yaml / wuji_glove.yaml.

Scanners degrade gracefully when their backends are missing (lsusb, wuji_sdk,
ruamel.yaml) so the Monitor stays usable on hosts without them.
"""

from __future__ import annotations

import difflib
import io
import re
import subprocess
import threading
from pathlib import Path
from typing import Callable, List, Optional

from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QApplication, QDialog, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QGroupBox, QPlainTextEdit, QMessageBox, QWidget, QSizePolicy, QScrollArea,
)

from ament_index_python.packages import get_package_share_directory

LogCallback = Callable[[str], None]

_DIALOG_DARK_CSS = """
    QDialog { background-color: #2b2b2b; }
    QLabel { color: #e8e8e8; }
    QGroupBox {
        color: #f0f0f0; font-weight: bold;
        border: 1px solid #555; border-radius: 5px;
        margin-top: 8px; padding-top: 8px;
    }
    QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; }
    QPushButton {
        background-color: #555; color: #f0f0f0;
        border: 1px solid #666; border-radius: 4px;
        padding: 4px 14px; font-size: 12px;
    }
    QPushButton:hover { background-color: #666; }
    QPushButton:disabled { background-color: #3a3a3a; color: #777; }
    QPlainTextEdit {
        background-color: #1e1e1e; color: #e8e8e8;
        border: 1px solid #555; border-radius: 4px;
        padding: 4px;
    }
"""


def _resolve_config_path(pkg_name: str, filename: str) -> Optional[Path]:
    """Resolve a package config file via ament_index.

    Single-path lookup matching wuji_teleop_bringup.launch_utils: the live
    `<filename>` is seeded from `<filename>.template` by docker/entrypoint.sh
    on container start, so launch helpers and Scan SNs read/write the same
    file. Returns the canonical (symlink-resolved) path so writes land in the
    bind-mounted source tree, not the install/share symlink chain.
    """
    try:
        cand = Path(get_package_share_directory(pkg_name)) / "config" / filename
    except Exception:
        return None
    return cand.resolve() if cand.exists() else None


# Container bind-mount: docker-compose.yml maps host `<repo>/public/src` →
# container `/home/wuji/ros2_ws/src`. Splitting on this prefix lets us show
# the equivalent host path so operators editing on the host (where the UI
# pixels render via X11) know which file actually changes on disk.
_CONTAINER_SRC_PREFIX = "/home/wuji/ros2_ws/src/"


def _format_path_lines(path: Path) -> List[str]:
    """Render a YAML path as one or two friendly lines.

    If the path lives under the container's bind-mounted `src/` tree, emit:
      Container: /home/wuji/ros2_ws/src/<rest>
      Host:      <repo>/public/src/<rest>   (Docker bind-mount)
    Otherwise just return the absolute path.
    """
    s = str(path)
    if s.startswith(_CONTAINER_SRC_PREFIX):
        rest = s[len(_CONTAINER_SRC_PREFIX):]
        return [
            f"Container: {_CONTAINER_SRC_PREFIX}{rest}",
            f"Host:      &lt;repo&gt;/public/src/{rest}   (Docker bind-mount)",
        ]
    return [s]


# -------------------- YAML helpers --------------------

def _load_yaml(path: Path, log: LogCallback):
    """Load YAML preserving comments + ordering when ruamel.yaml is available."""
    try:
        from ruamel.yaml import YAML
        yaml = YAML()
        yaml.preserve_quotes = True
        with path.open("r", encoding="utf-8") as f:
            return yaml.load(f), yaml
    except ImportError:
        log("WARNING: ruamel.yaml not installed; YAML comments will be lost")
        import yaml as pyyaml
        with path.open("r", encoding="utf-8") as f:
            return pyyaml.safe_load(f), None


def _dump_yaml(path: Path, data, yaml_handle) -> None:
    if yaml_handle is not None:
        with path.open("w", encoding="utf-8") as f:
            yaml_handle.dump(data, f)
    else:
        import yaml as pyyaml
        with path.open("w", encoding="utf-8") as f:
            pyyaml.safe_dump(data, f, sort_keys=False)


def _read_current_sns(path: Path, left_key: str, right_key: str,
                      log: LogCallback) -> dict:
    """Returns {'left': sn-or-None, 'right': sn-or-None}; {} if YAML unreadable."""
    if path is None or not path.exists():
        return {}
    try:
        data, _ = _load_yaml(path, log)
    except Exception as e:
        log(f"Read current SNs from {path.name} failed: {e}")
        return {}
    if not isinstance(data, dict):
        return {}
    out = {}
    for tag, key in (("left", left_key), ("right", right_key)):
        section = data.get(key)
        if isinstance(section, dict):
            sn = section.get("serial_number")
            out[tag] = str(sn) if sn not in (None, "") else None
        else:
            out[tag] = None
    return out


def _dump_to_string(data, yaml_handle) -> str:
    if yaml_handle is not None:
        buf = io.StringIO()
        yaml_handle.dump(data, buf)
        return buf.getvalue()
    import yaml as pyyaml
    return pyyaml.safe_dump(data, sort_keys=False)


# -------------------- Scanners --------------------

class WujiHandScanner:
    """Wuji Hand SNs via `lsusb -v -d 0483:2000`."""

    USB_VID = "0483"
    USB_PID = "2000"

    @classmethod
    def scan(cls, log: LogCallback) -> List[dict]:
        try:
            result = subprocess.run(
                ["lsusb", "-v", "-d", f"{cls.USB_VID}:{cls.USB_PID}"],
                capture_output=True, text=True, timeout=5,
            )
        except FileNotFoundError:
            log("Wuji Hand scan: lsusb not found (install usbutils)")
            return []
        except subprocess.TimeoutExpired:
            log("Wuji Hand scan: lsusb timed out (5s)")
            return []
        except Exception as e:
            log(f"Wuji Hand scan failed: {e}")
            return []

        devices: List[dict] = []
        current_bus = ""
        current_device = ""
        for line in result.stdout.split("\n"):
            stripped = line.strip()
            if stripped.startswith("Bus ") and "Device " in stripped:
                # e.g. "Bus 003 Device 005: ID 0483:2000 ..."
                parts = stripped.split()
                if len(parts) >= 4:
                    current_bus = parts[1]
                    current_device = parts[3].rstrip(":")
            if "iSerial" in stripped:
                parts = stripped.split()
                if len(parts) >= 3:
                    sn = parts[-1]
                    if sn and sn != "0":
                        devices.append({
                            "sn": sn,
                            "bus": current_bus,
                            "device": current_device,
                        })
        return devices


class WujiGloveScanner:
    """Wuji Glove SNs via `wuji_sdk.SdkManager.instance().scan()`."""

    @classmethod
    def is_sdk_available(cls) -> bool:
        try:
            import wuji_sdk  # noqa: F401
            return True
        except ImportError:
            return False

    @classmethod
    def scan(cls, log: LogCallback) -> List[dict]:
        if not cls.is_sdk_available():
            log("Wuji SDK unavailable - install via pip in container")
            return []
        try:
            from wuji_sdk import SdkManager
            devices = SdkManager.instance().scan()
        except Exception as e:
            log(f"Wuji Glove scan failed: {e}")
            return []

        results: List[dict] = []
        zenoh_skipped = 0
        for d in devices:
            sn = getattr(d, "sn", "") or ""
            address = getattr(d, "address", "") or ""
            transport = str(getattr(d, "transport_type", "") or "")
            if not sn:
                continue
            # Drop Zenoh-discovered entries — those are gloves visible on the
            # LAN via the SDK's bridge, which can include other operators'
            # hardware. Save-to-YAML only ever wants the directly-attached
            # UDP gloves on this operator's harness.
            if "udp" not in transport.lower():
                zenoh_skipped += 1
                continue
            results.append({
                "sn": sn,
                "address": address,
                "transport": transport,
            })
        if zenoh_skipped:
            log(f"[scan] Wuji Glove: {len(results)} UDP, "
                f"skipped {zenoh_skipped} via Zenoh")
        return results


# -------------------- Side auto-detection --------------------

# Wuji Glove convention: factory configures the left glove with the
# .100 IP and the right glove with .101 on the operator's harness LAN.
# Override only if a deployment uses different IPs.
_GLOVE_LEFT_OCTET = 100
_GLOVE_RIGHT_OCTET = 101
_IP_LAST_OCTET_RE = re.compile(r"\.(\d+)(?::|$)")


def _suggest_side(device: dict, kind: str) -> Optional[str]:
    """Return 'left' / 'right' / None for a detected device.

    Heuristics:
      - Wuji Glove: last octet of IP (`192.168.1.100` -> left, `.101` -> right).
      - Wuji Hand: current-gen SNs carry no L/R marker; next-gen reportedly
        will. Return None until that format is documented.
    """
    if kind == "glove":
        addr = device.get("address") or ""
        m = _IP_LAST_OCTET_RE.search(addr)
        if m:
            octet = int(m.group(1))
            if octet == _GLOVE_LEFT_OCTET:
                return "left"
            if octet == _GLOVE_RIGHT_OCTET:
                return "right"
    return None


# -------------------- Save-to-YAML dialog --------------------

class SaveToYamlDialog(QDialog):
    """Modal diff preview + write-back for a single SN edit."""

    def __init__(self, parent, path: Path, side: str, kind: str, sn: str,
                 log: LogCallback):
        super().__init__(parent)
        self._path = path
        self._side = side
        self._kind = kind
        self._sn = sn
        self._log = log
        self._yaml_handle = None
        self._new_data = None
        self._new_text = ""
        self._old_text = ""

        self.setWindowTitle(f"Save {kind} SN -> {side}")
        self.setMinimumSize(520, 340)
        self.setStyleSheet(_DIALOG_DARK_CSS)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(8)

        header = QLabel(f"Target: {path}\nSetting {side}.serial_number = {sn}")
        header.setWordWrap(True)
        layout.addWidget(header)

        self._diff_view = QPlainTextEdit()
        self._diff_view.setReadOnly(True)
        self._diff_view.setFont(QFont("Courier New", 10))
        layout.addWidget(self._diff_view, 1)

        btn_row = QHBoxLayout()
        btn_row.addStretch()
        self._confirm_btn = QPushButton("Confirm")
        self._confirm_btn.clicked.connect(self._on_confirm)
        cancel_btn = QPushButton("Cancel")
        cancel_btn.clicked.connect(self.reject)
        btn_row.addWidget(cancel_btn)
        btn_row.addWidget(self._confirm_btn)
        layout.addLayout(btn_row)

        self._prepare_diff()

    def _prepare_diff(self) -> None:
        if not self._path.exists():
            self._diff_view.setPlainText(
                f"ERROR: YAML not found: {self._path}\n"
                "Verify the package is built and sourced.")
            self._confirm_btn.setEnabled(False)
            return

        try:
            self._old_text = self._path.read_text(encoding="utf-8")
            data, yaml_handle = _load_yaml(self._path, self._log)
            self._yaml_handle = yaml_handle
        except Exception as e:
            self._diff_view.setPlainText(f"ERROR loading YAML: {e}")
            self._confirm_btn.setEnabled(False)
            return

        side_key = self._side
        if data is None or side_key not in data:
            self._diff_view.setPlainText(
                f"ERROR: top-level key '{side_key}' not found in {self._path.name}.\n"
                f"Available keys: {list(data) if data else '(empty)'}\n\n"
                f"Add this manually:\n"
                f"# [{self._kind} SN goes here]\n"
                f"{side_key}:\n"
                f"  serial_number: \"{self._sn}\"")
            self._confirm_btn.setEnabled(False)
            return

        section = data[side_key]
        if not isinstance(section, dict) or "serial_number" not in section:
            self._diff_view.setPlainText(
                f"ERROR: '{side_key}.serial_number' not present.\n"
                f"Add manually:\n"
                f"  {side_key}.serial_number: \"{self._sn}\"")
            self._confirm_btn.setEnabled(False)
            return

        section["serial_number"] = self._sn
        self._new_data = data
        self._new_text = _dump_to_string(data, self._yaml_handle)

        diff_lines = difflib.unified_diff(
            self._old_text.splitlines(keepends=True),
            self._new_text.splitlines(keepends=True),
            fromfile=str(self._path) + " (current)",
            tofile=str(self._path) + " (proposed)",
            n=2,
        )
        diff_text = "".join(diff_lines)
        if not diff_text.strip():
            diff_text = "(no changes - file already contains this SN)"
            self._confirm_btn.setEnabled(False)
        self._diff_view.setPlainText(diff_text)

    def _on_confirm(self) -> None:
        try:
            _dump_yaml(self._path, self._new_data, self._yaml_handle)
        except Exception as e:
            QMessageBox.critical(self, "Write failed", f"Could not write YAML:\n{e}")
            self._log(f"YAML write failed: {self._path}: {e}")
            return
        self._log(f"[save] {self._side}.serial_number = {self._sn}  → {self._path.name}")
        self.accept()


# -------------------- Main scan dialog --------------------

class ScanSNDialog(QDialog):
    """Modal dialog: run both scanners in a background thread, then show
    per-device Save-as-left / Save-as-right buttons."""

    _results_signal = pyqtSignal(list, list)

    def __init__(self, parent=None, log_callback: Optional[LogCallback] = None):
        super().__init__(parent)
        self._log = log_callback or (lambda _msg: None)
        self.setWindowTitle("Scan device SNs")
        # Portrait, wide enough for "Container: /home/wuji/ros2_ws/src/..."
        # paths to render on one line, tall enough that 6 detected devices +
        # save buttons fit without scrolling on a 1080p display. Scroll area
        # absorbs anything beyond.
        self.setMinimumSize(560, 760)

        self.setStyleSheet(_DIALOG_DARK_CSS)

        self._layout = QVBoxLayout(self)
        self._layout.setContentsMargins(10, 10, 10, 10)
        self._layout.setSpacing(8)
        self._status = QLabel("Scanning Wuji Hand (USB) and Wuji Glove (SDK)...")
        self._layout.addWidget(self._status)

        # Scrollable inner area — keeps the dialog at a sane proportion even
        # when 4+ devices push the result list past the viewport.
        self._scroll = QScrollArea()
        self._scroll.setWidgetResizable(True)
        self._scroll.setFrameShape(QScrollArea.NoFrame)
        self._scroll.setStyleSheet(
            "QScrollArea { background-color: #2b2b2b; border: none; }"
            "QScrollBar:vertical { background: #2a2a2a; width: 10px; }"
            "QScrollBar::handle:vertical { background: #666; border-radius: 5px;"
            " min-height: 30px; }"
            "QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical"
            " { height: 0px; }")
        self._results_host = QWidget()
        self._results_host.setStyleSheet("background-color: #2b2b2b;")
        self._results_host_layout = QVBoxLayout(self._results_host)
        self._results_host_layout.setContentsMargins(0, 0, 0, 0)
        self._results_host_layout.setSpacing(8)
        self._scroll.setWidget(self._results_host)
        self._layout.addWidget(self._scroll, 1)

        btn_row = QHBoxLayout()
        btn_row.addStretch()
        self._rescan_btn = QPushButton("Rescan")
        self._rescan_btn.setEnabled(False)
        self._rescan_btn.clicked.connect(self._start_scan)
        close_btn = QPushButton("Close")
        close_btn.clicked.connect(self.accept)
        btn_row.addWidget(self._rescan_btn)
        btn_row.addWidget(close_btn)
        self._layout.addLayout(btn_row)

        # Loading widget — a layout child added to _results_host while the
        # background thread blocks. Layout-embedded (not absolute-positioned)
        # so its geometry is computed by Qt's normal layout pass rather than
        # racing the dialog's first showEvent.
        self._loading_widget: Optional[QWidget] = None
        self._loading_label: Optional[QLabel] = None
        self._loading_dots = 0
        self._loading_timer = QTimer(self)
        self._loading_timer.timeout.connect(self._tick_loading)

        self._results_signal.connect(self._on_results)
        # Loading page is the dialog's INITIAL content so the very first frame
        # is "please wait" (not an empty/black viewport). The scan itself is
        # kicked off from the first paintEvent (below) — only after the window
        # has actually painted/composited is it safe to start the worker; an
        # earlier kick (showEvent / QTimer in __init__) paints to a back buffer
        # the compositor never displays on first open, hence the black screen.
        self._scan_started = False
        self._install_loading_widget()

    def paintEvent(self, event):
        super().paintEvent(event)
        # The first real paint just happened → the "please wait" loading page
        # (installed in __init__) is now on screen. Only now start the scan,
        # deferred one short tick so this frame flushes to the compositor
        # before the multi-second worker runs. Guarded so it fires exactly once.
        if self._scan_started:
            return
        self._scan_started = True
        QTimer.singleShot(60, self._start_scan)

    def _start_scan(self) -> None:
        self._status.setText("Scanning Wuji Hand (USB) and Wuji Glove (SDK)...")
        self._rescan_btn.setEnabled(False)
        self._clear_results()
        self._install_loading_widget()
        QApplication.setOverrideCursor(Qt.WaitCursor)
        # Synchronous repaint so the "please wait" loading page is actually on
        # screen before the worker starts. The SDK/Zenoh scan can hold the GIL
        # for seconds and starve the GUI, so processEvents alone isn't enough
        # on first open (the window may not be exposed yet) — repaint() forces
        # an immediate paint of the dialog + loading widget.
        self.repaint()
        threading.Thread(target=self._scan_worker, daemon=True,
                         name="sn-scan").start()

    def _install_loading_widget(self) -> None:
        """Add a centered "Scanning..." widget to the results layout."""
        self._loading_widget = QWidget()
        self._loading_widget.setStyleSheet(
            "QWidget { background-color: #2b2b2b; }")
        outer = QVBoxLayout(self._loading_widget)
        outer.setContentsMargins(40, 60, 40, 60)
        outer.addStretch(1)
        self._loading_label = QLabel("⟳  Scanning devices, please wait")
        self._loading_label.setAlignment(Qt.AlignCenter)
        self._loading_label.setWordWrap(True)
        self._loading_label.setStyleSheet(
            "QLabel {"
            "  color: #ffb74d; font-size: 16px; font-weight: bold;"
            "  background-color: #1e1e1e;"
            "  border: 1px solid #555; border-radius: 8px;"
            "  padding: 24px;"
            "}")
        outer.addWidget(self._loading_label)
        outer.addStretch(1)
        self._results_host_layout.addWidget(self._loading_widget)
        self._loading_dots = 0
        self._loading_timer.start(350)

    def _tick_loading(self) -> None:
        self._loading_dots = (self._loading_dots + 1) % 4
        if self._loading_label is None:
            return
        try:
            self._loading_label.setText(
                f"⟳  Scanning devices, please wait{'.' * self._loading_dots}")
        except RuntimeError:
            self._loading_label = None

    def _scan_worker(self) -> None:
        hands = WujiHandScanner.scan(self._log)
        gloves = WujiGloveScanner.scan(self._log)
        self._results_signal.emit(hands, gloves)

    def _clear_results(self) -> None:
        while self._results_host_layout.count():
            item = self._results_host_layout.takeAt(0)
            w = item.widget()
            if w is not None:
                w.deleteLater()
        self._loading_widget = None
        self._loading_label = None

    def _on_results(self, hands: list, gloves: list) -> None:
        self._loading_timer.stop()
        QApplication.restoreOverrideCursor()
        self._clear_results()

        self._status.setText(
            f"Found {len(hands)} Wuji Hand(s), {len(gloves)} Wuji Glove(s).")
        self._rescan_btn.setEnabled(True)

        hand_yaml = _resolve_config_path("wujihand_output", "wujihand_ik.yaml")
        glove_yaml = _resolve_config_path("wuji_glove", "wuji_glove.yaml")

        self._results_host_layout.addWidget(self._build_group(
            "Wuji Hand", hands, hand_yaml, "hand",
            left_key="left_hand", right_key="right_hand"))
        self._results_host_layout.addWidget(self._build_group(
            "Wuji Glove", gloves, glove_yaml, "glove",
            left_key="left_glove", right_key="right_glove"))

    _SECTION_CSS = "color: #888; font-size: 9px; font-weight: bold; letter-spacing: 1px;"
    _BODY_CSS = "color: #ddd; font-size: 10px;"
    _MUTED_CSS = "color: #888; font-size: 9px;"
    _SN_CSS = "color: #e8e8e8; font-size: 11px;"

    def _section_header(self, text: str) -> QLabel:
        lbl = QLabel(text.upper())
        lbl.setStyleSheet(self._SECTION_CSS)
        return lbl

    def _build_group(self, title: str, devices: list, yaml_path: Optional[Path],
                     kind: str, left_key: str, right_key: str) -> QGroupBox:
        box = QGroupBox(title)
        box.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred)
        v = QVBoxLayout(box)
        v.setSpacing(4)
        v.setContentsMargins(10, 6, 10, 8)

        if yaml_path is None:
            err = QLabel(f"YAML target not resolved for {kind}; build/source the package.")
            err.setStyleSheet(self._BODY_CSS)
            v.addWidget(err)
            return box

        # Section 1 — TARGET (file name big-ish, container + host paths small).
        v.addWidget(self._section_header("Target"))
        name_lbl = QLabel(f"   {yaml_path.name}")
        name_lbl.setStyleSheet(self._BODY_CSS)
        v.addWidget(name_lbl)
        # Render path lines individually so the mono "Container:" / "Host:" key
        # column lines up vertically when both lines are present.
        for path_line in _format_path_lines(yaml_path):
            line_lbl = QLabel(f"   {path_line}")
            line_lbl.setStyleSheet(
                "color: #888; font-size: 9px; font-family: 'Courier New';")
            line_lbl.setTextFormat(Qt.RichText)
            line_lbl.setWordWrap(True)
            v.addWidget(line_lbl)

        # Section 2 — CURRENTLY SAVED (small mono, colour-coded by side).
        current = _read_current_sns(yaml_path, left_key, right_key, self._log)
        left_now = current.get("left") or "(unset)"
        right_now = current.get("right") or "(unset)"
        v.addWidget(self._section_header("Currently saved"))
        saved_html = (
            f"   <span style='color:#888;'>Left&nbsp;&nbsp;</span>"
            f"<span style='color:#81C784;'>{left_now}</span><br>"
            f"   <span style='color:#888;'>Right&nbsp;</span>"
            f"<span style='color:#FFB74D;'>{right_now}</span>"
        )
        saved_lbl = QLabel(saved_html)
        saved_lbl.setTextFormat(Qt.RichText)
        saved_lbl.setFont(QFont("Courier New", 10))
        saved_lbl.setStyleSheet("color: #ddd;")
        saved_lbl.setWordWrap(True)
        v.addWidget(saved_lbl)

        # Section 3 — DETECTED.
        v.addWidget(self._section_header(f"Detected ({len(devices)})"))
        if not devices:
            empty = QLabel("   (no devices found)")
            empty.setStyleSheet(self._MUTED_CSS)
            v.addWidget(empty)
            return box

        for d in devices:
            sn = d["sn"]

            extra_parts = []
            if d.get("bus") and d.get("device"):
                extra_parts.append(f"bus={d['bus']} dev={d['device']}")
            if d.get("address"):
                extra_parts.append(f"addr={d['address']}")
            if d.get("transport"):
                extra_parts.append(f"tx={d['transport']}")
            extra_text = ", ".join(extra_parts)

            suggested = _suggest_side(d, kind)

            if sn == current.get("left"):
                tag_html = "<span style='color:#81C784;'>(currently left)</span>"
            elif sn == current.get("right"):
                tag_html = "<span style='color:#FFB74D;'>(currently right)</span>"
            elif suggested == "left":
                tag_html = "<span style='color:#81C784;'>(suggested: LEFT)</span>"
            elif suggested == "right":
                tag_html = "<span style='color:#FFB74D;'>(suggested: RIGHT)</span>"
            else:
                tag_html = "<span style='color:#888;'>(new)</span>"

            # SN line — courier mono, tag floats right via spaces.
            sn_lbl = QLabel(
                f"   <span style='color:#e8e8e8; font-family:\"Courier New\";'>"
                f"{sn}</span> &nbsp; {tag_html}")
            sn_lbl.setTextFormat(Qt.RichText)
            sn_lbl.setStyleSheet("font-size: 11px;")
            v.addWidget(sn_lbl)

            if extra_text:
                extra_lbl = QLabel(f"      {extra_text}")
                extra_lbl.setStyleSheet(self._MUTED_CSS)
                v.addWidget(extra_lbl)

            # Save buttons — wide enough to always fit; suggested side gets
            # the primary blue treatment.
            btn_row = QHBoxLayout()
            btn_row.setContentsMargins(24, 2, 8, 4)
            btn_row.setSpacing(8)
            primary_css = (
                "QPushButton { background-color: #1565C0; color: white;"
                " border: 1px solid #0D47A1; border-radius: 3px;"
                " padding: 2px 12px; font-size: 11px; font-weight: bold; }"
                "QPushButton:hover { background-color: #1976D2; }")
            secondary_css = (
                "QPushButton { background-color: #4a4a4a; color: #ddd;"
                " border: 1px solid #5a5a5a; border-radius: 3px;"
                " padding: 2px 12px; font-size: 11px; }"
                "QPushButton:hover { background-color: #5a5a5a; }")
            for label_text, side_key, side_tag in (
                    ("Save as left", left_key, "left"),
                    ("Save as right", right_key, "right")):
                btn = QPushButton(label_text)
                btn.setMinimumWidth(120)
                btn.setFixedHeight(26)
                btn.setStyleSheet(
                    primary_css if side_tag == suggested else secondary_css)
                btn.clicked.connect(
                    lambda _c=False, sn=sn, k=side_key:
                    self._open_save(yaml_path, k, kind, sn))
                btn_row.addWidget(btn)
            btn_row.addStretch()
            btn_container = QWidget()
            btn_container.setLayout(btn_row)
            v.addWidget(btn_container)
        return box

    def _open_save(self, yaml_path: Path, side_key: str, kind: str, sn: str) -> None:
        dlg = SaveToYamlDialog(self, yaml_path, side_key, kind, sn, self._log)
        if dlg.exec_() == QDialog.Accepted:
            # Refresh "Currently saved" + tags without making the operator hit Rescan.
            self._start_scan()

    def closeEvent(self, event) -> None:
        if self._loading_timer.isActive():
            self._loading_timer.stop()
            QApplication.restoreOverrideCursor()
        super().closeEvent(event)

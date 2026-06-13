
# XRoboToolkit Unity Client APK

The APK ships in this repo via Git LFS. After cloning, run `git lfs install && git lfs pull` and you'll find the binary at `XRoboToolkit-v1.4.apk` next to this README.

## Current version

**v1.4** — uses local coordinate system by default (no longer distinguishes local/global).

## Installation

```bash
# From the repo root, with LFS pulled:
adb install -r -g src/input_devices/pico_input/apk/XRoboToolkit-v1.4.apk
```

Prerequisites:
- PICO headset connected via USB with developer mode + USB debugging enabled.
- `adb` available on the host (Ubuntu: `sudo apt install android-tools-adb`).


# XRoboToolkit Unity Client APK

APK files have been removed from the Git repository to keep it lightweight.

## Download APK

Please download the latest version from GitHub Releases:

**v1.4 (Latest)**
- [v1.4.apk](https://github.com/lzhu686/XRoboToolkit-Unity-Client/releases/download/v1.4/v1.4.apk) — No longer distinguishes local/global; uses local coordinate system by default

**v1.3.0**
- Local coordinate system (Recommended): [v1.3local.apk](https://github.com/lzhu686/XRoboToolkit-Unity-Client/releases/download/v1.3.0/v1.3local.apk)
- Global coordinate system (Special scenarios only): [v1.3global.apk](https://github.com/lzhu686/XRoboToolkit-Unity-Client/releases/download/v1.3.0/v1.3global.apk)

**v1.2.0**
- [v1.2.apk](https://github.com/lzhu686/XRoboToolkit-Unity-Client/releases/download/v1.2.0/v1.2.apk)

## Coordinate System Mode Description

| Mode | Stability | Recommended Use |
|------|-----------|-----------------|
| **Local (Local coordinate system)** | More stable | Recommended for daily use |
| **Global (Global coordinate system)** | May be unstable | Only for multi-device spatial alignment scenarios |

**Local mode advantages:**
- Coordinate system relative to device initial position
- Does not depend on environmental feature points, more stable and reliable tracking
- Suitable for single device use, robotic arm teleoperation, and daily development

**Global mode limitations:**
- Depends on environmental spatial anchors and feature points
- May experience unstable tracking due to lighting changes or environmental occlusion
- Use only when multi-device spatial alignment is needed

## Installation

```bash
# 1. Download APK
wget https://github.com/lzhu686/XRoboToolkit-Unity-Client/releases/download/v1.4/v1.4.apk

# 2. Install to PICO headset
adb install -r -g v1.4.apk
```

## All Release Versions

View all versions: https://github.com/lzhu686/XRoboToolkit-Unity-Client/releases

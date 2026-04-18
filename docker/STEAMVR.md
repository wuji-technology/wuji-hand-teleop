
# SteamVR Teleoperation Solution

HTC Vive Tracker + MANUS Gloves, arm pose tracking via SteamVR.

> **Prerequisites:** Please first complete [README.md](README.md) steps 1-5 (Docker installation, build, startup).

## 1. Install SteamVR on Host

SteamVR runs on the **host machine** (not inside the container); the container accesses it via `.steam` mount.

```bash
# 1. Install Steam (if not already installed)
sudo apt install steam

# 2. Open Steam → Library → Search "SteamVR" → Install

# 3. Verify installation path
ls ~/.steam/debian-installation/steamapps/common/SteamVR/
```

## 2. Configure Null Driver (Headset-Free Mode)

Teleoperation does not require a VR headset; only Vive Trackers are used. Configure the Null Driver to let SteamVR start without a headset. Two files need to be modified:

**File 1 — Enable null driver** (driver default config):

```bash
nano ~/.steam/steam/steamapps/common/SteamVR/drivers/null/resources/settings/default.vrsettings
```

Change `"enable": false` to `"enable": true`:

```json
{
    "driver_null": {
        "enable": true,
        ...
    }
}
```

**File 2 — Force use of null driver** (user config, auto-generated after first SteamVR launch):

```bash
nano ~/.steam/debian-installation/config/steamvr.vrsettings
```

Ensure the `"steamvr"` section contains:

```json
{
    "steamvr": {
        "requireHmd": false,
        "forcedDriver": "null",
        "activateMultipleDrivers": true
    }
}
```

> **Note:** `forcedDriver: "null"` makes SteamVR skip headset detection, and `requireHmd: false` prevents errors when no headset is present. Vive Trackers still track normally via Lighthouse.

## 3. Start SteamVR

```bash
# Wayland desktop (Ubuntu 22.04+ default) requires XWayland parameters
GDK_BACKEND=x11 QT_QPA_PLATFORM=xcb steam steam://rungameid/250820

# X11 desktop can start directly
# steam steam://rungameid/250820
```

Verify null driver is active:

```bash
grep "null" ~/.steam/debian-installation/logs/vrserver.txt | tail -3
# Should see: "Using existing HMD null.Null Serial Number"

# Verify SteamVR process
ps aux | grep vrserver
```

> **Wayland note:** Gnome Wayland does not support the DRM lease required by SteamVR; vrmonitor crashing will cause vrserver to exit as well. **Every launch requires the `GDK_BACKEND=x11 QT_QPA_PLATFORM=xcb` prefix**. Confirm desktop type with `echo $XDG_SESSION_TYPE`.

## 4. HTC Vive Tracker Setup

### Hardware Relationships

```text
Base Station                  Tracker               Dongle             PC
  Emits IR light  ──→  Tracker receives IR to compute its own pose  ──radio──→  Dongle (USB) ──→  SteamVR
```

- **Base Station**: Emits infrared laser; Tracker locates itself by receiving IR signals from the base station. Base stations only need power, no pairing required
- **Dongle**: USB wireless receiver; each Tracker is paired with one Dongle, responsible for transmitting Tracker data back to the PC
- **Tracker**: Must be within the base station IR coverage area for positioning. Communicates with the PC via Dongle

### Hardware Preparation

- 2 Vive Trackers (bound to left and right arms respectively)
- 2 Lighthouse Base Stations (tracking space)
- 2 USB Dongle receivers (one per Tracker)

### Connect Dongle

Plug in the Dongle and verify detection:

```bash
lsusb | grep 28de
# Should see: Valve Software Watchman Dongle
```

### Pair Tracker

1. Ensure base stations are powered on with green indicator light steady
2. SteamVR status window → **≡ Menu** → **Devices** → **Pair Controller**
3. Select **I want to pair a different type of controller** → **Vive Tracker**
4. **Long press the Tracker center button for ~5 seconds**, blue light flashes rapidly → light turns green when pairing is complete
5. Pairing info is saved in the Dongle; subsequent boots connect automatically

### Mount Tracker

Secure the Trackers to both arms with straps:
- **Left arm:** Tracker orientation consistent
- **Right arm:** Tracker orientation consistent
- Both Trackers must face the same direction

### Verify (Inside Container)

```bash
python3 -c "
import openvr
openvr.init(openvr.VRApplication_Other)
vr = openvr.VRSystem()
for i in range(64):
    dc = vr.getTrackedDeviceClass(i)
    if dc != 0:
        serial = vr.getStringTrackedDeviceProperty(i, openvr.Prop_SerialNumber_String)
        names = {1:'HMD', 2:'Controller', 3:'Tracker', 4:'BaseStation'}
        print(f'  [{i}] {names.get(dc,dc)}: {serial}')
openvr.shutdown()
"
```

> **Note:** "Failed to lease display" warnings can be ignored — display output is not needed in null driver mode.
>
> When not using SteamVR, comment out the SteamVR-related volumes mounts in `docker-compose.yml`; this does not affect PICO and MANUS functionality.

## 5. Enter Container and Start Teleoperation

```bash
# 1. Allow container X11 access (required for GUI display)
xhost +local:docker

# 2. Enter container
docker exec -it wuji-teleop bash
```

### Launch Commands

| Launch Method | Description |
|---------------|-------------|
| `ros2 launch wuji_teleop_bringup wuji_teleop_camera.launch.py` | Hand + arm + camera (recommended) |
| `ros2 launch wuji_teleop_bringup wuji_teleop.launch.py` | Hand + arm (no camera) |

### Parameter Description

```bash
# Full parameter example
ros2 launch wuji_teleop_bringup wuji_teleop_camera.launch.py \
    hand_input:=manus \
    arm_input:=tracker \
    enable_camera:=true \
    enable_rviz:=false
```

| Parameter | Default | Description |
|-----------|---------|-------------|
| `hand_input` | `manus` | Hand input: `manus` |
| `arm_input` | `tracker` | Arm input: `tracker` |
| `enable_camera` | `true` | Whether to start cameras |
| `enable_rviz` | `false` | Whether to start RViz visualization |
| `enable_head` | `true` | Head stereo camera (camera launch) |

> **Coordinate system:** The SteamVR solution uses **chest coordinate system IK** (`tianji_output`); Vive Tracker data is converted through SteamVR before input.

## 6. Monitor GUI

A graphical monitoring interface can be launched inside the container:

```bash
# Run on host first (allow X11)
xhost +local:docker

# Launch inside container
ros2 run wuji_teleop_monitor monitor
```

Monitor provides device status monitoring, topic inspection, camera preview, and a dropdown menu for one-click teleoperation launch:

| Preset | Description | Corresponding Launch File |
|--------|-------------|--------------------------|
| Hand only | MANUS Gloves → Wuji Hand | `wuji_teleop_hand.launch.py` |
| Arm only | Vive Tracker → Tianji Arm | `wuji_teleop_arm.launch.py` |
| Hand + Arm | MANUS + Tracker → Full teleoperation | `wuji_teleop.launch.py` |
| Hand + Arm + Camera | Full teleoperation + cameras | `wuji_teleop_camera.launch.py` |
| Single side (Left/Right) | Single arm + single hand debug | `wuji_teleop_single.launch.py` |

## 7. Verification

```bash
# Check all topics
ros2 topic list

# Key topic frequencies
ros2 topic hz /stereo/left/compressed                              # Head stereo (~30fps)
ros2 topic hz /cam_left_wrist/color/image_rect_raw/compressed      # Left wrist D405
ros2 topic hz /cam_right_wrist/color/image_rect_raw/compressed     # Right wrist D405
```

## 8. FAQ

| Problem | Solution |
|---------|----------|
| SteamVR exits immediately after launch | Wayland desktop requires `GDK_BACKEND=x11 QT_QPA_PLATFORM=xcb` prefix when launching |
| SteamVR window not displayed | Run `xhost +local:docker` on host |
| `[--] OpenVR (SteamVR not mounted)` | Confirm SteamVR is installed on host, `ls ~/.steam/debian-installation/steamapps/common/SteamVR/` |
| openvr_input cannot connect to SteamVR | Confirm SteamVR is running on host; container needs `pid: host` (already configured in docker-compose.yml) |
| Vive Tracker not recognized | Confirm Lighthouse base stations are on, Tracker is paired, Dongle is plugged in (`lsusb \| grep 28de`) |
| Tracker light not on | Check if Dongle is plugged in, base station is powered on, Tracker is within base station IR range |
| Null Driver not taking effect | Check both config files: `enable: true` in driver `default.vrsettings`, `forcedDriver: "null"` in user `steamvr.vrsettings` |
| Monitor GUI cannot display | Run `xhost +local:docker` on host to allow X11 access |
| MANUS Gloves no data | Run `git lfs pull` on host to ensure SDK files are complete |

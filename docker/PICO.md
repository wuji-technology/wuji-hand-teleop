
# PICO Teleoperation Solution

PICO VR Headset + MANUS Gloves, arm pose tracking via PICO, head stereo H.264 real-time streaming to VR display.

> **Prerequisites:** Please first complete [README.md](README.md) steps 1-5 (Docker installation, build, startup).

## 1. PICO Preparation

### Enable Developer Mode

1. PICO Headset → Settings → General → Developer Mode → **Enable**
2. Allow USB debugging

### Install XRoboToolkit

XRoboToolkit is the PICO-side teleoperation application. Download the v1.4 APK from [XRoboToolkit-Unity-Client releases](https://github.com/lzhu686/XRoboToolkit-Unity-Client/releases):

```bash
# ADB sideload install (PICO USB connected)
adb install v1.4.apk
```

### USB Connection

Connect the PICO headset to the PC via USB cable:

```bash
# Verify connection on host
adb devices
# Should show: XXXXXXXXXX    device
```

> PICO requires confirming "Allow USB debugging" in the headset on first connection.

## 2. Operation Workflow

### Step 1: Start Container

```bash
cd ~/Desktop/wuji-hand-teleop/docker
docker compose up -d

# Wait for build to complete (about 2 minutes on first run)
docker logs -f wuji-teleop
# Ready when you see "SDK Status:"
```

### Step 2: Confirm ADB Status

```bash
docker exec -it wuji-teleop bash

# Check inside container
adb devices                 # Should show PICO device
adb reverse --list          # Should show two ports:
                            #   (reverse) tcp:63901 tcp:63901
                            #   (reverse) tcp:13579 tcp:13579
```

If `adb reverse --list` is empty, set up manually:

```bash
adb reverse tcp:63901 tcp:63901    # PC-Service control channel
adb reverse tcp:13579 tcp:13579    # Camera command channel
```

> ADB reverse ports are automatically managed by `adb_watchdog` inside the container (checks every 5 seconds). If PICO disconnects and reconnects, the watchdog will automatically restore the ports.

### Step 3: Start PICO Side

1. **PICO Headset:** Open the XRoboToolkit app
2. **PICO Headset:** Press the **Connect** button
3. Wait for the connection success prompt

### Step 4: Start Teleoperation Nodes

```bash
# Inside container
ros2 launch wuji_teleop_bringup pico_teleop.launch.py
```

After launch, a parameter overview and IP information will be displayed. The system automatically:
1. Waits for PICO data (120-second timeout)
2. Auto-initializes upon detecting data
3. Begins incremental control

### Step 5: Verify

```bash
# Enter container from another terminal
docker exec -it wuji-teleop bash

# Check topics
ros2 topic list | grep -E "arm_target|stereo|wrist"
ros2 topic hz /stereo/left/compressed         # Head stereo (~30fps)
```

### Step 6: Enable Stereo Vision

1. **PICO Headset:** Select connection mode in XRoboToolkit:
   - **ADB (USB wired):** Select ADB mode, enter IP address **127.0.0.1**
   - **WiFi (wireless):** Select WiFi mode, enter the PC's LAN IP (e.g., `192.168.1.100`)
2. **PICO Headset:** Press the **Listen** button
3. PC-side logs should show `OPEN_CAMERA` → `MEDIA_DECODER_READY` → `H.264 streaming active`
4. You should see the real-time stereo view from the head cameras in the PICO headset

> If PICO disconnects and reconnects, the system automatically falls back to ROS2-only mode; Connect → Listen again to resume H.264 streaming.

## 3. Launch Commands

| Launch Method | Description |
|---------------|-------------|
| `ros2 launch wuji_teleop_bringup pico_teleop.launch.py` | Full functionality (arm + hand + camera) |
| `ros2 launch wuji_teleop_bringup pico_teleop_minimal.launch.py` | Arm end-effector control only |

### pico_teleop.launch.py Parameters

```bash
# Preview mode (RViz visualization only, no robot control)
ros2 launch wuji_teleop_bringup pico_teleop.launch.py \
    enable_robot:=false enable_camera:=false enable_hand:=false enable_rviz:=true
```

| Parameter | Default | Description |
|-----------|---------|-------------|
| `enable_robot` | `true` | Tianji Arm output |
| `enable_camera` | `true` | Cameras (head stereo + wrist D405) |
| `enable_hand` | `true` | MANUS Glove input + dexterous hand output |
| `enable_rviz` | `false` | RViz visualization |

### pico_teleop_minimal.launch.py Parameters

```bash
# Playback mode (no PICO device needed)
ros2 launch wuji_teleop_bringup pico_teleop_minimal.launch.py \
    data_source_type:=recorded playback_speed:=0.3
```

| Parameter | Default | Description |
|-----------|---------|-------------|
| `data_source_type` | `live` | `live` (real-time PICO) or `recorded` (playback) |
| `playback_speed` | `0.3` | Playback speed (recorded mode only) |
| `loop_playback` | `true` | Loop playback |

> **Coordinate system:** The PICO solution uses **world coordinate system IK** (`tianji_world_output`); PICO tracking data is solved directly in the world coordinate system.

## 4. Head Stereo H.264 Streaming (Stereo Vision)

Exclusive to the PICO solution: head stereo camera images are encoded in real-time H.264 and streamed to the PICO VR headset for display.

### Data Flow

```text
Head stereo camera (USB) → OpenCV MJPEG 60fps
  ├── ROS2: split L/R → JPEG → /stereo/{left,right}/compressed (30fps)
  └── PICO: BGR24 → FFmpeg → H.264 → TCP:12345 → PICO VR display (60fps)
```

### Auto-Negotiation Flow

1. PICO XRoboToolkit sends `OPEN_CAMERA` command via TCP:13579
2. PICO sends `MEDIA_DECODER_READY` (with video port number)
3. PC starts FFmpeg H.264 encoding
4. PC connects to PICO video port (via ADB forward or WiFi direct)
5. H.264 frame stream begins

### Test Steps

Complete stereo vision test workflow:

```bash
# === Host ===

# 1. Confirm PICO USB is connected
adb devices

# === Inside Container ===
docker exec -it wuji-teleop bash

# 2. Confirm ADB reverse ports
adb reverse --list
# Should show:
#   (reverse) tcp:63901 tcp:63901
#   (reverse) tcp:13579 tcp:13579
# If empty: adb reverse tcp:63901 tcp:63901 && adb reverse tcp:13579 tcp:13579

# 3. Start camera node (test mode, camera only)
ros2 launch camera camera_launch.py enable_pico:=true

# 4. PICO Headset: Open XRoboToolkit → Connect → Listen
# Logs should show:
#   PICO client connected: 127.0.0.1:xxxxx
#   OPEN_CAMERA: 2560x720 @ 60fps, 30Mbps
#   MEDIA_DECODER_READY, video port=12345
#   ADB forward tcp:12345 → PICO:12345
#   TCP connected: 127.0.0.1:12345
#   H.264 streaming active
#   PICO H.264: xxx frames | 60.0fps

# 5. Verify ROS2 topics are working simultaneously
ros2 topic hz /stereo/left/compressed      # ~30fps
```

> If PICO disconnects and reconnects, the system automatically falls back to ROS2-only mode; Connect again to resume H.264 streaming.

## 5. ADB Management

### Architecture

PICO wired mode communicates via ADB:

```text
Host USB ← PICO Headset
        ↓
    Docker Container
        ├── adb_watchdog (background daemon, checks every 5s)
        │     └── adb reverse tcp:63901 (PC-Service control)
        │     └── adb reverse tcp:13579 (camera commands)
        ├── PC-Service (port 63901, XRoboToolkit Connect)
        └── unified_stereo (port 13579 commands + dynamic forward video)
              └── adb forward tcp:12345 (H.264 video, dynamically created per stream)
```

### Port Description

| Direction | Port | Purpose | Management |
|-----------|------|---------|------------|
| `reverse` | 63901 | PC-Service control channel | watchdog automatic |
| `reverse` | 13579 | Camera command channel (XRobo protocol) | watchdog automatic |
| `forward` | 12345 | H.264 video stream (PC→PICO) | Dynamically created per stream |

- **reverse** (PICO→PC): Persistent, session-level, lost on disconnect, watchdog auto-recovers
- **forward** (PC→PICO): Dynamically created per video connection, port number specified by PICO `MEDIA_DECODER_READY`

### Manual Diagnostics

```bash
# Check PICO connection
adb devices

# Check reverse port mappings
adb reverse --list

# Manual setup (when watchdog fails)
adb reverse tcp:63901 tcp:63901
adb reverse tcp:13579 tcp:13579

# Check forward port mappings
adb forward --list

# Check watchdog logs
docker logs wuji-teleop 2>&1 | grep "ADB watchdog"
```

### ADB Troubleshooting

| Problem | Solution |
|---------|----------|
| `adb devices` shows no device | Is PICO developer mode enabled? Is it a data cable? Confirm USB debugging authorization in the headset |
| `adb reverse --list` is empty | Wait 5 seconds (watchdog cycle), or manually `adb reverse tcp:63901 tcp:63901` |
| `unauthorized` device | Tap "Allow USB debugging" in the PICO headset |
| Cannot connect after PICO unplug/replug | `adb kill-server && sudo adb start-server`, wait for watchdog to recover |

## 6. WiFi Mode (No USB Cable)

PICO also supports WiFi wireless connection (no ADB needed):

1. Connect PC and PICO to the same LAN
2. Configure the PC's IP address in PICO XRoboToolkit
3. When launching the camera node, the system auto-detects no ADB device and uses WiFi direct connection

> In WiFi mode, all ADB-related steps are skipped. Video stream connects directly via LAN TCP, no ADB forward needed.
>
> WiFi has higher latency (RTT ~5-15ms vs USB ~1ms); wired mode is recommended.

## 7. FAQ

| Problem | Solution |
|---------|----------|
| PICO Connect fails | Confirm `adb reverse --list` shows port 63901 |
| H.264 no image | Confirm `adb reverse --list` shows 13579, check container logs for `OPEN_CAMERA` |
| Video stream interrupted | System auto-falls back to ROS2-only; reconnect PICO to resume |
| `TCP connect failed` | ADB forward port not established, check USB connection |
| NVENC encoding failure | Auto-falls back to libx264 when container has no GPU, no impact on functionality |
| MANUS Gloves no data | Run `git lfs pull` on host to ensure SDK files are complete |
| `pico_input` initialization timeout | Is PICO XRoboToolkit connected? Check PC-Service logs |

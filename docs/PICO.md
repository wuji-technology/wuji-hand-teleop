
# PICO Teleoperation Solution

PICO 4 / 4 Ultra VR Headset + 4 PICO Motion Trackers for arm pose tracking + Wuji Glove (default; MANUS supported via `wujihand_ik.yaml::input_source`), head stereo H.264 real-time streaming to VR display.

This is the single entry-point user guide. Architecture, tracker math, and package internals live in the package READMEs linked at the bottom.

## Prerequisites

- Complete [README.md](README.md) steps 1-5 (Docker installation, build, startup).
- **PICO OS >= 5.14.** Streaming all 4 Motion Trackers requires PICO OS 5.14 or later; earlier versions (e.g. 5.11.x) silently cap at 2 trackers and `pico_input_node` will report fewer trackers than expected. Verify via headset Settings -> About this device -> Software version. Upgrade through Settings -> About this device -> System update before going further.
- PICO 4 Motion Trackers paired with the headset (use the PICO Motion Tracker app on the headset itself; one-time pairing).
- USB-C data cable (PICO headset to PC). WiFi mode is supported but wired is recommended for latency.

## 1. PICO Preparation

### Enable Developer Mode

PICO 4 / 4 Ultra do not expose Developer Mode under General Settings. The toggle is unlocked by tapping the software version number:

1. Headset Settings -> **About this device**.
2. Tap **Software version** repeatedly (~7 times) until a toast confirms developer mode is unlocked.
3. A new **Developer** entry appears in the left navigation. Open it and enable **USB debugging**.
4. Some firmware revisions additionally require binding a PICO developer account before the toggle becomes writable; sign in via Settings -> Account if the USB debugging switch is greyed out.

### Install XRoboToolkit

XRoboToolkit is the PICO-side teleoperation application. The v1.4 APK is vendored via Git LFS at `src/input_devices/pico_input/apk/XRoboToolkit-v1.4.apk`. After `git lfs pull`, sideload it:

```bash
# From the repo root, with PICO USB connected and USB debugging enabled
adb install -r -g src/input_devices/pico_input/apk/XRoboToolkit-v1.4.apk
```

### USB Connection

Connect the PICO headset to the PC via USB cable:

```bash
# Verify connection on host
adb devices
# Should show: XXXXXXXXXX    device
```

> PICO requires confirming "Allow USB debugging" in the headset on first connection.

### Configure tracker serial numbers

Each PICO Motion Tracker has a unique SN. The four slots in `src/input_devices/pico_input/config/pico_input.yaml` (seeded from `pico_input.yaml.template` on first container start — see README §5) wire each physical tracker to its role on the body:

```yaml
tracker_sn_left_wrist:  "PC2310MLKC190058G"   # back of left hand
tracker_sn_right_wrist: "PC2310MLKC190600G"   # back of right hand
tracker_sn_left_arm:    "PC2310MLKC190046G"   # outer left upper arm
tracker_sn_right_arm:   "PC2310MLKC190023G"   # outer right upper arm
```

Use the full SN string the way the headset prints it — no digit-counting. Find each SN in the XRoboToolkit PC Service tab once the trackers are streaming, or in `pico_input_node`'s boot log when a tracker first arrives. An empty placeholder or a duplicate makes the node refuse to start with a `RuntimeError` pointing at the offending slot.

## 2. Operation Workflow

### Step 1: Start Container

```bash
cd ~/ros2_ws/src/wuji-hand-teleop/docker
docker compose up -d

# Wait for build to complete (about 2 minutes on first run)
docker logs -f wuji-hand-teleop
# Ready when you see "SDK Status:"
```

### Step 2: Confirm ADB Status

```bash
docker exec -it wuji-hand-teleop bash

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

### Step 3: Connect XRoboToolkit on the headset

Sideloading the APK and pressing Connect alone is not enough — tracker data only starts flowing once the PC Service link is up *and* you push trackers from the Motion Tracker tab. Do all three:

1. **PICO Headset:** open the XRoboToolkit app and press **Connect**. Wait for the connection success prompt.
2. **PICO Headset:** open the **PC Service** tab, enter host **`127.0.0.1`** (USB / ADB reverse mode; use the PC LAN IP in WiFi mode), and confirm status reads **WORKING**.
3. **PICO Headset:** open the **Pico Motion Tracker** tab, select **Full body** mode, then press **Send**. Tracker data now streams to the PC.

If you skip step 3, `pico_input_node` will sit at `Waiting for PICO device connection` forever even though Connect succeeded — the SDK reads 0 trackers until Send is pressed.

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
docker exec -it wuji-hand-teleop bash

# Check topics
ros2 topic list | grep -E "arm_target|stereo|wrist"
ros2 topic hz /stereo/left/compressed         # Head stereo (~30fps)
```

### Step 6: Enable Stereo Vision

1. **PICO Headset:** Select connection mode in XRoboToolkit:
   - **ADB (USB wired):** Select ADB mode, enter IP address **127.0.0.1**
   - **WiFi (wireless):** Select WiFi mode, enter the PC's LAN IP (e.g., `192.168.1.100`)
2. **PICO Headset:** Press the **Listen** button
3. PC-side logs should show `OPEN_CAMERA` -> `MEDIA_DECODER_READY` -> `H.264 streaming active`
4. You should see the real-time stereo view from the head cameras in the PICO headset

> If PICO disconnects and reconnects, the system automatically falls back to ROS2-only mode; Connect -> Listen again to resume H.264 streaming.

## 3. Launch Commands

| Launch Method | Description |
|---------------|-------------|
| `ros2 launch wuji_teleop_bringup pico_teleop.launch.py` | Full PICO pipeline (cameras + Wuji Hand + Tianji Arm). |
| `ros2 launch wuji_teleop_bringup pico_teleop.launch.py enable_hand:=false` | Arm only (skip hand controllers). |
| `ros2 launch wuji_teleop_bringup pico_teleop.launch.py enable_camera:=false` | Skip cameras + H.264 stream (CI / no headset). |

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
| `enable_hand` | `true` | Hand input + dexterous hand output (hand source via `wujihand_ik.yaml::input_source`) |
| `enable_rviz` | `false` | RViz visualization |

> **Coordinate system:** The PICO solution uses **world coordinate system IK** (`tianji_world_output`); PICO tracking data is solved directly in the world coordinate system.

## 4. Head Stereo H.264 Streaming (Stereo Vision)

Exclusive to the PICO solution: head stereo camera images are encoded in real-time H.264 and streamed to the PICO VR headset for display.

### Data Flow

```text
Head stereo camera (USB) -> OpenCV MJPEG 60fps
  |-- ROS2: split L/R -> JPEG -> /stereo/{left,right}/compressed (30fps)
  +-- PICO: BGR24 -> FFmpeg -> H.264 -> TCP:12345 -> PICO VR display (60fps)
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
docker exec -it wuji-hand-teleop bash

# 2. Confirm ADB reverse ports
adb reverse --list
# Should show:
#   (reverse) tcp:63901 tcp:63901
#   (reverse) tcp:13579 tcp:13579
# If empty: adb reverse tcp:63901 tcp:63901 && adb reverse tcp:13579 tcp:13579

# 3. Start camera node (test mode, camera only)
ros2 launch camera camera_launch.py enable_pico:=true

# 4. PICO Headset: Open XRoboToolkit -> Connect -> Listen
# Logs should show:
#   PICO client connected: 127.0.0.1:xxxxx
#   OPEN_CAMERA: 2560x720 @ 60fps, 30Mbps
#   MEDIA_DECODER_READY, video port=12345
#   ADB forward tcp:12345 -> PICO:12345
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
Host USB <- PICO Headset
        |
    Docker Container
        |-- adb_watchdog (background daemon, checks every 5s)
        |     |-- adb reverse tcp:63901 (PC-Service control)
        |     +-- adb reverse tcp:13579 (camera commands)
        |-- PC-Service (port 63901, XRoboToolkit Connect)
        +-- unified_stereo (port 13579 commands + dynamic forward video)
              +-- adb forward tcp:12345 (H.264 video, dynamically created per stream)
```

### Port Description

| Direction | Port | Purpose | Management |
|-----------|------|---------|------------|
| `reverse` | 63901 | PC-Service control channel | watchdog automatic |
| `reverse` | 13579 | Camera command channel (XRobo protocol) | watchdog automatic |
| `forward` | 12345 | H.264 video stream (PC->PICO) | Dynamically created per stream |

- **reverse** (PICO->PC): Persistent, session-level, lost on disconnect, watchdog auto-recovers
- **forward** (PC->PICO): Dynamically created per video connection, port number specified by PICO `MEDIA_DECODER_READY`

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
docker logs wuji-hand-teleop 2>&1 | grep "ADB watchdog"
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

1. Connect PC and PICO to the same LAN.
2. In the XRoboToolkit **PC Service** tab, enter the PC's LAN IP (e.g. `192.168.1.100`) instead of `127.0.0.1`; status should read **WORKING**.
3. In the **Pico Motion Tracker** tab, select **Full body** and press **Send** as in the USB workflow.
4. When launching the camera node, the system auto-detects no ADB device and uses WiFi direct connection.

> In WiFi mode, all ADB-related steps are skipped. Video stream connects directly via LAN TCP, no ADB forward needed.
>
> WiFi has higher latency (RTT ~5-15ms vs USB ~1ms); wired mode is recommended.

## 7. FAQ

| Problem | Solution |
|---------|----------|
| Fewer than 4 trackers in PICO Motion Tracker tab | Upgrade PICO OS to >= 5.14 (5.11.x caps at 2 trackers) |
| `Cannot find role mapping for tracker PCxxxx...` log spam | Paste the full SN string into the matching `tracker_sn_<slot>` line in `pico_input.yaml`. See [Configure tracker serial numbers](#configure-tracker-serial-numbers). |
| `pico_input_node` stuck on `Waiting for PICO device connection` | XRoboToolkit Connect succeeded but tracker stream not pushed; on the headset go to Pico Motion Tracker -> Full body -> **Send** |
| PC Service status is not `WORKING` | Check the host field is `127.0.0.1` (USB) or the PC LAN IP (WiFi); for USB also verify `adb reverse --list` shows port 63901 |
| Developer Mode toggle missing | Tap Settings -> About this device -> Software version ~7 times; some firmware also requires a PICO developer account |
| PICO Connect fails | Confirm `adb reverse --list` shows port 63901 |
| H.264 no image | Confirm `adb reverse --list` shows 13579, check container logs for `OPEN_CAMERA` |
| Video stream interrupted | System auto-falls back to ROS2-only; reconnect PICO to resume |
| `TCP connect failed` | ADB forward port not established, check USB connection |
| NVENC encoding failure | Auto-falls back to libx264 when container has no GPU, no impact on functionality |
| MANUS Gloves no data | Run `git lfs pull` on host to ensure SDK files are complete |
| `pico_input` initialization timeout | Is PICO XRoboToolkit connected and Send pressed? Check PC-Service logs |

## Further reading

This guide is the single user-facing entry point for the PICO solution. Deeper material lives in the package READMEs:

- [`src/input_devices/pico_input/README.md`](../src/input_devices/pico_input/README.md) — package internals: tracker role table and wearing positions, incremental control, PC-Service `.deb` install on the host (non-Docker), `pico_input.yaml` reference, output topics / TF tree, build & rebuild rules.
- [`src/input_devices/pico_input/ARCHITECTURE.md`](../src/input_devices/pico_input/ARCHITECTURE.md) — coordinate systems (PICO OpenXR -> world -> chest), incremental control derivation, arm-angle geometry, and the full mathematical transform chain. Read this if you're modifying `incremental_controller.py` / `xrobotoolkit_client.py` or chasing a coordinate-frame bug.

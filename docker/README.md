
# Wuji Teleop Docker

Tianji Arm + MANUS Gloves + Dexterous Hand + Stereo/Wrist Cameras, one-click Docker deployment.

Two teleoperation solutions are supported:

| Solution | Input Device | Guide |
|----------|-------------|-------|
| **SteamVR** | HTC Vive Tracker + MANUS Gloves | [STEAMVR.md](STEAMVR.md) |
| **PICO** | PICO VR Headset + MANUS Gloves | [PICO.md](PICO.md) |

## 1. Install Docker

```bash
sudo apt-get update
sudo apt-get install -y docker.io docker-compose-plugin git-lfs
sudo usermod -aG docker $USER
newgrp docker
```

## 2. Clone Repository

```bash
cd ~/Desktop
git clone --recurse-submodules https://github.com/wuji-technology/wuji-hand-teleop.git
cd wuji-hand-teleop

# Pull MANUS SDK large files (~250MB)
git lfs install
git lfs pull
```

## 3. Build Image

```bash
cd docker
docker compose build

# Accelerated build for China mainland
docker compose build --build-arg PIP_INDEX_URL=https://pypi.tuna.tsinghua.edu.cn/simple
```

The image only contains the runtime environment (ROS2 + drivers + Python dependencies), not the application code.

## 4. Start Container

```bash
docker compose up -d
```

The first startup automatically runs `colcon build` (about 2 minutes); subsequent startups are ready in seconds.

## 5. Enter Container

```bash
docker exec -it wuji-teleop bash
```

After entering, the SDK status will be displayed:

```text
SDK Status:
  [OK] PICO SDK
  [OK] PICO PC-Service
  [OK] MANUS SDK
  [OK] Tianji SDK
  [OK] Wuji Hand SDK
  [OK] RealSense Driver
```

## 6. Choose a Solution

After completing steps 1-5, choose the corresponding guide based on your input device:

- **HTC SteamVR Solution** → [STEAMVR.md](STEAMVR.md)
- **PICO VR Solution** → [PICO.md](PICO.md)

### All Launch Files

| Launch File | Head Stereo | Wrist D405 | Description |
|------------|:--------:|:---------:|------|
| `wuji_teleop.launch.py` | — | — | Dual arm + dual hand full teleoperation (SteamVR/MANUS) |
| `wuji_teleop_hand.launch.py` | — | — | Hand control only |
| `wuji_teleop_arm.launch.py` | — | — | Arm control only |
| `wuji_teleop_camera.launch.py` | ✅ | ✅ | Full teleoperation + all cameras |
| `wuji_teleop_single.launch.py` | — | — | Single-side debug (`side:=left` or `right`) |
| `pico_teleop.launch.py` | ✅ | ✅ | PICO VR full teleoperation (head + wrist + MANUS dexterous hand) |
| `pico_teleop_minimal.launch.py` | — | — | PICO minimal test (supports recorded data playback) |
| `camera_launch.py` (camera package) | ✅ | ✅ | Launch cameras standalone |

### MANUS Gloves Controlling Dexterous Hand (Standalone Test)

Plug in the MANUS Glove USB Dongle, then start inside the container:

```bash
# Dual hand control
ros2 launch wuji_teleop_bringup wuji_teleop_hand.launch.py hand_input:=manus

# Single-side debug (right hand only / left hand only)
ros2 launch wuji_teleop_bringup wuji_teleop_hand.launch.py hand_input:=manus \
    left_serial:=NONE left_hand_name:=disabled_left
ros2 launch wuji_teleop_bringup wuji_teleop_hand.launch.py hand_input:=manus \
    right_serial:=NONE right_hand_name:=disabled_right
```

Verify data flow:

```bash
# 1. Check MANUS Glove topics (C++ SDK → ROS2)
ros2 topic hz /manus_glove_0    # should be ~200Hz
ros2 topic hz /manus_glove_1

# 2. Check hand input topics (MANUS → MediaPipe 21-point)
ros2 topic hz /hand_input       # should be ~50Hz
ros2 topic echo /hand_input --once | head -20

# 3. Check dexterous hand command topics (retarget → driver)
ros2 topic hz /left_hand/joint_commands   # should be ~50Hz
ros2 topic hz /right_hand/joint_commands

# 4. Check dexterous hand state feedback (driver → encoder)
ros2 topic hz /left_hand/joint_states     # should be ~1000Hz
ros2 topic hz /right_hand/joint_states

# 5. Switch TELEOP/INFERENCE mode
ros2 service call /wuji_hand/switch_mode std_srvs/srv/SetBool "{data: true}"   # INFERENCE
ros2 service call /wuji_hand/switch_mode std_srvs/srv/SetBool "{data: false}"  # TELEOP
ros2 service call /wuji_hand/get_mode std_srvs/srv/Trigger {}
```

> **Connecting a new glove**: See the glove integration protocol description in `src/output_devices/wujihand_output/config/wujihand_ik.yaml`

## 7. Daily Usage

```bash
cd docker

docker compose up -d                    # Start
docker exec -it wuji-teleop bash        # Enter
docker compose stop                     # Stop (preserves build artifacts)
docker compose start                    # Resume (no rebuild needed)
docker compose down                     # Destroy (next startup requires colcon build again)
```

The host `src/` directory is directly mounted into the container. After modifying code, run inside the container:

```bash
colcon build --symlink-install
```

> **After PC reboot:** `cd docker && docker compose up -d`, wait for ready then enter the container to start teleoperation.
>
> **Optional auto-start on boot:** Add `restart: unless-stopped` to `docker-compose.yml`, and the container will automatically recover after PC reboot:
> ```yaml
> services:
>   teleop:
>     restart: unless-stopped   # Add this line
> ```
> Not recommended during rapid development phase; after `docker compose down` you need to run `colcon build` again.

## 8. Multi-Machine Data Collection

The container uses CycloneDDS multicast. On another machine in the same LAN:

```bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 topic list
```

## 9. Camera Configuration

### Head Stereo Camera

The head stereo video stream is processed by the **unified_stereo** node (single process, no v4l2loopback required):

```text
Head stereo camera (USB, /dev/stereo_camera) → OpenCV MJPEG 60fps
  ├── ROS2: split L/R → JPEG → /stereo/{left,right}/compressed (30fps)
  └── PICO: BGR24 → FFmpeg → H.264 → TCP → PICO VR (60fps, on-demand)
```

### Wrist RealSense D405

D405 connects via USB 3.2, **left/right wrist are bound by serial number**.

```bash
# Verify D405 is connected
lsusb | grep Intel    # should see Intel RealSense

# Check serial numbers
rs-enumerate-devices --compact
# Example output:
#   Intel RealSense D405    <LEFT_SERIAL>     5.15.1.55
#   Intel RealSense D405    <RIGHT_SERIAL>    5.15.1.55

# Launch wrist cameras standalone
ros2 launch camera camera_launch.py
```

When replacing a D405, modify `src/camera/config/camera_config.yaml`:

```yaml
cameras:
  left_wrist:
    serial_number: "YOUR_LEFT_WRIST_CAM_SERIAL"    # ← Change to actual serial number
  right_wrist:
    serial_number: "YOUR_RIGHT_WRIST_CAM_SERIAL"  # ← Change to actual serial number
```

> If the left and right wrist images are swapped, just swap the two serial numbers.

### udev Rules

Fix camera device paths to prevent device number drift with multiple cameras:

```bash
sudo cp src/camera/config/udev/99-teleop-cameras.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### ROS2 Topics

| Topic | Description |
|-------|-------------|
| `/cam_left_wrist/color/image_rect_raw/compressed` | Left wrist D405 |
| `/cam_right_wrist/color/image_rect_raw/compressed` | Right wrist D405 |
| `/stereo/left/compressed` | Head stereo left eye |
| `/stereo/right/compressed` | Head stereo right eye |

> D405 only has `image_rect_raw` (no `image_raw`), so topics consistently use `image_rect_raw`.

## 10. FAQ

| Problem | Solution |
|---------|----------|
| `docker compose` not found | `sudo apt-get install docker-compose-plugin` |
| `permission denied` Docker | `sudo usermod -aG docker $USER && newgrp docker` |
| manus_ros2 link failure | Run `git lfs pull` on host; MANUS is auto-skipped if not pulled |
| Need to rebuild after `down` | Use `stop/start` instead of `down/up` |
| Monitor GUI cannot display | Run `xhost +local:docker` on host to allow X11 access |
| D405 wrist camera not recognized | Confirm USB 3.2 port, check connection with `lsusb | grep Intel` |
| NVENC encoding failure | Auto-falls back to libx264 when container has no GPU access; see GPU notes below |

> **GPU Acceleration (Optional):** Enable NVENC hardware encoding when an NVIDIA GPU is available:
> 1. Install `nvidia-container-toolkit` on the host
> 2. Uncomment the `deploy.resources` section in `docker-compose.yml`
> 3. Recreate the container: `docker compose up -d`
>
> When no NVIDIA GPU is available, libx264 software encoding is used automatically with no impact on functionality.

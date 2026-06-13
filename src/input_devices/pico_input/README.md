# PICO Input — VR teleoperation input

ROS2 package that reads PICO VR tracking data (1 headset + 4 trackers) from the XRoboToolkit PC-Service over a localhost shared-memory channel, applies **incremental control**, and publishes TF + target-pose / elbow-direction topics for the Tianji arm controllers.

> PICO 4 / 4 Ultra + 4 Motion Trackers is the **VR alternative** for arm input. The default arm input on the open-source pipeline is the HTC Vive Tracker — see `src/input_devices/openvr_input/`. This README assumes you have chosen the PICO path (bring-up via `ros2 launch wuji_teleop_bringup pico_teleop.launch.py`).

---

## Quick start

```bash
# Terminal 1 — start PC-Service (keep running)
/opt/apps/roboticsservice/runService.sh

# Terminal 2 — pick one mode
# (A) PICO TF only, no robot, no hand, no camera; with RViz
ros2 launch wuji_teleop_bringup pico_teleop.launch.py \
    enable_robot:=false enable_camera:=false enable_hand:=false enable_rviz:=true

# (B) Arms only (no dexterous hand, no camera)
ros2 launch wuji_teleop_bringup pico_teleop.launch.py enable_hand:=false enable_camera:=false

# (C) Full teleop: dual arms + Wuji hands + stereo camera
ros2 launch wuji_teleop_bringup pico_teleop.launch.py

# (D) Playback from a recorded session (no PICO hardware needed)
ros2 launch wuji_teleop_bringup pico_teleop.launch.py \
    data_source_type:=recorded playback_speed:=0.3
```

Recommended bring-up order on a fresh machine: **(A) → (B) → (C)**.

### `pico_teleop.launch.py` parameters

| Parameter | Default | Effect |
|---|---|---|
| `enable_robot` | `true` | Send IK targets to the Tianji arm controllers |
| `enable_hand` | `true` | Enable the Wuji-hand path (MANUS / Wuji glove input + retargeting) |
| `enable_camera` | `true` | Bring up stereo head camera + RealSense wrist cameras |
| `enable_rviz` | `false` | Open RViz with the PICO TF tree pre-loaded |
| `data_source_type` | `live` | `live` reads from PC-Service; `recorded` plays back a file |
| `playback_speed` | `1.0` | Speed multiplier when `data_source_type:=recorded` |

---

## How it works

**Incremental control.** The robot does *not* mirror your absolute hand position — that would teleport the end-effector the moment you start. Instead, at init time the node records the user's pose and the robot's pose, then maps **deltas**:

```
robot_target_position    = robot_init_position    + (user_current_position    − user_init_position)
robot_target_orientation = robot_init_orientation × user_orientation_increment
```

Result: the user's hand can be anywhere in space at init; the robot only mirrors *change*, so motion is smooth and bounded.

### Role of each tracker

| Tracker | Worn on | TF frame | What it controls |
|---|---|---|---|
| HMD | Head | `head` | RViz visualisation only (does not drive the robot) |
| `#0` | Left wrist (back of hand) | `pico_left_wrist` | Left end-effector **pose** (position + orientation) |
| `#1` | Right wrist (back of hand) | `pico_right_wrist` | Right end-effector **pose** |
| `#2` | Left upper arm (outer, ~10–15 cm above elbow) | `pico_left_arm` | Left arm-angle constraint (elbow direction) — **position only** |
| `#3` | Right upper arm (outer, ~10–15 cm above elbow) | `pico_right_arm` | Right arm-angle constraint — **position only** |

The wrist tracker determines *where* and *how* the hand points. The upper-arm tracker resolves the 7-DoF arm's redundancy by telling IK which way the elbow should point (a 7-axis arm can reach the same end-effector pose with many different elbow positions).

### Initialisation

1. Robot reaches `init_joints` (configured in `tianji_robot.yaml`).
2. User stands facing the robot, puts on PICO, and assumes a roughly similar pose.
3. After `auto_init_delay` seconds (default 5; set `0` to require manual trigger), the node snapshots: HMD pose, wrist tracker poses, upper-arm tracker positions.
4. From that frame on, every PICO update produces a delta-driven target.

---

## Installation

Sources live at `src/input_devices/pico_input/`. The package has two prerequisites that are **not** part of normal `apt` / `pip`:

- **PC-Service** — the C++ daemon that talks to the PICO headset over WiFi (TCP/63901) and exposes data on a local socket (60061). Ships as a `.deb` at `docker/prebuilt/XRoboToolkit_PC_Service_*.deb`.
- **`xrobotoolkit_sdk`** — the Python binding (Pybind11) the ROS2 node imports. Two install paths:

```bash
cd ~/ros2_ws/src/wuji-hand-teleop/src/input_devices/pico_input

# (A) Recommended — use the prebuilt .so shipped under prebuilt/
./install_sdk.sh

# (B) Rebuild from the vendored sources under vendor/
./install_sdk.sh --build
```

`install_sdk.sh` copies `xrobotoolkit_sdk.cpython-310-x86_64-linux-gnu.so` and `libPXREARobotSDK.so` into `~/.local/`, adds `LD_LIBRARY_PATH=$HOME/.local/lib` to `~/.bashrc`, and installs `numpy` + `scipy`.

> Inside the supplied Docker image (`docker/Dockerfile`), the PC-Service `.deb` is installed during image build and `xrobotoolkit_sdk` is already on `PYTHONPATH` (see Dockerfile sections 3 and `ENV PYTHONPATH=...`). Running outside Docker requires the steps above.

### Installing the PC-Service `.deb` manually (host install)

```bash
cd ~/ros2_ws/src/wuji-hand-teleop
git lfs install && git lfs pull                                       # pull the .deb out of LFS
sudo dpkg -i docker/prebuilt/XRoboToolkit_PC_Service_*.deb
/opt/apps/roboticsservice/runService.sh                               # smoke test — should print "release mode"
```

> The shipped `.deb` is upstream v1.0.0 (no Wuji patches in the binary). The Wuji patches listed in `vendor/README.md` are applied to the vendored source; if you need them in the binary, rebuild with `vendor/build_pc_service.sh` (downloads upstream Redistributable + builds against system Qt6 + repackages the `.deb`).

### Network

PICO headset and PC must be on the same LAN (5 GHz WiFi strongly recommended).

```bash
hostname -I                  # note the PC IP, e.g. 192.168.1.100
# On the PICO: open XRoboToolkit, enter the PC IP, tap Connect.
```

### Tracker serial numbers

Four fixed slots in `config/pico_input.yaml` map each physical tracker to its body location (the live yaml is seeded from `pico_input.yaml.template` on first container start). Paste the full SN string the headset shows:

```yaml
tracker_sn_left_wrist:  "PC2310MLKC190058G"   # back of left hand
tracker_sn_right_wrist: "PC2310MLKC190600G"   # back of right hand
tracker_sn_left_arm:    "PC2310MLKC190046G"   # outer left upper arm
tracker_sn_right_arm:   "PC2310MLKC190023G"   # outer right upper arm
```

Read the SNs in the XRoboToolkit PC Service tab after pairing, or in `pico_input_node`'s boot log when a tracker first streams. An empty placeholder or a duplicate makes the node refuse to start with a `RuntimeError` naming the slot.

---

## Configuration

`config/pico_input.yaml`:

| Key | Default | Meaning |
|---|---|---|
| `publish_rate` | `90.0` | TF + topic publish frequency (Hz) |
| `pc_service_host` | `127.0.0.1` | PC-Service host |
| `pc_service_port` | `60061` | PC-Service local socket |
| `enable_topic_publishing` | `true` | Publish `/left_arm_target_pose`, `/right_arm_target_pose`, `/*_elbow_direction` |
| `enable_legacy_topics` | `false` | Publish `/pico/*` debug topics |
| `auto_init_delay` | `5.0` | Auto-init delay in seconds; `0` disables (requires manual trigger) |
| `pos_ema_alpha` | `0.6` | Position EMA smoothing (0..1, higher = less smoothing) |
| `elbow_dir_ema_alpha` | `1.0` | Arm-angle direction smoothing |
| `elbow_gray_zone` | `0.015` | Arm-angle dead-zone (m) — small upper-arm tracker jitter is ignored |
| `data_source_type` | `live` | `live` or `recorded` |
| `recorded_file_path` | (see YAML) | Path to a recorded session file |
| `playback_speed` | `1.0` | Replay speed multiplier |
| `loop_playback` | `true` | Loop the recorded file |
| `tracker_serial_<SN>` | (set per device) | Maps a serial to one of `pico_left_wrist` / `pico_right_wrist` / `pico_left_arm` / `pico_right_arm` |

### When to rebuild with `colcon`

| Change | Need `colcon build`? |
|---|---|
| Python files (`*.py`) | No (symlink install) |
| `setup.py`, `package.xml`, launch files | Yes |
| `.yaml` configs | Yes (or pass full path on the command line) |
| Add / delete files | Yes |

```bash
colcon build --packages-select pico_input && source install/setup.bash
```

---

## Output

### TF tree

```
world
├── head                 (HMD pose, dynamic)
├── world_left           (static: shoulder/chest frame, +90° around X)
│   ├── pico_left_wrist  (Tracker #0, dynamic)
│   └── pico_left_arm    (Tracker #2, dynamic — position only)
└── world_right          (static: shoulder/chest frame, −90° around X)
    ├── pico_right_wrist (Tracker #1, dynamic)
    └── pico_right_arm   (Tracker #3, dynamic — position only)
```

### Topics (when `enable_topic_publishing=true`)

| Topic | Type | Meaning |
|---|---|---|
| `/pico_hmd` | `PoseStamped` | HMD pose in `world` |
| `/pico_left_wrist`, `/pico_right_wrist` | `PoseStamped` | Wrist tracker poses in the respective shoulder/chest frame |
| `/left_arm_target_pose`, `/right_arm_target_pose` | `PoseStamped` | Incremental-control targets consumed by `tianji_world_output` |
| `/left_arm_elbow_direction`, `/right_arm_elbow_direction` | `Vector3Stamped` | Arm-angle constraint vectors |

---

## Coordinate systems

The node converts PICO's OpenXR frame to the Tianji robot frame using a fixed orthogonal transform (`det = +1`, no mirroring):

```
Tianji robot (right-handed)        PICO / OpenXR (right-handed)
    Z (up)                              Y (up)
    │                                   │
    └── Y (left)                        └── X (right)
   ╱                                   ╱
  X (forward, robot-facing)           Z (forward, user-facing)
```

Position mapping:

```
robot_x = −pico_z          (forward)
robot_y = −pico_x          (left)
robot_z =  pico_y          (up)
```

Rotation uses the axis-angle method: rotate the axis vector by the matrix above, keep the angle. User rotations therefore map intuitively (tilt-right ↔ tilt-right, twist-wrist ↔ twist-wrist) once the user is facing the robot.

---

## Robot initial pose (FK of `init_joints`)

Tianji joint angles (degrees) configured in `tianji_robot.yaml`:

```python
INIT_JOINTS_LEFT  = [ 55.0, -65.0, -70.0, -60.0,  60.0, 0.0, 0.0]
INIT_JOINTS_RIGHT = [-55.0, -65.0,  70.0, -60.0, -60.0, 0.0, 0.0]
```

### End-effector (wrist) pose

| Arm | Position `[x, y, z]` (m) | Orientation `[qx, qy, qz, qw]` |
|---|---|---|
| Left | `[0.5733, 0.2237, 0.2762]` | `[0.0067, 0.7270, 0.0111, 0.6865]` |
| Right | `[0.5733, −0.2237, 0.2762]` | `[−0.0067, 0.7270, −0.0111, 0.6865]` |

### Upper-arm tracker reference (chest frame)

| Arm | Position `[x, y, z]` (m) | Orientation `[qx, qy, qz, qw]` |
|---|---|---|
| Left | `[0.2, 0.3, 0.2]` | `[0.4177, −0.0283, 0.5206, 0.7441]` |
| Right | `[0.2, −0.3, 0.2]` | `[0.7448, −0.5141, 0.0058, 0.4254]` |

Roughly: elbow 30 cm below + 20 cm outward from shoulder (sunken-elbow / abducted posture).

### Recomputing after changing `init_joints`

```bash
cd src/output_devices/tianji_world_output/tianji_world_output
python3 get_init_pose.py
# Paste the printed init_pos / init_rot / init_quat into tianji_robot.yaml.
```

---

## Troubleshooting

### PC-Service

```bash
ls -la /opt/apps/roboticsservice/runService.sh          # is the .deb installed?
/opt/apps/roboticsservice/runService.sh                  # should print "release mode"
ss -tlnp | grep 60061                                    # is the local socket up?
```

### PICO link

```bash
ping <PICO_IP>                                           # PICO IP shown in the XRoboToolkit app
LD_LIBRARY_PATH=$HOME/.local/lib /usr/bin/python3 -c "
import xrobotoolkit_sdk as xrt
xrt.init()
print('trackers:', xrt.num_motion_data_available())
print('hmd:', xrt.get_headset_pose())
"
```

### ROS2

```bash
ros2 node list
ros2 topic list
ros2 topic echo /tf --once
ros2 run tf2_tools view_frames && evince frames.pdf
```

### Common errors

| Symptom | Likely cause | Fix |
|---|---|---|
| `No module named 'xrobotoolkit_sdk'` | Pybind SDK not installed on the host | Run `./install_sdk.sh` |
| `cannot find -lPXREARobotSDK` | C++ shared lib missing | Re-run `./install_sdk.sh` (it copies `libPXREARobotSDK.so` into `~/.local/lib`) |
| `Found zero norm quaternions` | PICO not yet sending data | Confirm the PICO App shows "Connected"; check `num_motion_data_available()` > 0 |
| `/opt/apps/roboticsservice/runService.sh: No such file` | PC-Service `.deb` not installed | `sudo dpkg -i docker/prebuilt/XRoboToolkit_PC_Service_*.deb` |
| `libManusSDK_Integrated.so: file format not recognized` | Git LFS files weren't fetched | `git lfs install && git lfs pull` then rebuild |
| No TF data | PICO not connected, or tracker serials in YAML don't match the physical devices | Check XRoboToolkit App; re-check `tracker_serial_*` in `config/pico_input.yaml` |
| `pip3` installs into a conda env | conda is activated | Use `/usr/bin/python3 -m pip install --user ...` |

### Git LFS not fetched

The `.deb`, the PICO APK, and several `.so`s ship via Git LFS. If `dpkg -i` or `colcon build` errors with "file format not recognized" or "archive corrupt":

```bash
cd ~/ros2_ws/src/wuji-hand-teleop
sudo apt install git-lfs
git lfs install
git lfs pull
file src/input_devices/manus_input/manus_ros2/ManusSDK/lib/libManusSDK_Integrated.so
# Should print "ELF 64-bit LSB shared object" — not "ASCII text".
colcon build --symlink-install
```

---

## Package layout

```
pico_input/
├── install_sdk.sh                          One-click SDK installer (host install)
├── prebuilt/x86_64/                        Prebuilt .so (LFS): xrobotoolkit_sdk + libPXREARobotSDK
├── vendor/                                 Vendored upstream sources (Apache-2.0 + MIT)
│   ├── XRoboToolkit-PC-Service/            C++ daemon source — rebuild the .deb from here
│   └── XRoboToolkit-PC-Service-Pybind/     Python binding source
├── config/pico_input.yaml                  Runtime config (serials, smoothing, source mode)
├── launch/pico_input.launch.py             Standalone launch (PICO input only, no robot)
├── pico_input/
│   ├── pico_input_node.py                  ROS2 node — incremental control + TF + topics
│   ├── incremental_controller.py           Pure-Python controller (no ROS deps; unit-testable)
│   ├── xrobotoolkit_client.py              Pybind SDK wrapper
│   └── data_source/                        Live / recorded data sources behind a common ABC
├── test/                                   Step-by-step bring-up scripts and diagnostics
├── setup.py
├── package.xml
└── README.md
```

---

## Related packages

`pico_teleop.launch.py` composes the following packages:

| Package | Role |
|---|---|
| `pico_input` (this package) | PICO data acquisition + incremental control |
| `tianji_world_output` | Consumes `/*_target_pose` + `/*_elbow_direction`, drives the Tianji arm in world frame |
| `wuji_glove` *(default)* or `manus_input` *(optional)* | Hand input → joint angles |
| `wujihand_output` (controller side) | Hand retargeting + Wuji-hand driver |
| `camera` | Stereo head camera + RealSense wrists (see `src/camera/README.md`) |
| `wuji_teleop_bringup` | Top-level launch composition |

Build everything in one go:

```bash
cd ~/ros2_ws/src/wuji-hand-teleop
colcon build
source install/setup.bash
```

---

## Notes

- **Face the robot at init**, otherwise the fixed PICO→robot transform will rotate your forward direction relative to the robot's.
- The HMD pose is recorded **only** for RViz visualisation — it does not drive the arms. You can move your head freely after init.
- PC-Service and the ROS2 node both run on the host; the link between them is a local socket, so there is no extra network hop after the PICO→PC WiFi.
- Stereo VR camera setup (USB camera → H.264 → PICO HMD → `v4l2loopback` → ROS topics) is documented in `src/camera/README.md`.

---

## See also

- **[`ARCHITECTURE.md`](ARCHITECTURE.md)** — coordinate systems (PICO OpenXR → world → chest), incremental control derivation, arm-angle geometry, and the full mathematical transform chain. Read this if you're modifying `incremental_controller.py` / `xrobotoolkit_client.py` or chasing a coordinate-frame bug.
- **[`docs/PICO.md`](../../../docs/PICO.md)** — user-facing PICO setup guide (Developer Mode, XRoboToolkit APK install, ADB, stereo H.264 streaming).

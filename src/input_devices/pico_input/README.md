
# PICO Input - VR Teleoperation Input Module

Reads PICO VR tracking data from XRoboToolkit PC-Service and publishes TF for Tianji arm control.

**Version**: v1.0.0
**Last Updated**: 2024-01

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| v1.0.0 | 2024-01 | Initial version: PICO 4/4 Ultra support, incremental control mode, 4 Tracker configuration |

---

## Quick Start Commands

> **Installation complete?** Follow the process below to test step by step from simple to full

```bash
# Terminal 1: Start PC-Service (keep running)
/opt/apps/roboticsservice/runService.sh

# Step 1: Minimal mode (arms only, recommended for first test)
ros2 launch wuji_teleop_bringup pico_teleop_minimal.launch.py

# Step 2: Preview mode (verify all devices, with RViz, no robot control)
ros2 launch wuji_teleop_bringup pico_teleop.launch.py \
  enable_robot:=false enable_camera:=false enable_hand:=false enable_rviz:=true

# Step 3: Full teleoperation mode (dual arms + dexterous hand + camera)
ros2 launch wuji_teleop_bringup pico_teleop.launch.py
```

**Common parameters:**
```bash
# Full mode but disable MANUS gloves (when not installed)
ros2 launch wuji_teleop_bringup pico_teleop.launch.py enable_hand:=false

# Full mode but disable cameras (when not connected)
ros2 launch wuji_teleop_bringup pico_teleop.launch.py enable_camera:=false

# Minimal playback mode (no PICO hardware needed)
ros2 launch wuji_teleop_bringup pico_teleop_minimal.launch.py \
  data_source_type:=recorded playback_speed:=0.3
```

---

## Core Feature: Incremental Control Mode

**Safe, intuitive, jitter-free** control method:
- User hand moves X amount → robot hand moves X amount
- User hand rotates X degrees → robot hand rotates X degrees
- No sudden robot jumps during initialization

```
Target position = Robot initial position + (User current position - User initial position)
Target orientation = Robot initial orientation × User orientation increment
```

---

## PICO Control Logic Explained

### Why "Incremental Control"?

Imagine you're remote-controlling a toy car:

**Absolute control (bad):**
> "Put the car at my hand's position"
> 
> Problem: Your hand is on a 2-meter-high desk, and the car suddenly flies onto the desk!

**Incremental control (good):**
> "I push my hand forward 10cm, the car also moves forward 10cm"
> 
> Safe: No matter where your hand is, the car only moves the same distance

### What Do the 4 PICO Trackers Do?

```
    ┌─────────────────────────────────────────────────────────────┐
    │                      You (User)                              │
    │                                                             │
    │                    ┌───────────┐                            │
    │                    │   HMD     │ ← Only for RViz visualization│
    │                    │  (Head)   │   Records incremental motion │
    │                    └───────────┘                            │
    │                          │                                  │
    │           ┌──────────────┼──────────────┐                   │
    │           ↓              │              ↓                   │
    │     ┌──────────┐         │        ┌──────────┐              │
    │     │ Tracker  │         │        │ Tracker  │              │
    │     │   #2     │←────────│────────│   #3     │              │
    │     │ Left     │ Controls│        │ Right    │ Controls     │
    │     │ upper arm│ elbow   │        │ upper arm│ elbow        │
    │     └──────────┘ direction│        └──────────┘ direction   │
    │           │              │              │                   │
    │           ↓              │              ↓                   │
    │     ┌──────────┐         │        ┌──────────┐              │
    │     │ Tracker  │         │        │ Tracker  │              │
    │     │   #0     │←────────│────────│   #1     │              │
    │     │ Left     │ Controls│        │ Right    │ Controls     │
    │     │ wrist    │ hand    │        │ wrist    │ hand         │
    │     └──────────┘ pos+ori │        └──────────┘ pos+ori     │
    │                          │                                  │
    └─────────────────────────────────────────────────────────────┘
```

### Simple Analogy

| Tracker | Controls | Analogy |
|---------|----------|--------|
| HMD | Visualization | Only for RViz display of head position, does not affect robot |
| Wrist (#0, #1) | Hand position and angle | Remote control: where your hand goes, the robot hand follows |
| Upper arm (#2, #3) | Elbow height | Elbow up/down/outward? Follows your upper arm direction |

### Initialization Process (Press the "Start" button)

```
Step 1: Robot assumes initial pose
        ┌─────┐
        │     │ ← Robot hand at initial position
        │  🤖 │
        └─────┘

Step 2: Put on PICO, assume a similar pose
        ┌─────┐
        │     │ ← Your hand can be anywhere
        │  👤 │   but assuming a similar pose is more intuitive
        └─────┘

Step 3: Wait 5 seconds (or press init button)
        System records:
          ✓ HMD initial pose (visualization only)
          ✓ Where your hand is now (wrist Tracker)
          ✓ Where your elbow points now (upper arm Tracker)

Step 4: Start teleoperation!
        Your hand forward 10cm → Robot hand forward 10cm
        Your hand turns left 30° → Robot hand turns left 30°
```

### Why Does the Elbow Need Separate Control?

A 7-axis robotic arm is "redundant", meaning: **for the same hand position, the elbow can have many postures**.

```
Example: Hand fixed at the same position, elbow can be high or low

    Elbow raised:                  Elbow lowered:
    
         ╲                            ╱
          ●                          ●
           ╲                        ╱
            ╲                      ╱
             ● Hand               ● Hand

    Upper arm Tracker          Upper arm Tracker
    Y-axis pointing up         Y-axis pointing down
```

The **Y-axis direction** of the upper arm Tracker tells the robot: which direction the elbow should point.

## System Requirements

- Ubuntu 22.04 LTS
- ROS2 Humble
- Python 3.10+
- PICO 4 / PICO 4 Ultra headset
- PICO Motion Tracker (4 units: 2 wrist + 2 upper arm)
- (Optional) MANUS data gloves - for hand control
- (Optional) USB stereo camera - for VR visual passthrough

---

## Quick Start (From Zero to Running)

> **Goal**: Follow the steps below to eventually see PICO Tracker TF visualization in RViz

### One-Click Installation (Recommended)

```bash
# Enter pico_input directory
cd ~/Desktop/wuji-hand-teleop/src/input_devices/pico_input

# Run installation script (using prebuilt binaries)
./install_sdk.sh

# Or build from source
./install_sdk.sh --build
```

**The script will automatically complete:**
- Install xrobotoolkit_sdk (Python SDK)
- Configure LD_LIBRARY_PATH
- Install Python dependencies (numpy, scipy)

---

### Manual Installation Steps

<details>
<summary>Expand to view detailed steps</summary>

#### Step 1: Install ROS2 Humble

```bash
# Add ROS2 source
sudo apt update && sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2
sudo apt update
sudo apt install -y ros-humble-desktop

# Add to bashrc (auto-activate in every terminal)
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

#### Step 2: Install Python Dependencies

**Important: ROS2 uses system Python (`/usr/bin/python3`), not conda environment!**

```bash
# Install system-level dependencies
sudo apt install python3-pip cmake build-essential pybind11-dev

# Install Python dependencies to system Python
/usr/bin/python3 -m pip install numpy scipy --user
```

#### Step 3: Install XRoboToolkit PC-Service (Important!)

**PC-Service is the bridge between PICO headset and PC, must be installed first!**

```bash
# 1. Download deb package (using version maintained by this project)
cd ~/Downloads
wget https://github.com/lzhu686/XRoboToolkit-PC-Service/releases/download/v1.0.0/XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb

# 2. Install
sudo dpkg -i XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb

# 3. Verify installation succeeded
ls -la /opt/apps/roboticsservice/
# You should see runService.sh, RoboticsServiceProcess and other files

# 4. Test launch
/opt/apps/roboticsservice/runService.sh
# Seeing "release mode" indicates successful startup!
```

**If installation fails:**
```bash
sudo apt --fix-broken install
sudo dpkg -i XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb
```

#### Step 4: Install PICO APK

**Latest version: v1.4** (uses local coordinate system by default, no longer distinguishes local/global)

Download and install from GitHub Release:

```bash
# 1. Ensure PICO is connected via USB with developer mode enabled
#    PICO Settings → Developer Options → Enable USB Debugging

# 2. If you don't have adb, install it first
sudo apt install android-tools-adb

# 3. Download and install APK
wget https://github.com/lzhu686/XRoboToolkit-Unity-Client/releases/download/v1.4/v1.4.apk
adb install -r -g v1.4.apk

# 4. Verify installation succeeded
adb shell pm list packages | grep xrobo
# Should see: package:com.xrobotoolkit.client
```

**Coordinate system mode selection:**

| Mode | Stability | Use Case | Recommended |
|------|-----------|----------|-------------|
| **Local (Local coordinate system)** | Excellent | Single device use, Tianji arm teleoperation, daily development | **Recommended** |
| **Global (Global coordinate system)** | Good | Multi-device spatial alignment, requiring environment anchors | Special scenarios only |

**Notes:**
- **Local mode** is more stable, coordinate system relative to device initial position, does not depend on environment feature point tracking
- **Global mode** depends on environmental spatial anchors, may experience tracking instability due to lighting changes, environmental occlusion, etc.

#### Step 5: Install xrobotoolkit_sdk (Pybind)

**Method A: Using prebuilt binaries (Recommended)**

```bash
cd ~/Desktop/wuji-hand-teleop/src/input_devices/pico_input
./install_sdk.sh
```

**Method B: Build from source**

```bash
# 1. Clone and build C++ SDK
cd /tmp
git clone https://github.com/lzhu686/XRoboToolkit-PC-Service.git pc-service
cd pc-service/RoboticsService/PXREARobotSDK
bash build.sh

# 2. Clone Pybind SDK (if not already done)
cd ~/Desktop
git clone https://github.com/lzhu686/XRoboToolkit-PC-Service-Pybind.git

# 3. Copy C++ library files to Pybind directory
cd ~/Desktop/XRoboToolkit-PC-Service-Pybind
mkdir -p include lib
cp /tmp/pc-service/RoboticsService/PXREARobotSDK/PXREARobotSDK.h include/
cp -r /tmp/pc-service/RoboticsService/PXREARobotSDK/nlohmann include/
cp /tmp/pc-service/RoboticsService/PXREARobotSDK/build/libPXREARobotSDK.so lib/

# 4. Build and install Python SDK
mkdir -p build_user && cd build_user
cmake .. -DCMAKE_LIBRARY_OUTPUT_DIRECTORY=$(pwd)/output -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

# 5. Install to user site-packages
mkdir -p ~/.local/lib/python3.10/site-packages/
cp output/xrobotoolkit_sdk.cpython-310-x86_64-linux-gnu.so ~/.local/lib/python3.10/site-packages/

# 6. Copy shared library to user lib directory
mkdir -p ~/.local/lib
cp ~/Desktop/XRoboToolkit-PC-Service-Pybind/lib/libPXREARobotSDK.so ~/.local/lib/

# 7. Add library path to bashrc
echo 'export LD_LIBRARY_PATH=$HOME/.local/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc
```

**Verify installation:**
```bash
/usr/bin/python3 -c "import xrobotoolkit_sdk as xrt; print('SDK installation successful!')"
```

</details>

---

### Step 6: Build ROS2 Workspace

```bash
cd ~/Desktop/wuji-hand-teleop
colcon build
source install/setup.bash

# Recommended: add to bashrc for auto-activation
echo "source ~/Desktop/wuji-hand-teleop/install/setup.bash" >> ~/.bashrc
```

**If build fails:**
```bash
# View specific errors
colcon build --event-handlers console_direct+

# Common issue 1: Missing dependencies
rosdep install --from-paths src --ignore-src -r -y

# Common issue 2: Build specific packages only (skip problematic ones)
colcon build --packages-select pico_input wuji_teleop_bringup

# Common issue 3: Clean and rebuild
rm -rf build install log
colcon build
```

### Step 7: Network Configuration

**Important: PICO headset and PC must be on the same LAN (5GHz WiFi recommended)**

```bash
# Check PC IP
hostname -I
# Example output: 192.168.1.100

# On the PICO headset:
# 1. Open XRoboToolkit App
# 2. Enter PC IP address (e.g., 192.168.1.100)
# 3. Click connect
```

### Step 8: Launch Preview Mode for Verification

**This is the key step to verify all devices are connected properly!**

```bash
# Terminal 1: Start PC-Service (keep running)
/opt/apps/roboticsservice/runService.sh

# Terminal 2: Launch Preview mode (with RViz visualization)
ros2 launch wuji_teleop_bringup pico_teleop.launch.py \
  enable_robot:=false enable_camera:=false enable_hand:=false enable_rviz:=true
```

**You should see in RViz:**
- `world` coordinate frame (origin)
- `head` (HMD position, moves with head)
- `pico_left_wrist` / `pico_right_wrist` (wrist Trackers)
- `pico_left_arm` / `pico_right_arm` (upper arm Trackers)
- `world_left` / `world_right` (static, shoulder chest coordinate frames)
**Preview mode does not control the robot**, allowing safe verification of all input devices!

---

## When Do You Need to Rebuild with colcon?

| Change | colcon build needed? |
|--------|---------------------|
| Modify `.py` files (Python nodes) | No (symlink mode) |
| Modify `setup.py` / `package.xml` | Yes |
| Modify `.yaml` config files | Yes (or use absolute path directly) |
| Modify `launch` files | Yes |
| Add/delete files | Yes |

**Quick rebuild for a single package:**
```bash
colcon build --packages-select pico_input
source install/setup.bash
```

---

## Data Flow

```
PICO → APK → WiFi(TCP:63901) → PC-Service → Shared memory(localhost:60061)
                                                    ↓
                                             pico_input_node
                                                    ↓
                                  TF frames + Topics (target_pose, elbow_direction)
                                                    ↓
                                        tianji_world_output → Tianji arm
```

## Output

### TF (Always enabled)

```
world
├── head (HMD, dynamic)
├── world_left (Left arm Chest coordinate frame, rotated +90° around X axis)
│   ├── pico_left_wrist (Tracker #0, dynamic)
│   └── pico_left_arm (Tracker #2, dynamic)
└── world_right (Right arm Chest coordinate frame, rotated -90° around X axis)
    ├── pico_right_wrist (Tracker #1, dynamic)
    └── pico_right_arm (Tracker #3, dynamic)
```

### Topics (Enabled when enable_topic_publishing=true)

| Topic | Type | Description |
|-------|------|-------------|
| /pico_hmd | PoseStamped | HMD pose (world frame) |
| /pico_left_wrist | PoseStamped | Left wrist pose (world_left frame) |
| /pico_right_wrist | PoseStamped | Right wrist pose (world_right frame) |
| /left_arm_target_pose | PoseStamped | Left arm target pose (world_left frame) |
| /right_arm_target_pose | PoseStamped | Right arm target pose (world_right frame) |
| /left_arm_elbow_direction | Vector3Stamped | Left arm elbow direction (arm angle constraint) |
| /right_arm_elbow_direction | Vector3Stamped | Right arm elbow direction (arm angle constraint) |

## Usage

### Preview Mode (Recommended for first use)

**Verify all input devices before controlling the robot!**

```bash
# Terminal 1: Start PC-Service (keep running)
/opt/apps/roboticsservice/runService.sh

# Terminal 2: Launch Preview mode
ros2 launch wuji_teleop_bringup pico_teleop.launch.py \
  enable_robot:=false enable_camera:=false enable_hand:=false enable_rviz:=true
```

**Preview mode includes:**
- PICO input (TF publishing)
- RViz visualization (enabled by default)
- MANUS glove input (optional)
- Camera video stream (optional)
- Does NOT control Tianji arm
- Does NOT control Wuji hand

**Preview mode parameters:**
```bash
# Full launch (PICO + MANUS + Camera + RViz, no robot control)
ros2 launch wuji_teleop_bringup pico_teleop.launch.py \
  enable_robot:=false enable_rviz:=true

# PICO tracking + RViz only (no gloves, no camera)
ros2 launch wuji_teleop_bringup pico_teleop.launch.py \
  enable_robot:=false enable_rviz:=true enable_hand:=false enable_camera:=false

# Disable RViz
ros2 launch wuji_teleop_bringup pico_teleop.launch.py \
  enable_robot:=false enable_rviz:=false

# Disable gloves (when MANUS is not installed)
ros2 launch wuji_teleop_bringup pico_teleop.launch.py \
  enable_robot:=false enable_rviz:=true enable_hand:=false

# Disable camera (when camera is not connected)
ros2 launch wuji_teleop_bringup pico_teleop.launch.py \
  enable_robot:=false enable_rviz:=true enable_camera:=false
```

**pico_teleop.launch.py parameter description:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `enable_robot` | true | Enable Tianji arm control |
| `enable_hand` | true | Enable MANUS glove input |
| `enable_camera` | true | Enable stereo camera video stream |
| `enable_rviz` | false | Enable RViz visualization |

### Full Teleoperation Mode (Robot Control)

**Make sure Preview mode verification passes before using!**

```bash
# Terminal 1: Start PC-Service
/opt/apps/roboticsservice/runService.sh

# Terminal 2: Launch full teleoperation (dual arms + dexterous hand + camera)
ros2 launch wuji_teleop_bringup pico_teleop.launch.py
```

### Minimal Mode (Arms only, no hand/camera)

**Recommended for first test!** Interactive launch with automatic device readiness detection.

```bash
# Terminal 1: Start PC-Service
/opt/apps/roboticsservice/runService.sh

# Terminal 2: Minimal mode (pico_input + tianji_world_output only)
ros2 launch wuji_teleop_bringup pico_teleop_minimal.launch.py

# Or: Use recorded data (no PICO hardware needed)
ros2 launch wuji_teleop_bringup pico_teleop_minimal.launch.py \
  data_source_type:=recorded playback_speed:=0.3
```

### Test PICO Connection Only (Minimal mode)

```bash
# Terminal 1: Start PC-Service
/opt/apps/roboticsservice/runService.sh

# Terminal 2: Launch PICO input node only (no robot control)
ros2 launch pico_input pico_input.launch.py

# Terminal 3: Verify TF
ros2 run tf2_tools view_frames
evince frames.pdf
```

---

## Running Mode Comparison

| Mode | Launch File | Purpose | Controls Robot |
|------|-------------|---------|---------------|
| **Minimal Test** | `pico_input.launch.py` | Test PICO connection only (TF) | No |
| **Preview** | `pico_teleop.launch.py enable_robot:=false enable_rviz:=true` | Verify input devices + RViz visualization | No |
| **Minimal** | `pico_teleop_minimal.launch.py` | Arms only control (interactive launch) | Arms only |
| **Full Teleop** | `pico_teleop.launch.py` | Dual arms + dexterous hand + camera | All |

**Recommended test flow:** Minimal Test → Preview → Minimal → Full Teleop

### Preview Mode Details

```
┌─────────────────────────────────────────────────────────────────────────┐
│                  Preview Mode Data Flow (input only, no output)         │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  PICO SDK ──▶ pico_input_node ──▶ TF ──▶ RViz (visualization)         │
│                                      ❌ Not sent to tianji_world_output │
│                                                                         │
│  MANUS ──▶ manus_input ──▶ /hand_input ──▶ (visualization)            │
│                                      ❌ Not sent to wujihand_retargeting│
│                                                                         │
│  USB Camera ──▶ stereovr_main ──▶ PICO VR (H.264 video stream, 60fps) │
│                               ──▶ v4l2loopback (/dev/video99)          │
│                                        ↓                                │
│                               stereovr_publisher ──▶ ROS2 Topics        │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

### Preview Mode Checklist

Verify the following in Preview mode:

| Check Item | Verification Method | Expected Result |
|------------|-------------------|-----------------|
| TF tree correct | View Axes in RViz | See head, left_wrist, right_wrist, left_arm, right_arm |
| Tracker tracking normal | Move arms | TF frames follow movement |
| HMD orientation correct | Turn head | head frame follows rotation |
| (Optional) Glove data | `ros2 topic echo /hand_input` | See joint angle data |
| (Optional) Camera | `ros2 topic hz /stereo/left/compressed` | Frame rate > 25 fps |

---

## 4 PICO Trackers and Robotic Arm Mapping

### Overall Architecture Diagram

```
                    User                                      Robot
              ┌─────────────┐                           ┌─────────────┐
              │   Headset   │ ─(HMD)── Viz only ──→     │    Base     │
              │   (PICO)    │                           │   (world)   │
              └─────────────┘                           └─────────────┘
                    │                                         │
    ┌───────────────┼───────────────┐         ┌───────────────┼───────────────┐
    │               │               │         │               │               │
    ▼               ▼               ▼         ▼               ▼               ▼
┌───────┐      ┌───────┐       ┌───────┐   ┌───────┐     ┌───────┐      ┌───────┐
│Tracker│      │Tracker│       │Tracker│   │ Left  │     │ Right │      │ Head  │
│  #2   │      │  #0   │       │  #1   │   │shoulder│    │shoulder│     │(viz   │
│Left   │      │Left   │       │Right  │   │ left  │     │ right │      │ only) │
│upper  │      │wrist  │       │wrist  │   │_chest │     │_chest │      │       │
│arm    │      │       │       │       │   │       │     │       │      │       │
└───┬───┘      └───┬───┘       └───┬───┘   └───┬───┘     └───┬───┘      └───────┘
    │              │               │           │             │
    ▼              ▼               ▼           ▼             ▼
┌───────┐      ┌───────┐       ┌───────┐   ┌───────┐     ┌───────┐
│Tracker│      │       │       │       │   │ Left  │     │ Right │
│  #3   │      │       │       │       │   │ arm   │     │ arm   │
│Right  │      │       │       │       │   │7 DOF  │     │7 DOF  │
│upper  │      │       │       │       │   │       │     │       │
│arm    │      │       │       │       │   │       │     │       │
└───────┘      └───────┘       └───────┘   └───────┘     └───────┘
```

### Tracker Detailed Mapping Table

| Tracker | Wearing Position | Robot Mapping | Extracted Info | Purpose |
|---------|-----------------|---------------|----------------|---------|
| HMD | Head | - | Position + orientation (incremental) | RViz visualization only |
| #0 | Left wrist | Left arm end-effector | Position + orientation | Control robot left hand pose |
| #1 | Right wrist | Right arm end-effector | Position + orientation | Control robot right hand pose |
| #2 | Left upper arm | Left elbow (J4) | **Position direction** | IK arm angle control (zsp_para) |
| #3 | Right upper arm | Right elbow (J4) | **Position direction** | IK arm angle control (zsp_para) |

### Headset (HMD) Role

**The headset is used for RViz visualization only, it does not control the robot!**

```
HMD uses incremental control mode, same as Trackers:
- Records HMD initial pose at initialization
- Publishes incremental changes relative to initial pose at runtime
- Displays user head position in RViz (head frame)
```

**Note**: The current implementation uses fixed coordinate transforms (PICO→robot). Users should face the robot's forward direction during initialization.

### Wrist Tracker (#0, #1) Role

Controls the robot hand's **full pose** [x, y, z, qx, qy, qz, qw]:

```python
# Position increment
delta_pos = user current hand position - user initial hand position
robot target position = robot initial position + delta_pos

# Orientation increment  
delta_rot = user current hand orientation × user initial hand orientation⁻¹
robot target orientation = robot initial orientation × delta_rot
```

### Upper Arm Tracker (#2, #3) Role

**Does not control "elbow position", but rather "elbow direction"!**

A 7-DOF robotic arm is redundant: for the same hand pose, the elbow can have infinitely many postures.

```
        Shoulder ●─────────────────────● Hand (fixed pose)
            ╲                   ╱
             ╲                 ╱
              ● Elbow can move along this arc
               ╲             ╱
                ●           ●
                 ╲         ╱
                  ●───────●
                    Arm angle φ
```

The **Y-axis direction** of the upper arm Tracker tells IK: which direction the elbow should face.

```python
# Extract Y-axis direction vector from Tracker orientation
y_axis = tracker_rotation_matrix[:, 1]  # Second column

# Used as IK null-space parameter
zsp_para = [y_axis.x, y_axis.y, y_axis.z, 0, 0, 0]
```

---

## Incremental Control Mode Details

### Why Incremental Control?

**Problem:** The user's standing position and posture differ from the robot's. If you directly send the user's hand position to the robot, the robot will suddenly jump to a strange position.

**Solution:** Only send "change amounts", not "absolute positions".

```
At initialization:
    User hand position in PICO coordinate system: init_tracker_pos = [0.5, 1.2, -0.3]
    Robot hand initial position:                   robot_init_pos  = [0.5061, 0.1837, 0.1319]

At runtime:
    User hand moves to:              current_pos = [0.6, 1.25, -0.2]
    Change amount:                   delta = current_pos - init_tracker_pos
                                           = [0.1, 0.05, 0.1]
    Robot target position:           target = robot_init_pos + delta
                                           = [0.6061, 0.2337, 0.2319]
```

### Initialization Process

```
┌─────────────────────────────────────────────────────────────────────┐
│ Step 1: Robot moves to initial joint angles init_joints              │
│         End-effector position = value computed by get_init_pose.py   │
└─────────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────────┐
│ Step 2: User puts on PICO headset and Trackers                      │
│         Recommended: assume a pose similar to the robot (not        │
│         required, but more intuitive)                               │
└─────────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────────┐
│ Step 3: Wait 5 seconds (or manually call /pico_input/init)          │
│         System records:                                              │
│           - HMD initial pose (visualization only)                    │
│           - All Tracker initial poses                                │
└─────────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────────┐
│ Step 4: Start teleoperation                                         │
│         User moves hand → publishes increment → robot follows       │
└─────────────────────────────────────────────────────────────────────┘
```

### Incremental Control Formula Summary

| Data | Formula | Description |
|------|---------|-------------|
| Hand position | `target = robot_init_pos + (current - init)` | 1:1 following (with coordinate transform) |
| Hand orientation | `target = robot_init_rot × transform(current × init⁻¹)` | 1:1 following (axis-angle method) |
| Arm angle | `zsp_para = normalize(arm_position)` | Position vector normalization |

---

## Configuration

Configuration file: `config/pico_input.yaml`

| Parameter | Default | Description |
|-----------|---------|-------------|
| publish_rate | 90.0 | Publishing frequency (Hz) |
| pc_service_host | 127.0.0.1 | PC-Service address |
| pc_service_port | 60061 | PC-Service port |
| enable_topic_publishing | true | Publish /left_arm_target_pose etc. Topics |
| enable_legacy_topics | false | Publish /pico/* debug Topics |
| topic_prefix | pico | Legacy Topic prefix |
| auto_init_delay | 5.0 | Auto-initialization delay (seconds), 0 to disable |
| pos_ema_alpha | 0.6 | Position EMA smoothing coefficient |
| elbow_dir_ema_alpha | 1.0 | Arm angle direction EMA smoothing coefficient |
| elbow_gray_zone | 0.015 | Arm angle gray zone threshold (m) |
| data_source_type | live | Data source: "live" or "recorded" |
| recorded_file_path | (see YAML) | Recorded file path |
| playback_speed | 1.0 | Playback speed multiplier |
| loop_playback | true | Whether to loop playback |
| enable_debug_log | false | Enable CSV debug logging |
| tracker_serial_XXXXXX | (see YAML) | Tracker serial number → role mapping |

---

## Coordinate System Description

### Tianji Robot Coordinate System (Right-hand system)

```
        Z (Up)
        ↑
        │
        │
        └───────→ Y (Left)
       ╱
      ╱
     X (Forward, robot facing direction)
```

### OpenXR / PICO Coordinate System (Right-hand system)

```
        Y (Up)
        ↑
        │
        │
        └───────→ X (Right)
       ╱
      ╱
     Z (Forward, user facing direction)
```

### Coordinate Transform (PICO → Robot)

**Position transform:**
```
robot_x = -pico_z  (Forward: user moves forward → robot moves forward)
robot_y = -pico_x  (Left = -Right)
robot_z =  pico_y  (Up)
```

**Transform matrix:**
```python
pico_to_robot = np.array([
    [0, 0, -1],  # Robot X = -PICO Z
    [-1, 0, 0],  # Robot Y = -PICO X
    [0, 1, 0]    # Robot Z = PICO Y
])
# Note: det(pico_to_robot) = +1 (orthogonal matrix, no mirroring)
```

**Rotation mapping (using axis-angle method):**
| PICO Rotation Axis | User Action | Robot Rotation Axis |
|-------------------|-------------|---------------------|
| Around X axis | Tilt hand right | Around -Y axis (tilt right) |
| Around Y axis | Raise/lower wrist | Around Z axis |
| Around Z axis | Rotate wrist | Around X axis |

> **Technical details**: The code uses the **axis-angle method**: transforms the rotation axis vector, keeps the rotation angle unchanged.
> pico_to_robot matrix det=+1 (two axis negations cancel each other), it is an orthogonal matrix.

---

## Robot Initial Pose (FK Calculation Result)

Corresponding to Tianji initial joint angles (loaded from `tianji_robot.yaml`):
```python
INIT_JOINTS_LEFT  = [55.0, -65.0, -70.0, -60.0, 60.0, 0.0, 0.0]  # degrees
INIT_JOINTS_RIGHT = [-55.0, -65.0, 70.0, -60.0, -60.0, 0.0, 0.0]  # degrees
```

### End-Effector Pose (Wrist)

| Arm | Position [x, y, z] meters | Orientation [qx, qy, qz, qw] |
|-----|--------------------------|-------------------------------|
| Left | `[0.5733, 0.2237, 0.2762]` | `[0.0067, 0.7270, 0.0111, 0.6865]` |
| Right | `[0.5733, -0.2237, 0.2762]` | `[-0.0067, 0.7270, -0.0111, 0.6865]` |

### Arm Tracker Reference Position (Chest coordinate frame, for incremental control baseline)

| Arm | Position [x, y, z] meters | Orientation [qx, qy, qz, qw] |
|-----|--------------------------|-------------------------------|
| Left | `[0.2, 0.3, 0.2]` | `[0.4177, -0.0283, 0.5206, 0.7441]` |
| Right | `[0.2, -0.3, 0.2]` | `[0.7448, -0.5141, 0.0058, 0.4254]` |

**Physical meaning:** Elbow is 30cm below and 20cm outward from shoulder (sunken elbow + abduction posture)
- Left Chest: Y+=down, Z+=left(outward) → `[0.2, +0.3, +0.2]`
- Right Chest: Y-=down, Z+=right(outward) → `[0.2, -0.3, +0.2]`

### How to Recalculate Initial Pose

When initial joint angles are modified, end-effector pose needs to be recalculated:

```bash
cd src/output_devices/tianji_world_output/tianji_world_output
python3 get_init_pose.py
```

Update the output values in `tianji_world_output/config/tianji_robot.yaml` for `init_pos`, `init_rot`, and `init_quat` fields.

---

## DH Parameters Explained

DH (Denavit-Hartenberg) parameters are the standard method for describing a robotic arm's "skeletal structure".

### Think of the Robotic Arm as Building Blocks

```
       ╔═══╗
       ║ 7 ║  ← End-effector (wrist)
       ╚═╤═╝
         │ 
       ╔═╧═╗
       ║ 6 ║  ← Wrist joint
       ╚═╤═╝
         │ d5=314mm (forearm length)
       ╔═╧═╗
       ║ 4 ║  ← Elbow joint (J4) ← Tracker #2, #3 correspond here!
       ╚═╤═╝
         │ d3=287mm (upper arm length)
       ╔═╧═╗
       ║ 2 ║  ← Shoulder joint
       ╚═╤═╝
         │ d1=174.5mm (shoulder height)
       ╔═╧═╗
       ║ 1 ║  ← Base
       ╚═══╝
```

### DH Parameter Table (Tianji M6-CCS)

| Joint | a (mm) | α (deg) | d (mm) | θ (deg) | Meaning |
|-------|--------|---------|--------|---------|---------|
| 1 | 0 | 0 | **174.5** | 0 | Base to shoulder height |
| 2 | 0 | 90 | 0 | 0 | Shoulder horizontal rotation |
| 3 | 0 | -90 | **287** | 0 | Upper arm length |
| 4 | 18 | 90 | 0 | 180 | **Elbow joint** |
| 5 | 18 | 90 | **314** | 180 | Forearm length |
| 6 | 0 | 90 | 0 | 90 | Wrist joint |
| 7 | 0 | 90 | 0 | 90 | Wrist end |

### Meaning of the Four Parameters

```
a  = Translation along X axis (link horizontal offset)
α  = Rotation around X axis (joint twist angle)
d  = Translation along Z axis (link vertical height) ← This is the "bone length"
θ  = Rotation around Z axis (joint angle) ← This is the variable you control!
```

Simple mnemonic:
- **d** = How long the bone is (upper arm 287mm, forearm 314mm)
- **θ** = How much the joint has rotated (control variable)

---

## Tracker Wearing and Configuration

| Index | Wearing Position | TF Frame | Extracted Data |
|-------|-----------------|----------|----------------|
| HMD  | Head     | head     | Incremental pose (visualization only) |
| #0   | Left wrist   | pico_left_wrist | Position + orientation |
| #1   | Right wrist   | pico_right_wrist | Position + orientation |
| #2   | Left upper arm   | pico_left_arm | **Position only** |
| #3   | Right upper arm   | pico_right_arm | **Position only** |

### Tracker Wearing Recommendations

**Wrist Trackers (#0, #1):**
- Wear on the back of the wrist
- Controls robot hand **full pose** (position + orientation)

**Upper Arm Trackers (#2, #3):**
- Wear on the outer side of upper arm, approximately 10-15cm above the elbow joint
- Only **position information** is used, independent of Tracker rotation orientation
- Used to control the arm angle of the robotic arm (elbow raised/lowered)

### Upper Arm Tracker Wearing Position Diagram

```
        Shoulder
          ●
         /|
        / |
       /  |
      /   | Upper arm
     /    |
    ●     |  ←── Tracker (#2/#3) worn here (outer side of upper arm)
     \    |
      \   |
       \  |
        \ |
         \|
          ●  Elbow
          |
          |  Forearm
          |
          ●  Wrist
          |
          |  ←── Tracker (#0/#1) worn on wrist
          ●  Palm
```

### Position Vector and Arm Angle Control

**Upper arm Trackers use position only, not orientation!**

```
Data flow:

1. pico_input_node computes elbow direction:
   shoulder → wrist vector as axis, arm tracker position projected to perpendicular plane
   Gets elbow offset direction (geometric direction → negated before publishing to IK)

2. Published to Topic: /left_arm_elbow_direction (Vector3Stamped)

3. tianji_world_output_node subscribes to this Topic:
   zsp_para = [dir.x, dir.y, dir.z, 0, 0, 0]
   Tells IK solver which direction the elbow should face
```

**Arm angle control diagram:**

```
                Shoulder (world_left)
                  ●
                 /|
                / |
    Elbow up   /  |    Elbow down
              /   |          \
             ●    |           ●
    Tracker here  |     Tracker here
                  |
                  |
                  ●
                 Hand

  arm_direction   |   arm_direction
  points upper    |   points lower
  left            |   left
```

This is how `zsp_para` (Zero Space Parameter) works:
- A 7-DOF robotic arm can have infinitely many elbow postures when reaching the same hand pose
- The **position vector direction** tells the IK solver which direction the elbow should face
- Consistent with original developer's description: "vector from shoulder to elbow joint"

---

## Troubleshooting

### PC-Service Check

```bash
# Check if PC-Service is installed
ls -la /opt/apps/roboticsservice/runService.sh

# Start PC-Service
/opt/apps/roboticsservice/runService.sh
# Seeing "release mode" indicates successful startup!

# Check if PC-Service process is running
ps aux | grep -i robotics

# Check if gRPC port is listening
netstat -tlnp | grep 60061
# Or
ss -tlnp | grep 60061
```

### PICO Connection Check

```bash
# Check PICO network connection (need to know PICO IP)
ping <PICO_IP>

# Test SDK connection (requires PC-Service running)
LD_LIBRARY_PATH=$HOME/.local/lib:$LD_LIBRARY_PATH /usr/bin/python3 -c "
import xrobotoolkit_sdk as xrt
xrt.init()
print('Trackers:', xrt.num_motion_data_available())
print('HMD Pose:', xrt.get_headset_pose())
"
```

### ROS2 Check

```bash
# View TF tree
ros2 run tf2_tools view_frames
evince frames.pdf

# Check if nodes are running normally
ros2 node list

# Check topics
ros2 topic list

# View TF data stream
ros2 topic echo /tf --once
```

### Common Issues

| Issue | Possible Cause | Solution |
|-------|---------------|----------|
| `libManusSDK_Integrated.so: file format not recognized` | Git LFS files not downloaded | Run `git lfs install && git lfs pull` |
| `/opt/apps/roboticsservice/runService.sh: No such file` | PC-Service not installed | Install deb package per Step 3 |
| Cannot connect to PC-Service | Service not started | Run `/opt/apps/roboticsservice/runService.sh` |
| No TF data | PICO not connected | Check if PICO App shows connected |
| Package not found | Not built/not sourced | `colcon build && source install/setup.bash` |
| No MANUS data | MANUS Core not running | Start MANUS Core software |
| `No module named 'xrobotoolkit_sdk'` | Pybind SDK not installed | Install xrobotoolkit_sdk per Step 5 |
| `cannot find -lPXREARobotSDK` | C++ library not built | Build PC-Service's PXREARobotSDK first |
| `Found zero norm quaternions` | PICO not connected/no data | Ensure PICO headset is connected and running XRoboToolkit App |
| `No module named 'scipy'` | scipy not installed | `/usr/bin/python3 -m pip install scipy --user` |
| `pip3` installs to conda instead of system | conda environment activated | Use `/usr/bin/python3 -m pip install --user` |

### get_init_pose.py Error Fixes

**Issue 1: `ModuleNotFoundError: No module named 'scipy'`**
```bash
pip install scipy
```

**Issue 2: `NameError: name 'np' is not defined`**

Ensure `get_init_pose.py` has at the top:
```python
import numpy as np
```

### Git LFS Files Not Properly Downloaded

**Symptom:** `file format not recognized` or linker errors during build

**Cause:** This repository uses Git LFS to store large binary files (such as MANUS SDK libraries). If LFS files were not properly fetched during cloning, these files will be text pointers instead of actual binaries.

**Solution steps:**
```bash
# 1. Enter repository directory
cd ~/Desktop/wuji-hand-teleop

# 2. Install Git LFS (if not installed)
sudo apt install git-lfs

# 3. Initialize and pull LFS files
git lfs install
git lfs pull

# 4. Verify file type
file src/input_devices/manus_input/manus_ros2/ManusSDK/lib/libManusSDK_Integrated.so
# Correct: ELF 64-bit LSB shared object...
# Wrong: ASCII text (need to re-run git lfs pull)

# 5. Rebuild
colcon build --symlink-install
```

---

## File Structure

```
pico_input/
├── install_sdk.sh                   # One-click installation script
├── prebuilt/                        # Prebuilt binaries
│   └── x86_64/
│       ├── libPXREARobotSDK.so
│       └── xrobotoolkit_sdk.cpython-310-x86_64-linux-gnu.so
├── config/pico_input.yaml           # Configuration file
├── launch/pico_input.launch.py      # Launch file
├── pico_input/
│   ├── pico_input_node.py           # ROS2 node (incremental control, no rebuild needed after modification)
│   ├── incremental_controller.py    # Incremental controller (pure computation, no ROS2 dependency)
│   ├── xrobotoolkit_client.py       # Pybind SDK wrapper
│   └── data_source/                 # Data source abstraction layer + implementations
│       ├── __init__.py              # re-export (DataSource, TrackerData, HeadsetData)
│       ├── base.py                  # ABC base class (DataSource, TrackerData, HeadsetData)
│       ├── live_data_source.py      # Real PICO SDK data source
│       └── recorded_data_source.py  # Recorded file playback data source
├── test/
│   ├── common/                      # Shared modules
│   │   ├── robot_config.py          # Test compatibility layer (loads from tianji_robot.yaml)
│   │   ├── robot_lifecycle.py       # Robot power on/off lifecycle management
│   │   └── transform_utils.py       # Re-export wrapper → tianji_world_output
│   ├── docs/
│   │   └── PICO_TELEOP_GUIDE.md     # Complete teleoperation guide
│   ├── tool/                        # Utility scripts
│   │   ├── move_to_init_pose.py     # Move robot to initial pose
│   │   ├── verify_fk_values.py      # Verify FK computed values
│   │   └── diagnose_zsp_para.py     # FK→IK closed-loop diagnose zsp_para
│   ├── step1_direct_joint_control.py     # Direct joint control
│   ├── step2_pose_topic_control.py       # ROS2 Topic pose control
│   ├── step3_visualize_in_rviz.py        # RViz coordinate frame visualization
│   ├── step4_visualize_recorded_data.py  # PICO recorded data visualization
│   ├── step5_incremental_control_with_robot.py  # PICO incremental control with real robot
│   └── step6_arm_angle_stability_test.py  # Arm angle stability test
├── setup.py                         # Package config (rebuild needed after modification)
├── package.xml                      # ROS2 package description (rebuild needed after modification)
└── README.md
```

---

## Related Package Dependencies

Full teleoperation (`pico_teleop.launch.py`) requires the following packages:

| Package | Function | Source |
|---------|----------|--------|
| `pico_input` | PICO data acquisition | This package |
| `manus_ros2` | MANUS driver | src/input_devices/manus_input/ |
| `manus_input_py` | MANUS data conversion | src/input_devices/manus_input/ |
| `tianji_world_output` | Tianji arm control (World coordinate frame) | src/output_devices/tianji_world_output/ |
| `wujihand_ik` | Wuji hand IK | src/wujihand_ik/ |
| `tf2_ros` | TF transforms | ros-humble-tf2-ros (apt install) |

**Build all packages at once:**
```bash
cd ~/Desktop/wuji-hand-teleop
colcon build
source install/setup.bash
```

---

## Complete Data Flow Architecture

### System Architecture Diagram

```
┌─────────────────────────────────────────────────────────────────────────────────────┐
│                                    User Side                                         │
├─────────────────────────────────────────────────────────────────────────────────────┤
│                                                                                     │
│   ┌──────────┐    ┌──────────┐    ┌──────────┐    ┌──────────┐    ┌──────────┐    │
│   │   HMD    │    │Tracker#0 │    │Tracker#1 │    │Tracker#2 │    │Tracker#3 │    │
│   │(Headset) │    │(L wrist) │    │(R wrist) │    │(L upper  │    │(R upper  │    │
│   │          │    │          │    │          │    │  arm)    │    │  arm)    │    │
│   └────┬─────┘    └────┬─────┘    └────┬─────┘    └────┬─────┘    └────┬─────┘    │
│        │               │               │               │               │           │
│        └───────────────┴───────────────┴───────────────┴───────────────┘           │
│                                        │ Bluetooth/USB                              │
│                                        ▼                                            │
│                               ┌────────────────┐                                    │
│                               │ PICO Headset   │                                    │
│                               │ XRoboToolkit   │                                    │
│                               │     App        │                                    │
│                               └───────┬────────┘                                    │
│                                       │                                             │
└───────────────────────────────────────┼─────────────────────────────────────────────┘
                                        │ TCP (WiFi, port 63901)
                                        ▼
┌───────────────────────────────────────────────────────────────────────────────────┐
│                                    PC Side                                         │
├───────────────────────────────────────────────────────────────────────────────────┤
│                                                                                   │
│   ┌──────────────────────────────────┐                                           │
│   │    XRoboToolkit PC-Service        │ ← TCP Server (port 63901)                │
│   │  Receives PICO raw tracking data  │                                           │
│   └────────────────┬─────────────────┘                                           │
│                    │ Shared memory (localhost:60061)                               │
│                    ▼                                                              │
│   ┌──────────────────────────────────┐                                           │
│   │        pico_input_node            │ ← ROS2 node (90Hz)                        │
│   │  ┌─────────────────────────────┐ │                                           │
│   │  │ 1. Read PICO raw data       │ │                                           │
│   │  │ 2. Init: record HMD initial │ │                                           │
│   │  │         pose                │ │                                           │
│   │  │         Record Tracker      │ │                                           │
│   │  │         initial poses       │ │                                           │
│   │  │ 3. Compute incremental pose │ │ ← controller.compute_target_pose()       │
│   │  │    delta = current - init   │ │                                           │
│   │  │    target = robot_init+delta│ │                                           │
│   │  │ 4. Publish TF + Topics      │ │                                           │
│   │  └─────────────────────────────┘ │                                           │
│   └────────────────┬─────────────────┘                                           │
│                    │                                                              │
│                    │ TF publishing (inside pico_input_node):                      │
│                    │   world → world_left/right (static, chest frames)            │
│                    │   world_left/right → pico_*_wrist (dynamic, wrist poses)     │
│                    │   world_left/right → pico_*_arm (dynamic, elbow direction)   │
│                    │   world → head (dynamic, visualization only)                 │
│                    │                                                              │
│                    │ Topics publishing:                                            │
│                    │   /left_arm_target_pose, /right_arm_target_pose              │
│                    │   /left_arm_elbow_direction, /right_arm_elbow_direction      │
│                    ▼                                                              │
│   ┌──────────────────────────────────┐                                           │
│   │    tianji_world_output_node      │ ← ROS2 node (90Hz)                        │
│   │  ┌─────────────────────────────┐ │                                           │
│   │  │ 1. Subscribe to Topics:     │ │                                           │
│   │  │    /left_arm_target_pose    │ │ ← arm target pose (chest frame)           │
│   │  │    /left_arm_elbow_direction│ │ ← elbow direction (zsp_para)              │
│   │  │                             │ │                                           │
│   │  │ 2. Update zsp_para          │ │ ← from elbow_direction topic              │
│   │  │                             │ │                                           │
│   │  │ 3. IK inverse kinematics    │ │ ← controller.move_to_pose_direct()       │
│   │  │    Input: end-effector pose │ │                                           │
│   │  │    + zsp_para               │ │                                           │
│   │  │    Output: 7 joint angles   │ │                                           │
│   │  │                             │ │                                           │
│   │  │ 4. Publish joint commands   │ │                                           │
│   │  └─────────────────────────────┘ │                                           │
│   └────────────────┬─────────────────┘                                           │
│                    │                                                              │
│                    │ ROS2 Topics:                                                 │
│                    │   /tianji_arm/left/joint_command                             │
│                    │   /tianji_arm/right/joint_command                            │
│                    ▼                                                              │
└────────────────────┼──────────────────────────────────────────────────────────────┘
                     │ Network (ROS2 Topic / custom protocol)
                     ▼
┌───────────────────────────────────────────────────────────────────────────────────┐
│                                   Robot Side                                       │
├───────────────────────────────────────────────────────────────────────────────────┤
│   ┌──────────────────────────────────┐                                           │
│   │         Tianji Arm Driver         │                                           │
│   │   Receives joint angle commands,  │                                           │
│   │   drives motors                   │                                           │
│   │                                  │                                           │
│   │   Joints 1-7 move simultaneously │                                           │
│   └──────────────────────────────────┘                                           │
└───────────────────────────────────────────────────────────────────────────────────┘
```

---

## Key Data Transform Details

### Stage 1: pico_input_node Initialization

When the user presses initialize (or waits 5 seconds for auto-initialization):

```python
# 1. Record HMD initial pose (visualization only)
self.init_hmd_pose = self._pose_to_matrix(hmd_pose)

# 2. Record each Tracker's initial 4x4 transform matrix
for role in ['left_wrist', 'right_wrist', 'left_arm', 'right_arm']:
    self.init_tracker_poses[role] = self._pose_to_matrix(tracker_pose)
```

**Incremental control principle:**
- Records all Tracker positions and orientations at initialization
- Computes only the change relative to initial state at runtime
- User hand moves X amount, robot hand moves X amount

### Stage 2: pico_input_node Incremental Computation

Target pose computed each frame (90Hz):

```python
# ==================== Position Increment ====================
# 1. Compute user hand displacement in PICO coordinate system
delta_pos_pico = current_T[:3, 3] - init_T[:3, 3]

# 2. Transform displacement to robot coordinate system
delta_pos_robot = pico_to_robot @ delta_pos_pico

# 3. Add to robot initial position
target_pos = robot_init_pos + delta_pos_robot

# ==================== Orientation Increment (Axis-Angle Method) ====================
# 1. Compute user hand orientation change (in PICO coordinate system)
delta_rot_pico = current_rot * init_rot.inv()

# 2. Transform rotation using axis-angle method
#    Note: Uses axis-angle method to transform rotation (pico_to_robot det=+1)
rotvec = delta_rot_pico.as_rotvec()
angle = np.linalg.norm(rotvec)
axis = rotvec / angle
axis_robot = pico_to_robot @ axis  # Transform rotation axis
delta_rot_robot = R.from_rotvec(axis_robot * angle)

# 3. Apply to robot initial orientation
target_rot = robot_init_rot * delta_rot_robot
target_quat = target_rot.as_quat()
```

**Why use the axis-angle method?**
- `pico_to_robot` matrix is a coordinate axis mapping matrix (det=+1, orthogonal matrix)
- Axis-angle method directly transforms the rotation axis vector, conceptually clear and efficient
- Equivalent to `R.from_matrix(pico_to_robot) * delta * R.from_matrix(pico_to_robot).inv()`, but avoids redundant matrix operations

### Stage 3: TF Tree Structure

```
                            world (fixed, robot base)
                              │
    ┌─────────────────────────┼─────────────────────────┐
    ↓                         ↓                         ↓
world_left              world_right                   head
 (static, chest)         (static, chest)             (dynamic)
    │                         │
    ├── pico_left_wrist       ├── pico_right_wrist
    │   (dynamic, wrist)      │   (dynamic, wrist)
    └── pico_left_arm         └── pico_right_arm
        (dynamic, elbow)          (dynamic, elbow)
```

Note: `left_dh_ee`/`right_dh_ee` are only published by test scripts (step3/step4), not part of production nodes.

### Tracker Data Usage Differences (Key!)

**Although all Trackers publish full poses (xyz + quaternion), downstream usage differs:**

| Tracker | Published Content | **Actual Usage** | Purpose |
|---------|-------------------|-----------------|---------|
| **pico_left/right_wrist** | Full pose | **Position + orientation** | IK end-effector target |
| **pico_left/right_arm** | Full pose | **Position only → normalized direction** | zsp_para (elbow arm angle) |

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        Data Usage Differences Explained                  │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  Wrist Trackers (#0, #1):                                               │
│    ┌───────────────────────────────────────────────────────────────┐   │
│    │ Input: PICO raw pose [x, y, z, qx, qy, qz, qw]               │   │
│    │                     ↓                                         │   │
│    │ Incremental calc: delta_pos = current_pos - init_pos          │   │
│    │                   delta_rot = current_rot × init_rot⁻¹        │   │
│    │                     ↓                                         │   │
│    │ Output: target pose = robot_init + delta                      │   │
│    │       [x, y, z, qx, qy, qz, qw] ← Both position & orient!  │   │
│    │                     ↓                                         │   │
│    │ IK input: End-effector full 6-DOF pose                       │   │
│    └───────────────────────────────────────────────────────────────┘   │
│                                                                         │
│  Upper Arm Trackers (#2, #3):                                           │
│    ┌───────────────────────────────────────────────────────────────┐   │
│    │ Input: PICO raw pose [x, y, z, qx, qy, qz, qw]               │   │
│    │                     ↓                                         │   │
│    │ Incremental calc: delta_pos = current_pos - init_pos          │   │
│    │                   delta_rot = current_rot × init_rot⁻¹        │   │
│    │                     ↓                                         │   │
│    │ Output: target pose = robot_init + delta                      │   │
│    │       [x, y, z, qx, qy, qz, qw]                               │   │
│    │                     ↓                                         │   │
│    │ pico_input_node computes elbow direction:                     │   │
│    │   shoulder→wrist vector as axis, arm position projected to    │   │
│    │   perpendicular plane                                         │   │
│    │   Gets geometric offset direction, negated before publishing  │   │
│    │   to IK                                                       │   │
│    │                     ↓                                         │   │
│    │ Publishes: /left_arm_elbow_direction (Vector3Stamped)         │   │
│    │ IK input: zsp_para = [dir.x, dir.y, dir.z, 0, 0, 0]          │   │
│    └───────────────────────────────────────────────────────────────┘   │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

**Why do upper arm Trackers only use position?**

1. **Different purpose**: Upper arm Trackers control "which direction the elbow faces", not "elbow posture"
2. **More robust**: Only depends on Tracker position, not on wearing rotation orientation
3. **Geometric computation**: pico_input_node computes offset direction via perpendicular projection from shoulder→wrist axis and arm position

```python
# Key data flow in pico_input_node.py

# Wrist: pico_input computes target pose, publishes to /left_arm_target_pose
pos, quat = controller.compute_target_pose(pose, role)
# → [x, y, z, qx, qy, qz, qw] sent via PoseStamped Topic to tianji_world_output

# Upper arm: pico_input computes elbow direction, publishes to /left_arm_elbow_direction
direction, proj_point = controller.compute_elbow_direction(
    shoulder_pos, wrist_pos, elbow_pos, side
)
ik_direction = -direction  # Geometric direction negated → IK anti-gravity direction
# → [dx, dy, dz] sent via Vector3Stamped Topic to tianji_world_output
```

### Stage 4: tianji_world_output_node Topic Subscription and IK

```python
# 1. Subscribe to wrist target pose Topic (published by pico_input)
#    Callback converts PoseStamped → 4x4 matrix
self.left_arm_pose = self._pose_to_matrix(msg.pose)

# 2. Subscribe to elbow direction Topic (published by pico_input)
self.left_arm_direction = [msg.vector.x, msg.vector.y, msg.vector.z]

# 3. Update IK null-space parameter in control loop
self.controller.left_zsp_para = [
    self.left_arm_direction[0],
    self.left_arm_direction[1],
    self.left_arm_direction[2],
    0, 0, 0
]

# 4. Call IK to compute joint angles
l_success, r_success, l_joints, r_joints = self.controller.move_to_pose_direct(
    left_pose=self.left_arm_pose,   # 4x4 matrix
    right_pose=self.right_arm_pose,
    unit='matrix'
)
```

### Stage 5: IK Null-Space Parameter (zsp_para) Details

**Problem: 7-DOF robotic arm is redundant**
- Same hand pose can have infinitely many elbow postures
- Additional constraint needed to determine unique solution

**Solution: zsp_para reference plane**
```python
# zsp_para = [x, y, z, a, b, c]
# First three values [x, y, z] define a direction vector
# IK solver will try to orient the elbow toward this direction

# Examples:
zsp_para = [0.7, 0.0, 0.7, 0, 0, 0]  # Elbow toward upper right
zsp_para = [0.0, -1.0, 0.0, 0, 0, 0]  # Elbow downward
```

### Elbow Direction Computation Algorithm (IncrementalController.compute_elbow_direction)

**Core idea:** Project arm tracker position onto the perpendicular plane of the shoulder-wrist line to get elbow offset direction.

```
            Shoulder (shoulder, chest origin)
              ●
             /|\\
            / | \\
           /  |  \\
  shoulder-/ |  \\ shoulder-
  elbow  /   |   \\  wrist
        /    |    \\
       ●─────●─────●
    Elbow   Proj   Wrist
            point

        ←────→
      Elbow offset vector (direction)
```

**Computation steps:**

```python
# IncrementalController.compute_elbow_direction()

# 1. Shoulder-wrist vector (arm main axis)
shoulder_to_wrist = wrist - shoulder
sw_unit = shoulder_to_wrist / norm(shoulder_to_wrist)

# 2. Project elbow onto shoulder-wrist line
shoulder_to_elbow = elbow - shoulder
proj_length = dot(shoulder_to_elbow, sw_unit)
proj_point = shoulder + proj_length * sw_unit

# 3. Elbow offset vector = actual elbow position - projection point
elbow_offset = elbow - proj_point

# 4. Gray zone debounce: maintain last stable direction when offset is too small
if norm(elbow_offset) < elbow_gray_zone:
    direction = last_stable_direction
else:
    direction = normalize(elbow_offset)

# 5. EMA smoothing (re-normalize to ensure unit vector)
smoothed = alpha * direction + (1 - alpha) * prev_smoothed
direction = normalize(smoothed)
```

**Negation before publishing to IK:**
```python
# In pico_input_node.py:
ik_direction = -direction  # Geometric direction negated → IK anti-gravity direction
# → Published to /left_arm_elbow_direction (Vector3Stamped)
# → tianji_world_output subscribes and sets zsp_para = [dir.x, dir.y, dir.z, 0, 0, 0]
```

**Why negate?** The geometric elbow offset direction points toward gravity (elbow below when sunken),
while IK zsp_para requires anti-gravity direction (reference plane normal vector).

---

## Important Notes

### Initialization Timing
1. Ensure robot has reached `init_joints` pose
2. Ensure PC-Service is running
3. Ensure PICO is connected
4. User should assume pose before initializing

### Coordinate System Alignment
- User should **face the robot's forward direction** during initialization
- Uses fixed coordinate transform (PICO→robot), user's forward corresponds to robot's forward

### Tracker Serial Number Confirmation
```yaml
# pico_input.yaml — maps tracker role via 6-digit serial number
tracker_serial_YOUR_LEFT_WRIST_SN: "pico_left_wrist"   # Confirm actual wearing position
tracker_serial_YOUR_RIGHT_WRIST_SN: "pico_right_wrist"
tracker_serial_YOUR_LEFT_ARM_SN: "pico_left_arm"
tracker_serial_YOUR_RIGHT_ARM_SN: "pico_right_arm"
```

### Network Latency
- 5GHz WiFi recommended
- PC-Service and ROS2 nodes run on the same machine

---

## StereoVR Stereo Camera Configuration (v2.0)

The StereoVR system has been upgraded to v2.0, adopting a dual-process architecture for higher stability and performance isolation.

### Architecture Overview

```
┌─────────────────────────────────��─��─────────────────────────────────────────┐
│                     StereoVR v2.0 Dual-Process Architecture                 │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   USB Camera (/dev/stereo_camera)                                          │
│        │                                                                    │
│        ▼                                                                    │
│   stereovr_main (main process)                                              │
│        │                                                                    │
│        ├──▶ H.264 encoding ──▶ PICO VR (60fps, low latency)               │
│        │                                                                    │
│        └──▶ BGR24 ──▶ v4l2loopback (/dev/video99) ──┐                      │
│                                                      │                      │
│                                                      ▼                      │
│                                        stereovr_publisher (ROS2 process)    │
│                                                      │                      │
│                                                      ▼                      │
│                                          /stereo/left/compressed            │
│                                          /stereo/right/compressed           │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Launch Commands

```bash
# Method 1: Use ROS2 Launch file (recommended)
ros2 launch camera stereovr_launch.py

# Launch with parameters
ros2 launch camera stereovr_launch.py \
    camera_device:=/dev/stereo_camera \
    loopback_device:=/dev/video99 \
    fps:=30

# Method 2: Launch processes separately (for debugging)
# Terminal 1: Start main process
ros2 run camera stereovr_main --device /dev/stereo_camera --loopback /dev/video99

# Terminal 2: Start ROS2 publisher
ros2 run camera stereovr_publisher --device /dev/video99 --fps 30
```

### Configuration File

Configuration file location: `src/camera/config/stereovr/stereovr_config.yaml`

```yaml
stereovr:
  camera_device: "/dev/stereo_camera"    # Stereo camera device
  loopback_device: "/dev/video99"        # v4l2loopback virtual device

  resolution:
    width: 2560     # Total stereo width
    height: 720     # Single eye height
    fps: 30         # Frame rate

  topics:
    left: "/stereo/left/compressed"
    right: "/stereo/right/compressed"
```

### v4l2loopback Setup

Before using StereoVR, the v4l2loopback kernel module must be loaded:

```bash
# Load module
sudo modprobe v4l2loopback devices=1 video_nr=99 card_label='StereoVR'

# Verify device
ls -la /dev/video99
v4l2-ctl --device /dev/video99 --all
```

### UDEV Rules (Create fixed device links)

Create fixed device links for the stereo camera to avoid device number changes:

```bash
# Find camera VID/PID
lsusb | grep -i camera

# Create udev rule
sudo tee /etc/udev/rules.d/99-stereo-camera.rules << 'EOF'
SUBSYSTEM=="video4linux", ATTRS{idVendor}=="1bcf", ATTRS{idProduct}=="2d4f", ATTR{index}=="0", SYMLINK+="stereo_camera", MODE="0666"
EOF

# Reload rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### Camera Changelog

| Version | Date | Changes |
|---------|------|---------|
| v2.0 | 2025-01 | Migrated to stereocamera package, dual-process architecture, v4l2loopback support |
| v1.0 | 2024-12 | Initial version, single-process xrobo_server |

---

# PICO Input - VR 遥操作输入模块

从 XRoboToolkit PC-Service 读取 PICO VR 追踪数据，发布 TF 供天机臂控制使用。

**版本**: v1.0.0
**最后更新**: 2024-01

---

## 📋 版本历史

| 版本 | 日期 | 更新内容 |
|------|------|----------|
| v1.0.0 | 2024-01 | 初始版本：PICO 4/4 Ultra 支持，增量控制模式，4 Tracker 配置 |

---

## ⚡ 快速启动命令

> **已完成安装？** 按以下流程从简到全逐步测试

```bash
# 终端 1: 启动 PC-Service (保持运行)
/opt/apps/roboticsservice/runService.sh

# 步骤 1: Minimal 模式 (仅双臂, 推荐首次测试)
ros2 launch wuji_teleop_bringup pico_teleop_minimal.launch.py

# 步骤 2: Preview 模式 (验证全部设备，带 RViz，不控制机器人)
ros2 launch wuji_teleop_bringup pico_teleop.launch.py \
  enable_robot:=false enable_camera:=false enable_hand:=false enable_rviz:=true

# 步骤 3: 完整遥操作模式 (双臂 + 灵巧手 + 相机)
ros2 launch wuji_teleop_bringup pico_teleop.launch.py
```

**常用参数：**
```bash
# 完整模式但关闭 MANUS 手套 (未安装时)
ros2 launch wuji_teleop_bringup pico_teleop.launch.py enable_hand:=false

# 完整模式但关闭摄像头 (未连接时)
ros2 launch wuji_teleop_bringup pico_teleop.launch.py enable_camera:=false

# Minimal 录播模式 (无需 PICO 硬件)
ros2 launch wuji_teleop_bringup pico_teleop_minimal.launch.py \
  data_source_type:=recorded playback_speed:=0.3
```

---

## 核心特性：增量控制模式

**安全、直观、无跳动** 的控制方式：
- 用户手移动多少 → 机器人手移动多少
- 用户手转多少角度 → 机器人手转多少角度
- 初始化时不会导致机器人突然跳动

```
目标位置 = 机器人初始位置 + (用户当前位置 - 用户初始位置)
目标姿态 = 机器人初始姿态 × 用户姿态增量
```

---

## 🎯 PICO 控制逻辑通俗解释

### 为什么需要"增量控制"？

想象你在遥控一辆玩具车：

**❌ 绝对控制（不好）：**
> "把车放到我手的位置"
> 
> 问题：你的手在 2 米高的桌子上，车突然飞到桌子上！

**✅ 增量控制（好）：**
> "我手往前推 10 厘米，车也往前走 10 厘米"
> 
> 安全：无论你手在哪里，车都只会移动相同的距离

### PICO 的 4 个追踪器分别干什么？

```
    ┌─────────────────────────────────────────────────────────────┐
    │                        你 (用户)                            │
    │                                                             │
    │                    ┌───────────┐                            │
    │                    │   头显    │ ← 仅用于 RViz 可视化        │
    │                    │   (HMD)   │   记录增量运动             │
    │                    └───────────┘                            │
    │                          │                                  │
    │           ┌──────────────┼──────────────┐                   │
    │           ↓              │              ↓                   │
    │     ┌──────────┐         │        ┌──────────┐              │
    │     │ Tracker  │         │        │ Tracker  │              │
    │     │   #2     │←────────│────────│   #3     │              │
    │     │ 左大臂   │ 控制肘部 │        │ 右大臂   │ 控制肘部     │
    │     └──────────┘ 方向     │        └──────────┘ 方向        │
    │           │              │              │                   │
    │           ↓              │              ↓                   │
    │     ┌──────────┐         │        ┌──────────┐              │
    │     │ Tracker  │         │        │ Tracker  │              │
    │     │   #0     │←────────│────────│   #1     │              │
    │     │ 左手腕   │ 控制手的 │        │ 右手腕   │ 控制手的     │
    │     └──────────┘ 位置+姿态│        └──────────┘ 位置+姿态    │
    │                          │                                  │
    └─────────────────────────────────────────────────────────────┘
```

### 简单比喻

| 追踪器 | 控制什么 | 比喻 |
|--------|----------|------|
| 头显 (HMD) | 可视化 | 仅用于 RViz 显示头部位置，不影响机器人 |
| 手腕 (#0, #1) | 手的位置和角度 | 遥控器：你手移到哪，机器人手就跟到哪 |
| 大臂 (#2, #3) | 肘部的高低 | 肘部向上/向下/向外？跟着你的大臂方向 |

### 初始化过程（按一下"开始"按钮）

```
步骤 1: 机器人摆好初始姿势
        ┌─────┐
        │     │ ← 机器人手在初始位置
        │  🤖 │
        └─────┘

步骤 2: 你戴上 PICO，摆个类似的姿势
        ┌─────┐
        │     │ ← 你的手随便在哪都行
        │  👤 │   但摆类似的姿势会更直观
        └─────┘

步骤 3: 等 5 秒 (或按初始化按钮)
        系统记住：
          ✓ HMD 初始位姿 (仅用于可视化)
          ✓ 你现在手在哪 (手腕 Tracker)
          ✓ 你现在肘部朝哪 (大臂 Tracker)

步骤 4: 开始遥操作！
        你手往前 10cm → 机器人手往前 10cm
        你手向左转 30° → 机器人手向左转 30°
```

### 为什么肘部需要单独控制？

7 轴机械臂是"冗余"的，意思是：**同一个手的位置，肘部可以有很多姿态**。

```
例子：手固定在同一个位置，肘部可以高或低

    肘部抬高:                    肘部放低:
    
         ╲                            ╱
          ●                          ●
           ╲                        ╱
            ╲                      ╱
             ● 手                 ● 手

    大臂 Tracker Y轴向上        大臂 Tracker Y轴向下
```

大臂 Tracker 的 **Y 轴方向** 告诉机器人：肘部应该往哪边。

## 系统要求

- Ubuntu 22.04 LTS
- ROS2 Humble
- Python 3.10+
- PICO 4 / PICO 4 Ultra 头显
- PICO Motion Tracker (4个：2个手腕 + 2个大臂)
- (可选) MANUS 数据手套 - 用于手部控制
- (可选) USB 立体摄像头 - 用于 VR 视觉透传

---

## 🚀 快速开始 (从零到运行)

> **目标**: 按照以下步骤操作，最终能在 RViz 中看到 PICO Tracker 的 TF 可视化

### ⚡ 一键安装 (推荐)

```bash
# 进入 pico_input 目录
cd ~/Desktop/wuji-teleop-ros2-private/src/input_devices/pico_input

# 运行安装脚本 (使用预编译二进制)
./install_sdk.sh

# 或从源码编译
./install_sdk.sh --build
```

**脚本会自动完成:**
- ✅ 安装 xrobotoolkit_sdk (Python SDK)
- ✅ 配置 LD_LIBRARY_PATH
- ✅ 安装 Python 依赖 (numpy, scipy)

---

### 📋 手动安装步骤

<details>
<summary>展开查看详细步骤</summary>

#### 步骤 1: 安装 ROS2 Humble

```bash
# 添加 ROS2 源
sudo apt update && sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 安装 ROS2
sudo apt update
sudo apt install -y ros-humble-desktop

# 添加到 bashrc (每次终端自动激活)
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

#### 步骤 2: 安装 Python 依赖

**⚠️ 重要：ROS2 使用系统 Python (`/usr/bin/python3`)，不是 conda 环境！**

```bash
# 安装系统级依赖
sudo apt install python3-pip cmake build-essential pybind11-dev

# 安装 Python 依赖到系统 Python
/usr/bin/python3 -m pip install numpy scipy --user
```

#### 步骤 3: 安装 XRoboToolkit PC-Service ⚠️ 重要

**PC-Service 是 PICO 头显与 PC 通信的桥梁，必须先安装！**

```bash
# 1. 下载 deb 包 (使用本项目维护的版本)
cd ~/Downloads
wget https://github.com/lzhu686/XRoboToolkit-PC-Service/releases/download/v1.0.0/XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb

# 2. 安装
sudo dpkg -i XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb

# 3. 验证安装成功
ls -la /opt/apps/roboticsservice/
# 应该看到 runService.sh, RoboticsServiceProcess 等文件

# 4. 测试启动
/opt/apps/roboticsservice/runService.sh
# 看到 "release mode" 表示启动成功！
```

**如果安装失败：**
```bash
sudo apt --fix-broken install
sudo dpkg -i XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb
```

#### 步骤 4: 安装 PICO APK

**最新版本：v1.3.0** （支持全局/本地坐标系选择）

从 GitHub Release 下载并安装：

```bash
# 1. 确保 PICO 连接 USB 并开启开发者模式
#    PICO 设置 → 开发者选项 → 开启 USB 调试

# 2. 如果没有 adb，先安装
sudo apt install android-tools-adb

# 3. 下载并安装 APK（推荐使用 local 本地坐标系版本）

# 本地坐标系版本（推荐 - 更稳定可靠）⭐
wget https://github.com/lzhu686/XRoboToolkit-Unity-Client/releases/download/v1.3.0/XRoboToolkit-v1.3.0-local.apk
adb install -r -g XRoboToolkit-v1.3.0-local.apk

# 或全局坐标系版本（仅在需要多设备空间对齐时使用）
# 注意：global 模式可能因环境特征点丢失导致追踪不稳定
wget https://github.com/lzhu686/XRoboToolkit-Unity-Client/releases/download/v1.3.0/XRoboToolkit-v1.3.0-global.apk
adb install -r -g XRoboToolkit-v1.3.0-global.apk

# 4. 验证安装成功
adb shell pm list packages | grep xrobo
# 应该看到: package:com.xrobotoolkit.client
```

**坐标系模式选择：**

| 模式 | 稳定性 | 适用场景 | 推荐 |
|------|--------|----------|------|
| **Local（本地坐标系）** | ⭐⭐⭐⭐⭐ | 单设备使用、天机臂遥操作、日常开发 | ✅ **推荐** |
| **Global（全局坐标系）** | ⭐⭐⭐ | 多设备空间对齐、需要环境锚点 | 仅特殊场景 |

**说明：**
- **Local 模式**更稳定，相对于设备初始位置的坐标系，不依赖环境特征点追踪
- **Global 模式**依赖环境空间锚点，可能因光照变化、环境遮挡等导致特征点丢失，影响追踪稳定性

#### 步骤 5: 安装 xrobotoolkit_sdk (Pybind)

**方式 A: 使用预编译二进制 (推荐)**

```bash
cd ~/Desktop/wuji-teleop-ros2-private/src/input_devices/pico_input
./install_sdk.sh
```

**方式 B: 从源码编译**

```bash
# 1. 克隆并编译 C++ SDK
cd /tmp
git clone https://github.com/lzhu686/XRoboToolkit-PC-Service.git pc-service
cd pc-service/RoboticsService/PXREARobotSDK
bash build.sh

# 2. 克隆 Pybind SDK (如果没有)
cd ~/Desktop
git clone https://github.com/lzhu686/XRoboToolkit-PC-Service-Pybind.git

# 3. 复制 C++ 库文件到 Pybind 目录
cd ~/Desktop/XRoboToolkit-PC-Service-Pybind
mkdir -p include lib
cp /tmp/pc-service/RoboticsService/PXREARobotSDK/PXREARobotSDK.h include/
cp -r /tmp/pc-service/RoboticsService/PXREARobotSDK/nlohmann include/
cp /tmp/pc-service/RoboticsService/PXREARobotSDK/build/libPXREARobotSDK.so lib/

# 4. 编译并安装 Python SDK
mkdir -p build_user && cd build_user
cmake .. -DCMAKE_LIBRARY_OUTPUT_DIRECTORY=$(pwd)/output -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

# 5. 安装到用户 site-packages
mkdir -p ~/.local/lib/python3.10/site-packages/
cp output/xrobotoolkit_sdk.cpython-310-x86_64-linux-gnu.so ~/.local/lib/python3.10/site-packages/

# 6. 复制共享库到用户 lib 目录
mkdir -p ~/.local/lib
cp ~/Desktop/XRoboToolkit-PC-Service-Pybind/lib/libPXREARobotSDK.so ~/.local/lib/

# 7. 添加库路径到 bashrc
echo 'export LD_LIBRARY_PATH=$HOME/.local/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc
```

**验证安装：**
```bash
/usr/bin/python3 -c "import xrobotoolkit_sdk as xrt; print('SDK 安装成功!')"
```

</details>

---

### 步骤 6: 编译 ROS2 工作空间

```bash
cd ~/Desktop/wuji-teleop-ros2-private
colcon build
source install/setup.bash

# 建议添加到 bashrc，每次自动激活
echo "source ~/Desktop/wuji-teleop-ros2-private/install/setup.bash" >> ~/.bashrc
```

**如果编译失败：**
```bash
# 查看具体错误
colcon build --event-handlers console_direct+

# 常见问题 1: 缺少依赖
rosdep install --from-paths src --ignore-src -r -y

# 常见问题 2: 只编译特定包 (跳过有问题的包)
colcon build --packages-select pico_input wuji_teleop_bringup

# 常见问题 3: 清理后重新编译
rm -rf build install log
colcon build
```

### 步骤 7: 网络配置

**⚠️ 重要：PICO 头显和 PC 必须在同一局域网 (推荐 5GHz WiFi)**

```bash
# 查看 PC IP
hostname -I
# 例如输出: 192.168.1.100

# 在 PICO 头显中:
# 1. 打开 XRoboToolkit App
# 2. 输入 PC IP 地址 (如 192.168.1.100)
# 3. 点击连接
```

### 步骤 8: 启动 Preview 模式验证 ✅

**这是验证所有设备连接正常的关键步骤！**

```bash
# 终端 1: 启动 PC-Service (保持运行)
/opt/apps/roboticsservice/runService.sh

# 终端 2: 启动 Preview 模式 (带 RViz 可视化)
ros2 launch wuji_teleop_bringup pico_teleop.launch.py \
  enable_robot:=false enable_camera:=false enable_hand:=false enable_rviz:=true
```

**在 RViz 中应该看到：**
- `world` 坐标系 (原点)
- `head` (HMD 位置，随头部移动)
- `pico_left_wrist` / `pico_right_wrist` (手腕 Tracker)
- `pico_left_arm` / `pico_right_arm` (大臂 Tracker)
- `world_left` / `world_right` (静态，肩部 chest 坐标系)
**Preview 模式不会控制机器人**，可以安全地验证所有输入设备！

---

## 何时需要重新 colcon build？

| 修改内容 | 是否需要 colcon build |
|---------|---------------------|
| 修改 `.py` 文件 (Python 节点) | ❌ 不需要 (symlink 模式) |
| 修改 `setup.py` / `package.xml` | ✅ 需要 |
| 修改 `.yaml` 配置文件 | ✅ 需要 (或直接用绝对路径) |
| 修改 `launch` 文件 | ✅ 需要 |
| 新增/删除文件 | ✅ 需要 |

**快速重编译单个包：**
```bash
colcon build --packages-select pico_input
source install/setup.bash
```

---

## 数据流

```
PICO → APK → WiFi(TCP:63901) → PC-Service → 共享内存(localhost:60061)
                                                    ↓
                                             pico_input_node
                                                    ↓
                                  TF frames + Topics (target_pose, elbow_direction)
                                                    ↓
                                        tianji_world_output → 天机臂
```

## 输出

### TF (始终启用)

```
world
├── head (HMD，动态)
├── world_left (左臂 Chest 坐标系，绕 X 轴 +90°)
│   ├── pico_left_wrist (Tracker #0，动态)
│   └── pico_left_arm (Tracker #2，动态)
└── world_right (右臂 Chest 坐标系，绕 X 轴 -90°)
    ├── pico_right_wrist (Tracker #1，动态)
    └── pico_right_arm (Tracker #3，动态)
```

### Topics (enable_topic_publishing=true 时启用)

| Topic | Type | 描述 |
|-------|------|------|
| /pico_hmd | PoseStamped | HMD 位姿 (world 坐标系) |
| /pico_left_wrist | PoseStamped | 左手腕位姿 (world_left 坐标系) |
| /pico_right_wrist | PoseStamped | 右手腕位姿 (world_right 坐标系) |
| /left_arm_target_pose | PoseStamped | 左臂目标位姿 (world_left 坐标系) |
| /right_arm_target_pose | PoseStamped | 右臂目标位姿 (world_right 坐标系) |
| /left_arm_elbow_direction | Vector3Stamped | 左臂肘部方向 (臂角约束) |
| /right_arm_elbow_direction | Vector3Stamped | 右臂肘部方向 (臂角约束) |

## 使用方式

### 🔍 Preview 模式 (推荐首次使用)

**在控制机器人之前，先验证所有输入设备！**

```bash
# 终端 1: 启动 PC-Service (保持运行)
/opt/apps/roboticsservice/runService.sh

# 终端 2: 启动 Preview 模式
ros2 launch wuji_teleop_bringup pico_teleop.launch.py \
  enable_robot:=false enable_camera:=false enable_hand:=false enable_rviz:=true
```

**Preview 模式包含：**
- ✅ PICO 输入 (TF 发布)
- ✅ RViz 可视化 (默认开启)
- ✅ MANUS 手套输入 (可选)
- ✅ 摄像头视频流 (可选)
- ❌ 不控制天机臂
- ❌ 不控制舞肌手

**Preview 模式参数：**
```bash
# 完整启动 (PICO + MANUS + Camera + RViz, 不控制机器人)
ros2 launch wuji_teleop_bringup pico_teleop.launch.py \
  enable_robot:=false enable_rviz:=true

# 仅 PICO 追踪 + RViz (无手套、无相机)
ros2 launch wuji_teleop_bringup pico_teleop.launch.py \
  enable_robot:=false enable_rviz:=true enable_hand:=false enable_camera:=false

# 关闭 RViz
ros2 launch wuji_teleop_bringup pico_teleop.launch.py \
  enable_robot:=false enable_rviz:=false

# 关闭手套 (未安装 MANUS 时使用)
ros2 launch wuji_teleop_bringup pico_teleop.launch.py \
  enable_robot:=false enable_rviz:=true enable_hand:=false

# 关闭摄像头 (未连接相机时使用)
ros2 launch wuji_teleop_bringup pico_teleop.launch.py \
  enable_robot:=false enable_rviz:=true enable_camera:=false
```

**pico_teleop.launch.py 参数说明：**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `enable_robot` | true | 启用天机臂控制 |
| `enable_hand` | true | 启用 MANUS 手套输入 |
| `enable_camera` | true | 启用立体相机视频流 |
| `enable_rviz` | false | 启用 RViz 可视化 |

### 🤖 完整遥操作模式 (控制机器人)

**确保 Preview 模式验证通过后再使用！**

```bash
# 终端 1: 启动 PC-Service
/opt/apps/roboticsservice/runService.sh

# 终端 2: 启动完整遥操作 (双臂 + 灵巧手 + 相机)
ros2 launch wuji_teleop_bringup pico_teleop.launch.py
```

### 🦾 Minimal 模式 (仅双臂，无手/相机)

**推荐首次测试使用！** 交互式启动，自动检测设备就绪。

```bash
# 终端 1: 启动 PC-Service
/opt/apps/roboticsservice/runService.sh

# 终端 2: Minimal 模式 (仅 pico_input + tianji_world_output)
ros2 launch wuji_teleop_bringup pico_teleop_minimal.launch.py

# 或: 使用录播数据 (无需 PICO 硬件)
ros2 launch wuji_teleop_bringup pico_teleop_minimal.launch.py \
  data_source_type:=recorded playback_speed:=0.3
```

### 仅测试 PICO 连接 (最小模式)

```bash
# 终端 1: 启动 PC-Service
/opt/apps/roboticsservice/runService.sh

# 终端 2: 仅启动 PICO 输入节点 (不控制机器人)
ros2 launch pico_input pico_input.launch.py

# 终端 3: 验证 TF
ros2 run tf2_tools view_frames
evince frames.pdf
```

---

## 运行模式对比

| 模式 | Launch 文件 | 用途 | 控制机器人 |
|------|-------------|------|-----------|
| **最小测试** | `pico_input.launch.py` | 仅测试 PICO 连接 (TF) | ❌ |
| **Preview** | `pico_teleop.launch.py enable_robot:=false enable_rviz:=true` | 验证输入设备 + RViz 可视化 | ❌ |
| **Minimal** | `pico_teleop_minimal.launch.py` | 仅双臂控制 (交互式启动) | ✅ 仅臂 |
| **完整遥操** | `pico_teleop.launch.py` | 双臂 + 灵巧手 + 相机 | ✅ 全部 |

**推荐测试流程：** 最小测试 → Preview → Minimal → 完整遥操

### Preview 模式详解

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    Preview 模式数据流 (仅输入，无输出)                   │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  PICO SDK ──▶ pico_input_node ──▶ TF ──▶ RViz (可视化)                 │
│                                      ❌ 不发送到 tianji_world_output     │
│                                                                         │
│  MANUS ──▶ manus_input ──▶ /hand_input ──▶ (可视化)                    │
│                                      ❌ 不发送到 wujihand_retargeting   │
│                                                                         │
│  USB Camera ──▶ stereovr_main ──▶ PICO VR (H.264视频流, 60fps)          │
│                               ──▶ v4l2loopback (/dev/video99)           │
│                                        ↓                                │
│                               stereovr_publisher ──▶ ROS2 Topics        │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

### Preview 模式检查清单

在 Preview 模式下验证以下内容：

| 检查项 | 验证方法 | 预期结果 |
|--------|----------|----------|
| TF 树正确 | RViz 中查看 Axes | 看到 head, left_wrist, right_wrist, left_arm, right_arm |
| Tracker 追踪正常 | 移动手臂 | TF 帧跟随移动 |
| HMD 朝向正确 | 转头 | head 帧跟随旋转 |
| (可选) 手套数据 | `ros2 topic echo /hand_input` | 看到关节角数据 |
| (可选) 摄像头 | `ros2 topic hz /stereo/left/compressed` | 帧率 > 25 fps |

---

## 4 个 PICO Tracker 与机械臂的对应关系

### 整体架构图

```
                    用户                                      机器人
              ┌─────────────┐                           ┌─────────────┐
              │    头显     │ ─(HMD)── 仅可视化 ──→     │    基座     │
              │   (PICO)    │                           │   (world)   │
              └─────────────┘                           └─────────────┘
                    │                                         │
    ┌───────────────┼───────────────┐         ┌───────────────┼───────────────┐
    │               │               │         │               │               │
    ▼               ▼               ▼         ▼               ▼               ▼
┌───────┐      ┌───────┐       ┌───────┐   ┌───────┐     ┌───────┐      ┌───────┐
│Tracker│      │Tracker│       │Tracker│   │ 左肩  │     │ 右肩  │      │  头   │
│  #2   │      │  #0   │       │  #1   │   │ left  │     │ right │      │(可视化)│
│左大臂 │      │左手腕 │       │右手腕 │   │_chest │     │_chest │      │       │
└───┬───┘      └───┬───┘       └───┬───┘   └───┬───┘     └───┬───┘      └───────┘
    │              │               │           │             │
    ▼              ▼               ▼           ▼             ▼
┌───────┐      ┌───────┐       ┌───────┐   ┌───────┐     ┌───────┐
│Tracker│      │       │       │       │   │ 左臂  │     │ 右臂  │
│  #3   │      │       │       │       │   │7 自由 │     │7 自由 │
│右大臂 │      │       │       │       │   │  度   │     │  度   │
└───────┘      └───────┘       └───────┘   └───────┘     └───────┘
```

### Tracker 详细对应表

| Tracker | 佩戴位置 | 对应机器人 | 提取信息 | 用途 |
|---------|---------|-----------|---------|------|
| HMD | 头部 | - | 位置 + 姿态 (增量) | 仅 RViz 可视化 |
| #0 | 左手腕 | 左臂末端 | 位置 + 姿态 | 控制机器人左手位姿 |
| #1 | 右手腕 | 右臂末端 | 位置 + 姿态 | 控制机器人右手位姿 |
| #2 | 左大臂 | 左肘 (J4) | **位置方向** | IK 臂角控制 (zsp_para) |
| #3 | 右大臂 | 右肘 (J4) | **位置方向** | IK 臂角控制 (zsp_para) |

### 头显 (HMD) 的作用

**头显仅用于 RViz 可视化，不控制机器人！**

```
HMD 使用增量控制模式，与 Tracker 相同：
- 初始化时记录 HMD 初始位姿
- 运行时发布相对于初始位姿的增量变化
- 在 RViz 中显示用户头部位置 (head frame)
```

**注意**: 当前实现使用固定坐标变换 (PICO→机器人)，用户初始化时应面向机器人前方。

### 手腕 Tracker (#0, #1) 的作用

控制机器人手的**完整位姿** [x, y, z, qx, qy, qz, qw]：

```python
# 位置增量
delta_pos = 用户当前手位置 - 用户初始手位置
机器人目标位置 = 机器人初始位置 + delta_pos

# 姿态增量  
delta_rot = 用户当前手姿态 × 用户初始手姿态⁻¹
机器人目标姿态 = 机器人初始姿态 × delta_rot
```

### 大臂 Tracker (#2, #3) 的作用

**不是控制"肘部位置"，而是控制"肘部方向"！**

7 自由度机械臂是冗余的：同一个手的位姿，肘部可以有无数种姿态。

```
        肩 ●─────────────────────● 手 (固定位姿)
            ╲                   ╱
             ╲                 ╱
              ● 肘可以在这条弧线上移动
               ╲             ╱
                ●           ●
                 ╲         ╱
                  ●───────●
                    臂角 φ
```

大臂 Tracker 的 **Y 轴方向** 告诉 IK：肘部应该往哪边。

```python
# 从 Tracker 姿态提取 Y 轴方向向量
y_axis = tracker_rotation_matrix[:, 1]  # 第二列

# 作为 IK 零空间参数
zsp_para = [y_axis.x, y_axis.y, y_axis.z, 0, 0, 0]
```

---

## 增量控制模式详解

### 为什么需要增量控制？

**问题：** 用户站的位置、姿势和机器人不一样。如果直接把用户手的位置发给机器人，机器人会突然跳到一个奇怪的位置。

**解决：** 只发送"变化量"，不发送"绝对位置"。

```
初始化时:
    用户手在 PICO 坐标系的位置: init_tracker_pos = [0.5, 1.2, -0.3]
    机器人手的初始位置:        robot_init_pos  = [0.5061, 0.1837, 0.1319]

运行时:
    用户手移动到:              current_pos = [0.6, 1.25, -0.2]
    变化量:                    delta = current_pos - init_tracker_pos
                                     = [0.1, 0.05, 0.1]
    机器人目标位置:            target = robot_init_pos + delta
                                     = [0.6061, 0.2337, 0.2319]
```

### 初始化过程

```
┌─────────────────────────────────────────────────────────────────────┐
│ 步骤 1: 机器人移动到初始关节角 init_joints                           │
│         此时末端位置 = get_init_pose.py 计算的值                     │
└─────────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────────┐
│ 步骤 2: 用户戴上 PICO 头显和 Tracker                                 │
│         建议用户摆出类似机器人的姿势 (非必须，但更直观)               │
└─────────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────────┐
│ 步骤 3: 等待 5 秒 (或手动调用 /pico_input/init)                      │
│         系统记录:                                                    │
│           - HMD 初始位姿 (仅用于可视化)                              │
│           - 所有 Tracker 的初始位姿                                  │
└─────────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────────┐
│ 步骤 4: 开始遥操作                                                   │
│         用户移动手 → 发布增量 → 机器人跟随移动                       │
└─────────────────────────────────────────────────────────────────────┘
```

### 增量控制公式汇总

| 数据 | 公式 | 说明 |
|------|------|------|
| 手位置 | `target = robot_init_pos + (current - init)` | 1:1 跟随 (经坐标变换) |
| 手姿态 | `target = robot_init_rot × transform(current × init⁻¹)` | 1:1 跟随 (轴角方法) |
| 臂角 | `zsp_para = normalize(arm_position)` | 位置向量归一化 |

---

## 配置

配置文件: `config/pico_input.yaml`

| 参数 | 默认值 | 描述 |
|------|--------|------|
| publish_rate | 90.0 | 发布频率 (Hz) |
| pc_service_host | 127.0.0.1 | PC-Service 地址 |
| pc_service_port | 60061 | PC-Service 端口 |
| enable_topic_publishing | true | 发布 /left_arm_target_pose 等 Topic |
| enable_legacy_topics | false | 发布 /pico/* 调试 Topic |
| topic_prefix | pico | Legacy Topic 前缀 |
| auto_init_delay | 5.0 | 自动初始化延迟 (秒), 0 禁用 |
| pos_ema_alpha | 0.6 | 位置 EMA 平滑系数 |
| elbow_dir_ema_alpha | 1.0 | 臂角方向 EMA 平滑系数 |
| elbow_gray_zone | 0.015 | 臂角灰区阈值 (m) |
| data_source_type | live | 数据源: "live" 或 "recorded" |
| recorded_file_path | (见 YAML) | 录播文件路径 |
| playback_speed | 1.0 | 录播速度倍率 |
| loop_playback | true | 是否循环录播 |
| enable_debug_log | false | 启用 CSV 调试日志 |
| tracker_serial_XXXXXX | (见 YAML) | Tracker 序列号 → 角色映射 |

---

## 坐标系说明

### 天机机器人坐标系 (右手系)

```
        Z (上)
        ↑
        │
        │
        └───────→ Y (左)
       ╱
      ╱
     X (前，机器人面向方向)
```

### OpenXR / PICO 坐标系 (右手系)

```
        Y (上)
        ↑
        │
        │
        └───────→ X (右)
       ╱
      ╱
     Z (前，用户面向方向)
```

### 坐标变换 (PICO → 机器人)

**位置变换:**
```
robot_x = -pico_z  (前: 用户前移 → 机器人前移)
robot_y = -pico_x  (左 = -右)
robot_z =  pico_y  (上)
```

**变换矩阵:**
```python
pico_to_robot = np.array([
    [0, 0, -1],  # Robot X = -PICO Z
    [-1, 0, 0],  # Robot Y = -PICO X
    [0, 1, 0]    # Robot Z = PICO Y
])
# 注意: det(pico_to_robot) = +1 (正交矩阵，无镜像)
```

**旋转映射 (使用轴角方法):**
| PICO 旋转轴 | 用户动作 | 机器人旋转轴 |
|------------|----------|-------------|
| 绕 X 轴 | 向右倾斜手 | 绕 -Y 轴 (向右倾斜) |
| 绕 Y 轴 | 抬起/放下手腕 | 绕 Z 轴 |
| 绕 Z 轴 | 转动手腕 | 绕 X 轴 |

> **技术细节**: 代码使用**轴角方法**: 变换旋转轴向量，保持旋转角度不变。
> pico_to_robot 矩阵 det=+1（两个轴取反相互抵消），是正交矩阵。

---

## 机器人初始位姿 (FK 计算结果)

对应天机初始关节角（从 `tianji_robot.yaml` 加载）：
```python
INIT_JOINTS_LEFT  = [55.0, -65.0, -70.0, -60.0, 60.0, 0.0, 0.0]  # 度
INIT_JOINTS_RIGHT = [-55.0, -65.0, 70.0, -60.0, -60.0, 0.0, 0.0]  # 度
```

### 末端位姿 (手腕)

| 手臂 | 位置 [x, y, z] 米 | 姿态 [qx, qy, qz, qw] |
|------|-------------------|----------------------|
| 左臂 | `[0.5733, 0.2237, 0.2762]` | `[0.0067, 0.7270, 0.0111, 0.6865]` |
| 右臂 | `[0.5733, -0.2237, 0.2762]` | `[-0.0067, 0.7270, -0.0111, 0.6865]` |

### Arm Tracker 参考位置 (Chest 坐标系, 用于增量控制基准)

| 手臂 | 位置 [x, y, z] 米 | 姿态 [qx, qy, qz, qw] |
|------|-------------------|----------------------|
| 左臂 | `[0.2, 0.3, 0.2]` | `[0.4177, -0.0283, 0.5206, 0.7441]` |
| 右臂 | `[0.2, -0.3, 0.2]` | `[0.7448, -0.5141, 0.0058, 0.4254]` |

**物理含义:** 肘部在肩膀下方30cm + 外侧20cm (沉肘+外展姿态)
- Left Chest: Y+=下, Z+=左(外侧) → `[0.2, +0.3, +0.2]`
- Right Chest: Y-=下, Z+=右(外侧) → `[0.2, -0.3, +0.2]`

### 如何重新计算初始位姿

当修改初始关节角后，需要重新计算末端位姿：

```bash
cd src/output_devices/tianji_world_output/tianji_world_output
python3 get_init_pose.py
```

将输出的值更新到 `tianji_world_output/config/tianji_robot.yaml` 中的 `init_pos`、`init_rot` 和 `init_quat` 字段。

---

## DH 参数通俗解释

DH (Denavit-Hartenberg) 参数是描述机械臂"骨架结构"的标准方法。

### 把机械臂想象成搭积木

```
       ╔═══╗
       ║ 7 ║  ← 末端 (手腕)
       ╚═╤═╝
         │ 
       ╔═╧═╗
       ║ 6 ║  ← 手腕关节
       ╚═╤═╝
         │ d5=314mm (前臂长度)
       ╔═╧═╗
       ║ 4 ║  ← 肘关节 (J4) ← Tracker #2, #3 对应这里!
       ╚═╤═╝
         │ d3=287mm (上臂长度)
       ╔═╧═╗
       ║ 2 ║  ← 肩关节
       ╚═╤═╝
         │ d1=174.5mm (肩高)
       ╔═╧═╗
       ║ 1 ║  ← 基座
       ╚═══╝
```

### DH 参数表 (天机 M6-CCS)

| 关节 | a (mm) | α (°) | d (mm) | θ (°) | 含义 |
|-----|--------|-------|--------|-------|------|
| 1 | 0 | 0 | **174.5** | 0 | 基座到肩膀高度 |
| 2 | 0 | 90 | 0 | 0 | 肩膀水平旋转 |
| 3 | 0 | -90 | **287** | 0 | 上臂长度 |
| 4 | 18 | 90 | 0 | 180 | **肘关节** |
| 5 | 18 | 90 | **314** | 180 | 前臂长度 |
| 6 | 0 | 90 | 0 | 90 | 手腕关节 |
| 7 | 0 | 90 | 0 | 90 | 手腕末端 |

### 四个参数的含义

```
a  = 沿 X 轴移动 (连杆水平偏移)
α  = 绕 X 轴旋转 (关节扭转角)
d  = 沿 Z 轴移动 (连杆垂直高度) ← 这就是"骨头长度"
θ  = 绕 Z 轴旋转 (关节角度) ← 这是你控制的变量!
```

简单记忆：
- **d** = 骨头有多长 (上臂 287mm, 前臂 314mm)
- **θ** = 关节转了多少度 (控制变量)

---

## Tracker 佩戴与配置

| 索引 | 佩戴位置 | TF Frame | 提取数据 |
|------|----------|----------|----------|
| HMD  | 头部     | head     | 增量位姿 (仅可视化) |
| #0   | 左手腕   | pico_left_wrist | 位置 + 姿态 |
| #1   | 右手腕   | pico_right_wrist | 位置 + 姿态 |
| #2   | 左上臂   | pico_left_arm | **仅位置** |
| #3   | 右上臂   | pico_right_arm | **仅位置** |

### Tracker 佩戴建议

**手腕 Tracker (#0, #1):**
- 佩戴在手腕背面
- 控制机器人手的 **完整位姿** (位置 + 姿态)

**大臂 Tracker (#2, #3):**
- 佩戴在上臂外侧，肘关节上方约 10-15cm
- 只使用 **位置信息**，与 Tracker 旋转姿态无关
- 用于控制机械臂的臂角 (肘部抬高/放低)

### 大臂 Tracker 佩戴位置示意图

```
        肩膀
          ●
         /|
        / |
       /  |
      /   | 上臂
     /    |
    ●     |  ←── Tracker (#2/#3) 佩戴在这里 (上臂外侧)
     \    |
      \   |
       \  |
        \ |
         \|
          ●  肘
          |
          |  前臂
          |
          ●  手腕
          |
          |  ←── Tracker (#0/#1) 佩戴在手腕
          ●  手掌
```

### 位置向量与臂角控制

**大臂 Tracker 只使用位置，不使用姿态！**

```
数据流:

1. pico_input_node 计算肘部方向:
   shoulder → wrist 向量作为轴，arm tracker 位置投影到垂直平面
   得到肘部偏移方向 (几何方向 → 取反后发布给 IK)

2. 发布到 Topic: /left_arm_elbow_direction (Vector3Stamped)

3. tianji_world_output_node 订阅此 Topic:
   zsp_para = [dir.x, dir.y, dir.z, 0, 0, 0]
   告诉 IK 解算器肘部应该朝这个方向
```

**臂角控制示意图：**

```
                肩部 (world_left)
                  ●
                 /|
                / |
    肘部向上   /  |    肘部向下
              /   |          \
             ●    |           ●
    Tracker在这里 |     Tracker在这里
                  |
                  |
                  ●
                 手

  arm_direction   |   arm_direction
  指向左上方      |   指向左下方
```

这就是 `zsp_para` (Zero Space Parameter，零空间参数) 的工作原理：
- 7 自由度机械臂在到达同一手部位姿时，肘部可以有无限种姿态
- **位置向量方向** 告诉 IK 解算器肘部应该往哪边
- 与原开发者描述一致："肩部到肘关节的向量"

---

## 故障排查

### PC-Service 检查

```bash
# 检查 PC-Service 是否安装
ls -la /opt/apps/roboticsservice/runService.sh

# 启动 PC-Service
/opt/apps/roboticsservice/runService.sh
# 看到 "release mode" 表示启动成功！

# 检查 PC-Service 进程是否运行
ps aux | grep -i robotics

# 检查 gRPC 端口是否监听
netstat -tlnp | grep 60061
# 或
ss -tlnp | grep 60061
```

### PICO 连接检查

```bash
# 检查 PICO 网络连接 (需要知道 PICO IP)
ping <PICO_IP>

# 测试 SDK 连接 (需要 PC-Service 运行)
LD_LIBRARY_PATH=$HOME/.local/lib:$LD_LIBRARY_PATH /usr/bin/python3 -c "
import xrobotoolkit_sdk as xrt
xrt.init()
print('Trackers:', xrt.num_motion_data_available())
print('HMD Pose:', xrt.get_headset_pose())
"
```

### ROS2 检查

```bash
# 查看 TF 树
ros2 run tf2_tools view_frames
evince frames.pdf

# 检查节点是否正常
ros2 node list

# 检查话题
ros2 topic list

# 查看 TF 数据流
ros2 topic echo /tf --once
```

### 常见问题

| 问题 | 可能原因 | 解决方案 |
|------|----------|----------|
| `libManusSDK_Integrated.so: file format not recognized` | Git LFS 文件未下载 | 执行 `git lfs install && git lfs pull` |
| `/opt/apps/roboticsservice/runService.sh: No such file` | PC-Service 未安装 | 按步骤 3 安装 deb 包 |
| 无法连接 PC-Service | 服务未启动 | 运行 `/opt/apps/roboticsservice/runService.sh` |
| TF 无数据 | PICO 未连接 | 检查 PICO App 是否显示已连接 |
| 找不到包 | 未编译/未 source | `colcon build && source install/setup.bash` |
| MANUS 无数据 | MANUS Core 未运行 | 启动 MANUS Core 软件 |
| `No module named 'xrobotoolkit_sdk'` | Pybind SDK 未安装 | 按步骤 5 安装 xrobotoolkit_sdk |
| `cannot find -lPXREARobotSDK` | C++ 库未编译 | 先编译 PC-Service 的 PXREARobotSDK |
| `Found zero norm quaternions` | PICO 未连接/无数据 | 确保 PICO 头显已连接并运行 XRoboToolkit App |
| `No module named 'scipy'` | scipy 未安装 | `/usr/bin/python3 -m pip install scipy --user` |
| `pip3` 安装到 conda 而非系统 | conda 环境激活 | 使用 `/usr/bin/python3 -m pip install --user` |

### get_init_pose.py 错误修复

**问题 1: `ModuleNotFoundError: No module named 'scipy'`**
```bash
pip install scipy
```

**问题 2: `NameError: name 'np' is not defined`**

确保 `get_init_pose.py` 顶部有：
```python
import numpy as np
```

### Git LFS 文件未正确下载

**症状：** 编译时出现 `file format not recognized` 或链接错误

**原因：** 本仓库使用 Git LFS 存储大型二进制文件（如 Manus SDK 库）。如果克隆时未正确拉取 LFS 文件，这些文件会是文本指针而非实际二进制。

**解决步骤：**
```bash
# 1. 进入仓库目录
cd ~/Desktop/wuji-teleop-ros2-private

# 2. 安装 Git LFS（如果未安装）
sudo apt install git-lfs

# 3. 初始化并拉取 LFS 文件
git lfs install
git lfs pull

# 4. 验证文件类型
file src/input_devices/manus_input/manus_ros2/ManusSDK/lib/libManusSDK_Integrated.so
# ✅ 正确: ELF 64-bit LSB shared object...
# ❌ 错误: ASCII text (需要重新 git lfs pull)

# 5. 重新编译
colcon build --symlink-install
```

---

## 文件结构

```
pico_input/
├── install_sdk.sh                   # ⚡ 一键安装脚本
├── prebuilt/                        # 预编译二进制
│   └── x86_64/
│       ├── libPXREARobotSDK.so
│       └── xrobotoolkit_sdk.cpython-310-x86_64-linux-gnu.so
├── config/pico_input.yaml           # 配置文件
├── launch/pico_input.launch.py      # Launch 文件
├── pico_input/
│   ├── pico_input_node.py           # ROS2 节点 (增量控制，修改后无需 rebuild)
│   ├── incremental_controller.py    # 增量控制器 (纯计算，无 ROS2 依赖)
│   ├── xrobotoolkit_client.py       # Pybind SDK 封装
│   └── data_source/                 # 数据源抽象层 + 实现
│       ├── __init__.py              # re-export (DataSource, TrackerData, HeadsetData)
│       ├── base.py                  # ABC 基类 (DataSource, TrackerData, HeadsetData)
│       ├── live_data_source.py      # 真实 PICO SDK 数据源
│       └── recorded_data_source.py  # 录制文件回放数据源
├── test/
│   ├── common/                      # 共享模块
│   │   ├── robot_config.py          # 测试兼容层 (从 tianji_robot.yaml 加载)
│   │   ├── robot_lifecycle.py       # 机器人上电/下电生命周期管理
│   │   └── transform_utils.py       # Re-export wrapper → tianji_world_output
│   ├── docs/
│   │   └── PICO_TELEOP_GUIDE.md     # 遥操作完整指南
│   ├── tool/                        # 工具脚本
│   │   ├── move_to_init_pose.py     # 移动机器人到初始姿态
│   │   ├── verify_fk_values.py      # 验证 FK 计算值
│   │   └── diagnose_zsp_para.py     # FK→IK 闭环诊断 zsp_para
│   ├── step1_direct_joint_control.py     # 直接关节控制
│   ├── step2_pose_topic_control.py       # ROS2 Topic 位姿控制
│   ├── step3_visualize_in_rviz.py        # RViz 坐标系可视化
│   ├── step4_visualize_recorded_data.py  # PICO 录制数据可视化
│   ├── step5_incremental_control_with_robot.py  # PICO 增量控制真机
│   └── step6_arm_angle_stability_test.py  # 臂角稳定性测试
├── setup.py                         # 包配置 (修改后需 rebuild)
├── package.xml                      # ROS2 包描述 (修改后需 rebuild)
└── README.md
```

---

## 相关包依赖

完整遥操作 (`pico_teleop.launch.py`) 需要以下包：

| 包名 | 功能 | 来源 |
|------|------|------|
| `pico_input` | PICO 数据采集 | 本包 |
| `manus_ros2` | MANUS 驱动 | src/input_devices/manus_input/ |
| `manus_input_py` | MANUS 数据转换 | src/input_devices/manus_input/ |
| `tianji_world_output` | 天机臂控制 (World 坐标系) | src/output_devices/tianji_world_output/ |
| `wujihand_ik` | 舞肌手 IK | src/wujihand_ik/ |
| `tf2_ros` | TF 变换 | ros-humble-tf2-ros (apt 安装) |

**一次性编译所有包：**
```bash
cd ~/Desktop/wuji-teleop-ros2-private
colcon build
source install/setup.bash
```

---

## 🔄 完整数据流架构

### 系统架构图

```
┌─────────────────────────────────────────────────────────────────────────────────────┐
│                                    用户端                                            │
├─────────────────────────────────────────────────────────────────────────────────────┤
│                                                                                     │
│   ┌──────────┐    ┌──────────┐    ┌──────────┐    ┌──────────┐    ┌──────────┐    │
│   │   HMD    │    │Tracker#0 │    │Tracker#1 │    │Tracker#2 │    │Tracker#3 │    │
│   │ (头显)   │    │(左手腕)   │    │(右手腕)   │    │(左大臂)   │    │(右大臂)   │    │
│   └────┬─────┘    └────┬─────┘    └────┬─────┘    └────┬─────┘    └────┬─────┘    │
│        │               │               │               │               │           │
│        └───────────────┴───────────────┴───────────────┴───────────────┘           │
│                                        │ 蓝牙/USB                                  │
│                                        ▼                                            │
│                               ┌────────────────┐                                    │
│                               │   PICO 头显    │                                    │
│                               │ XRoboToolkit   │                                    │
│                               │     App        │                                    │
│                               └───────┬────────┘                                    │
│                                       │                                             │
└───────────────────────────────────────┼─────────────────────────────────────────────┘
                                        │ TCP (WiFi, 端口 63901)
                                        ▼
┌───────────────────────────────────────────────────────────────────────────────────┐
│                                    PC 端                                           │
├───────────────────────────────────────────────────────────────────────────────────┤
│                                                                                   │
│   ┌──────────────────────────────────┐                                           │
│   │    XRoboToolkit PC-Service        │ ← TCP Server (端口 63901)               │
│   │  接收 PICO 原始追踪数据            │                                           │
│   └────────────────┬─────────────────┘                                           │
│                    │ 共享内存 (localhost:60061)                                   │
│                    ▼                                                              │
│   ┌──────────────────────────────────┐                                           │
│   │        pico_input_node            │ ← ROS2 节点 (90Hz)                        │
│   │  ┌─────────────────────────────┐ │                                           │
│   │  │ 1. 读取 PICO 原始数据       │ │                                           │
│   │  │ 2. 初始化: 记录 HMD 初始位姿│ │                                           │
│   │  │         记录 Tracker 初始位姿│ │                                           │
│   │  │ 3. 计算增量位姿             │ │ ← controller.compute_target_pose()       │
│   │  │    delta = current - init   │ │                                           │
│   │  │    target = robot_init+delta│ │                                           │
│   │  │ 4. 发布 TF + Topics          │ │                                           │
│   │  └─────────────────────────────┘ │                                           │
│   └────────────────┬─────────────────┘                                           │
│                    │                                                              │
│                    │ TF 发布 (pico_input_node 内部):                              │
│                    │   world → world_left/right (静态, chest 坐标系)             │
│                    │   world_left/right → pico_*_wrist (动态, 手腕位姿)          │
│                    │   world_left/right → pico_*_arm (动态, 肘部方向)            │
│                    │   world → head (动态, 仅可视化)                             │
│                    │                                                              │
│                    │ Topics 发布:                                                 │
│                    │   /left_arm_target_pose, /right_arm_target_pose             │
│                    │   /left_arm_elbow_direction, /right_arm_elbow_direction     │
│                    ▼                                                              │
│   ┌──────────────────────────────────┐                                           │
│   │    tianji_world_output_node      │ ← ROS2 节点 (90Hz)                        │
│   │  ┌─────────────────────────────┐ │                                           │
│   │  │ 1. 订阅 Topics:            │ │                                           │
│   │  │    /left_arm_target_pose   │ │ ← 手臂目标位姿 (chest 坐标系)             │
│   │  │    /left_arm_elbow_direction│ │ ← 肘部方向 (zsp_para)                    │
│   │  │                             │ │                                           │
│   │  │ 2. 更新 zsp_para            │ │ ← 从 elbow_direction topic              │
│   │  │                             │ │                                           │
│   │  │ 3. IK 逆运动学计算          │ │ ← controller.move_to_pose_direct()       │
│   │  │    输入: 末端位姿 + zsp_para│ │                                           │
│   │  │    输出: 7个关节角度        │ │                                           │
│   │  │                             │ │                                           │
│   │  │ 4. 发布关节指令             │ │                                           │
│   │  └─────────────────────────────┘ │                                           │
│   └────────────────┬─────────────────┘                                           │
│                    │                                                              │
│                    │ ROS2 Topics:                                                 │
│                    │   /tianji_arm/left/joint_command                            │
│                    │   /tianji_arm/right/joint_command                           │
│                    ▼                                                              │
└────────────────────┼──────────────────────────────────────────────────────────────┘
                     │ 网络 (ROS2 Topic / 自定义协议)
                     ▼
┌───────────────────────────────────────────────────────────────────────────────────┐
│                                   机器人端                                         │
├───────────────────────────────────────────────────────────────────────────────────┤
│   ┌──────────────────────────────────┐                                           │
│   │         天机臂驱动                │                                           │
│   │   接收关节角指令, 驱动电机        │                                           │
│   │                                  │                                           │
│   │   关节 1-7 同时运动               │                                           │
│   └──────────────────────────────────┘                                           │
└───────────────────────────────────────────────────────────────────────────────────┘
```

---

## 🔑 关键数据变换详解

### 阶段 1: pico_input_node 初始化

当用户按下初始化（或等待 5 秒自动初始化）时：

```python
# 1. 记录 HMD 初始位姿 (仅用于可视化)
self.init_hmd_pose = self._pose_to_matrix(hmd_pose)

# 2. 记录每个 Tracker 的初始 4x4 变换矩阵
for role in ['left_wrist', 'right_wrist', 'left_arm', 'right_arm']:
    self.init_tracker_poses[role] = self._pose_to_matrix(tracker_pose)
```

**增量控制原理：**
- 初始化时记录所有 Tracker 的位置和姿态
- 运行时只计算相对于初始状态的变化量
- 用户手移动多少，机器人手就移动多少

### 阶段 2: pico_input_node 增量计算

每帧（90Hz）计算目标位姿：

```python
# ==================== 位置增量 ====================
# 1. 计算用户手在 PICO 坐标系中的位移
delta_pos_pico = current_T[:3, 3] - init_T[:3, 3]

# 2. 将位移转换到机器人坐标系
delta_pos_robot = pico_to_robot @ delta_pos_pico

# 3. 叠加到机器人初始位置
target_pos = robot_init_pos + delta_pos_robot

# ==================== 姿态增量 (轴角方法) ====================
# 1. 计算用户手姿态的变化量 (在 PICO 坐标系)
delta_rot_pico = current_rot * init_rot.inv()

# 2. 使用轴角方法变换旋转
#    注意: 使用轴角方法变换旋转（pico_to_robot det=+1）
rotvec = delta_rot_pico.as_rotvec()
angle = np.linalg.norm(rotvec)
axis = rotvec / angle
axis_robot = pico_to_robot @ axis  # 变换旋转轴
delta_rot_robot = R.from_rotvec(axis_robot * angle)

# 3. 应用到机器人初始姿态
target_rot = robot_init_rot * delta_rot_robot
target_quat = target_rot.as_quat()
```

**为什么使用轴角方法？**
- `pico_to_robot` 矩阵是坐标轴映射矩阵（det=+1，正交矩阵）
- 轴角方法直接变换旋转轴向量，概念清晰且高效
- 等效于 `R.from_matrix(pico_to_robot) * delta * R.from_matrix(pico_to_robot).inv()`，但避免了多余的矩阵运算

### 阶段 3: TF 树结构

```
                            world (固定，机器人基座)
                              │
    ┌─────────────────────────┼─────────────────────────┐
    ↓                         ↓                         ↓
world_left              world_right                   head
 (静态,chest)            (静态,chest)                (动态)
    │                         │
    ├── pico_left_wrist       ├── pico_right_wrist
    │   (动态,手腕)           │   (动态,手腕)
    └── pico_left_arm         └── pico_right_arm
        (动态,肘部)               (动态,肘部)
```

注: `left_dh_ee`/`right_dh_ee` 仅由测试脚本 (step3/step4) 发布，不属于生产节点。

### 🎯 Tracker 数据使用差异 (关键！)

**虽然所有 Tracker 都发布完整位姿 (xyz + quaternion)，但下游使用方式不同：**

| Tracker | 发布内容 | **实际使用** | 用途 |
|---------|----------|-------------|------|
| **pico_left/right_wrist** | 完整位姿 | **位置 + 姿态** | IK 末端目标 |
| **pico_left/right_arm** | 完整位姿 | **仅位置 → 归一化方向** | zsp_para (肘部臂角) |

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        数据使用差异详解                                  │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  手腕 Tracker (#0, #1):                                                 │
│    ┌───────────────────────────────────────────────────────────────┐   │
│    │ 输入: PICO 原始位姿 [x, y, z, qx, qy, qz, qw]                 │   │
│    │                     ↓                                         │   │
│    │ 增量计算: delta_pos = current_pos - init_pos                  │   │
│    │           delta_rot = current_rot × init_rot⁻¹                │   │
│    │                     ↓                                         │   │
│    │ 输出: 目标位姿 = robot_init + delta                           │   │
│    │       [x, y, z, qx, qy, qz, qw] ← 位置 + 姿态都用!            │   │
│    │                     ↓                                         │   │
│    │ IK 输入: 末端执行器的完整 6-DOF 位姿                          │   │
│    └───────────────────────────────────────────────────────────────┘   │
│                                                                         │
│  大臂 Tracker (#2, #3):                                                 │
│    ┌───────────────────────────────────────────────────────────────┐   │
│    │ 输入: PICO 原始位姿 [x, y, z, qx, qy, qz, qw]                 │   │
│    │                     ↓                                         │   │
│    │ 增量计算: delta_pos = current_pos - init_pos                  │   │
│    │           delta_rot = current_rot × init_rot⁻¹                │   │
│    │                     ↓                                         │   │
│    │ 输出: 目标位姿 = robot_init + delta                           │   │
│    │       [x, y, z, qx, qy, qz, qw]                               │   │
│    │                     ↓                                         │   │
│    │ pico_input_node 计算肘部方向:                                  │   │
│    │   shoulder→wrist 向量作为轴，arm 位置投影到垂直平面           │   │
│    │   得到几何偏移方向，取反后发布给 IK                            │   │
│    │                     ↓                                         │   │
│    │ 发布: /left_arm_elbow_direction (Vector3Stamped)              │   │
│    │ IK 输入: zsp_para = [dir.x, dir.y, dir.z, 0, 0, 0]           │   │
│    └───────────────────────────────────────────────────────────────┘   │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

**为什么大臂 Tracker 只用位置？**

1. **目的不同**：大臂 Tracker 控制的是 "肘部在哪个方向"，而不是 "肘部的姿态"
2. **更鲁棒**：只依赖 Tracker 位置，不依赖佩戴时的旋转姿态
3. **几何计算**：pico_input_node 通过 shoulder→wrist 轴和 arm 位置的垂直投影计算偏移方向

```python
# pico_input_node.py 中的关键数据流

# 手腕: pico_input 计算目标位姿，发布到 /left_arm_target_pose
pos, quat = controller.compute_target_pose(pose, role)
# → [x, y, z, qx, qy, qz, qw] 通过 PoseStamped Topic 发给 tianji_world_output

# 大臂: pico_input 计算肘部方向，发布到 /left_arm_elbow_direction
direction, proj_point = controller.compute_elbow_direction(
    shoulder_pos, wrist_pos, elbow_pos, side
)
ik_direction = -direction  # 几何方向取反 → IK 反重力方向
# → [dx, dy, dz] 通过 Vector3Stamped Topic 发给 tianji_world_output
```

### 阶段 4: tianji_world_output_node Topic 订阅与 IK

```python
# 1. 订阅手腕目标位姿 Topic (pico_input 发布)
#    回调中将 PoseStamped → 4x4 矩阵
self.left_arm_pose = self._pose_to_matrix(msg.pose)

# 2. 订阅肘部方向 Topic (pico_input 发布)
self.left_arm_direction = [msg.vector.x, msg.vector.y, msg.vector.z]

# 3. 控制循环中更新 IK 零空间参数
self.controller.left_zsp_para = [
    self.left_arm_direction[0],
    self.left_arm_direction[1],
    self.left_arm_direction[2],
    0, 0, 0
]

# 4. 调用 IK 计算关节角
l_success, r_success, l_joints, r_joints = self.controller.move_to_pose_direct(
    left_pose=self.left_arm_pose,   # 4x4 矩阵
    right_pose=self.right_arm_pose,
    unit='matrix'
)
```

### 阶段 5: IK 零空间参数 (zsp_para) 详解

**问题：7 自由度机械臂是冗余的**
- 同一个手部位姿，肘部可以有无数种姿态
- 需要额外约束来确定唯一解

**解决方案：zsp_para 参考平面**
```python
# zsp_para = [x, y, z, a, b, c]
# 前三个值 [x, y, z] 定义一个方向向量
# IK 解算器会让肘部尽量朝这个方向

# 示例：
zsp_para = [0.7, 0.0, 0.7, 0, 0, 0]  # 肘部向右上方
zsp_para = [0.0, -1.0, 0.0, 0, 0, 0]  # 肘部向下
```

### 肘部方向计算算法 (IncrementalController.compute_elbow_direction)

**核心思路：** 将 arm tracker 位置投影到肩-腕连线的垂直平面，得到肘部偏移方向。

```
            肩膀 (shoulder, chest 原点)
              ●
             /|\\
            / | \\
           /  |  \\
     肩-肘 /   |   \\ 肩-腕
          /    |    \\
         ●─────●─────●
      肘部   投影点   手腕

        ←────→
      肘部偏移向量 (direction)
```

**计算步骤：**

```python
# IncrementalController.compute_elbow_direction()

# 1. 肩-腕向量 (手臂主轴)
shoulder_to_wrist = wrist - shoulder
sw_unit = shoulder_to_wrist / norm(shoulder_to_wrist)

# 2. 将肘部投影到肩-腕直线上
shoulder_to_elbow = elbow - shoulder
proj_length = dot(shoulder_to_elbow, sw_unit)
proj_point = shoulder + proj_length * sw_unit

# 3. 肘部偏移向量 = 实际肘部位置 - 投影点
elbow_offset = elbow - proj_point

# 4. 灰色区间防抖: 偏移太小时保持上一稳定方向
if norm(elbow_offset) < elbow_gray_zone:
    direction = last_stable_direction
else:
    direction = normalize(elbow_offset)

# 5. EMA 平滑 (再归一化确保单位向量)
smoothed = alpha * direction + (1 - alpha) * prev_smoothed
direction = normalize(smoothed)
```

**发布给 IK 时取反：**
```python
# pico_input_node.py 中:
ik_direction = -direction  # 几何方向取反 → IK 反重力方向
# → 发布到 /left_arm_elbow_direction (Vector3Stamped)
# → tianji_world_output 订阅后设为 zsp_para = [dir.x, dir.y, dir.z, 0, 0, 0]
```

**为什么取反？** 几何计算的肘部偏移方向指向重力方向（沉肘时肘部在下方），
而 IK zsp_para 要求反重力方向（参考平面法向量）。

---

## ⚠️ 注意事项

### 初始化时机
1. 确保机器人已到达 `init_joints` 姿态
2. 确保 PC-Service 正在运行
3. 确保 PICO 已连接
4. 用户摆好姿势后再初始化

### 坐标系对齐
- 用户初始化时应**面向机器人前方**
- 使用固定坐标变换 (PICO→机器人)，用户前方对应机器人前方

### Tracker 序列号确认
```yaml
# pico_input.yaml — 通过 6 位序列号映射 tracker 角色
tracker_serial_190058: "pico_left_wrist"   # 确认实际佩戴位置
tracker_serial_190600: "pico_right_wrist"
tracker_serial_190046: "pico_left_arm"
tracker_serial_190023: "pico_right_arm"
```

### 网络延迟
- 建议使用 5GHz WiFi
- PC-Service 和 ROS2 节点在同一台机器上运行

---

## 📹 StereoVR 立体相机配置 (v2.0)

StereoVR 系统已升级至 v2.0，采用双进程架构以实现更高的稳定性和性能隔离。

### 架构概述

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                        StereoVR v2.0 双进程架构                              │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   USB Camera (/dev/stereo_camera)                                          │
│        │                                                                    │
│        ▼                                                                    │
│   stereovr_main (主进程)                                                    │
│        │                                                                    │
│        ├──▶ H.264 编码 ──▶ PICO VR (60fps, 低延迟)                         │
│        │                                                                    │
│        └──▶ BGR24 ──▶ v4l2loopback (/dev/video99) ──┐                      │
│                                                      │                      │
│                                                      ▼                      │
│                                        stereovr_publisher (ROS2进程)        │
│                                                      │                      │
│                                                      ▼                      │
│                                          /stereo/left/compressed            │
│                                          /stereo/right/compressed           │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 启动命令

```bash
# 方式 1: 使用 ROS2 Launch 文件 (推荐)
ros2 launch camera stereovr_launch.py

# 带参数启动
ros2 launch camera stereovr_launch.py \
    camera_device:=/dev/stereo_camera \
    loopback_device:=/dev/video99 \
    fps:=30

# 方式 2: 单独启动进程 (调试用)
# 终端 1: 启动主进程
ros2 run camera stereovr_main --device /dev/stereo_camera --loopback /dev/video99

# 终端 2: 启动 ROS2 发布器
ros2 run camera stereovr_publisher --device /dev/video99 --fps 30
```

### 配置文件

配置文件位置: `src/camera/config/stereovr/stereovr_config.yaml`

```yaml
stereovr:
  camera_device: "/dev/stereo_camera"    # 双目相机设备
  loopback_device: "/dev/video99"        # v4l2loopback虚拟设备

  resolution:
    width: 2560     # 双目总宽度
    height: 720     # 单目高度
    fps: 30         # 帧率

  topics:
    left: "/stereo/left/compressed"
    right: "/stereo/right/compressed"
```

### v4l2loopback 设置

在使用 StereoVR 之前，需要加载 v4l2loopback 内核模块：

```bash
# 加载模块
sudo modprobe v4l2loopback devices=1 video_nr=99 card_label='StereoVR'

# 验证设备
ls -la /dev/video99
v4l2-ctl --device /dev/video99 --all
```

### UDEV 规则 (创建固定设备链接)

为双目相机创建固定的设备链接，避免设备编号变化：

```bash
# 查找相机 VID/PID
lsusb | grep -i camera

# 创建 udev 规则
sudo tee /etc/udev/rules.d/99-stereo-camera.rules << 'EOF'
SUBSYSTEM=="video4linux", ATTRS{idVendor}=="1bcf", ATTRS{idProduct}=="2d4f", ATTR{index}=="0", SYMLINK+="stereo_camera", MODE="0666"
EOF

# 重新加载规则
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### 相机变更日志

| 版本 | 日期 | 更新内容 |
|------|------|----------|
| v2.0 | 2025-01 | 迁移至 stereocamera 包，双进程架构，支持 v4l2loopback |
| v1.0 | 2024-12 | 初始版本，单进程 xrobo_server |

---


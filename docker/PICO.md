# PICO 遥操方案

PICO VR 头显 + Manus 手套，通过 PICO 追踪手臂位姿，头部双目 H.264 实时串流到 VR 显示。

> **前置步骤:** 请先完成 [README.md](README.md) 步骤 1-5 (Docker 安装、构建、启动)。

## 1. PICO 准备

### 开启开发者模式

1. PICO 头显 → 设置 → 通用 → 开发者模式 → **开启**
2. 允许 USB 调试

### 安装 XRoboToolkit

XRoboToolkit 是 PICO 端遥操应用，从 [XRoboToolkit-Unity-Client releases](https://github.com/lzhu686/XRoboToolkit-Unity-Client/releases) 下载 v1.4 APK:

```bash
# ADB 侧载安装 (PICO USB 已连接)
adb install XRoboToolkit-v1.4.apk
```

### USB 连接

用 USB 线连接 PICO 头显到 PC:

```bash
# 宿主机验证连接
adb devices
# 应显示: XXXXXXXXXX    device
```

> PICO 首次连接需在头显内确认"允许 USB 调试"。

## 2. 操作流程

### 步骤一: 启动容器

```bash
cd ~/Desktop/wuji-teleop-ros2/docker
docker compose up -d

# 等待构建完成 (首次约 2 分钟)
docker logs -f wuji-teleop
# 看到 "SDK Status:" 即就绪
```

### 步骤二: 确认 ADB 状态

```bash
docker exec -it wuji-teleop bash

# 容器内检查
adb devices                 # 应显示 PICO 设备
adb reverse --list          # 应显示两个端口:
                            #   (reverse) tcp:63901 tcp:63901
                            #   (reverse) tcp:13579 tcp:13579
```

如果 `adb reverse --list` 为空，手动设置:

```bash
adb reverse tcp:63901 tcp:63901    # PC-Service 控制通道
adb reverse tcp:13579 tcp:13579    # 相机命令通道
```

> ADB reverse 端口由容器内 `adb_watchdog` 自动管理 (每 5 秒检测)。如果 PICO 断线重连，watchdog 会自动恢复端口。

### 步骤三: 启动 PICO 端

1. **PICO 头显:** 打开 XRoboToolkit 应用
2. **PICO 头显:** 点击 **Connect** 按钮
3. 等待连接成功提示

### 步骤四: 启动遥操节点

```bash
# 容器内
ros2 launch wuji_teleop_bringup pico_teleop.launch.py
```

启动后会显示参数概览和 IP 信息。系统自动:
1. 等待 PICO 数据 (120 秒超时)
2. 检测到数据后自动初始化
3. 开始增量控制

### 步骤五: 验证

```bash
# 另一终端进入容器
docker exec -it wuji-teleop bash

# 检查话题
ros2 topic list | grep -E "arm_target|stereo|wrist"
ros2 topic hz /stereo/left/compressed         # 头部双目 (~30fps)
```

### 步骤六: 打开立体视觉

1. **PICO 头显:** XRoboToolkit 中选择连接模式:
   - **ADB (USB 有线):** 选择 ADB 模式，IP 地址填 **127.0.0.1**
   - **WiFi (无线):** 选择 WiFi 模式，IP 地址填 PC 的局域网 IP (如 `192.168.1.100`)
2. **PICO 头显:** 点击 **Listen** 按钮
3. PC 端日志应显示 `OPEN_CAMERA` → `MEDIA_DECODER_READY` → `H.264 streaming active`
4. PICO 头显中应看到头部双目相机的实时立体画面

> 如果 PICO 断开重连，系统自动回退到 ROS2-only 模式，再次 Connect → Listen 即恢复 H.264 推流。

## 3. 启动命令

| 启动方式 | 说明 |
|----------|------|
| `ros2 launch wuji_teleop_bringup pico_teleop.launch.py` | 全功能 (臂+手+相机) |
| `ros2 launch wuji_teleop_bringup pico_teleop_minimal.launch.py` | 仅手臂末端控制 |

### pico_teleop.launch.py 参数

```bash
# 预览模式 (仅 RViz 可视化, 不控制机器人)
ros2 launch wuji_teleop_bringup pico_teleop.launch.py \
    enable_robot:=false enable_camera:=false enable_hand:=false enable_rviz:=true
```

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `enable_robot` | `true` | 天机臂输出 |
| `enable_camera` | `true` | 相机 (头部双目 + 腕部 D405) |
| `enable_hand` | `true` | Manus 手套输入 + 灵巧手输出 |
| `enable_rviz` | `false` | RViz 可视化 |

### pico_teleop_minimal.launch.py 参数

```bash
# 回放模式 (无需 PICO 设备)
ros2 launch wuji_teleop_bringup pico_teleop_minimal.launch.py \
    data_source_type:=recorded playback_speed:=0.3
```

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `data_source_type` | `live` | `live` (实时 PICO) 或 `recorded` (回放) |
| `playback_speed` | `0.3` | 回放速度 (仅 recorded 模式) |
| `loop_playback` | `true` | 循环回放 |

> **坐标系:** PICO 方案使用**世界坐标系 IK** (`tianji_world_output`)，PICO 追踪数据在世界坐标系下直接解算。

## 4. 头部双目 H.264 串流 (立体视觉)

PICO 方案独有: 头部双目相机画面实时 H.264 编码推流到 PICO VR 头显显示。

### 数据流

```
头部双目相机 (USB) → OpenCV MJPEG 60fps
  ├── ROS2: split L/R → JPEG → /stereo/{left,right}/compressed (30fps)
  └── PICO: BGR24 → FFmpeg → H.264 → TCP:12345 → PICO VR 显示 (60fps)
```

### 自动协商流程

1. PICO XRoboToolkit 通过 TCP:13579 发送 `OPEN_CAMERA` 命令
2. PICO 发送 `MEDIA_DECODER_READY` (含视频端口号)
3. PC 端启动 FFmpeg H.264 编码
4. PC 端连接 PICO 视频端口 (通过 ADB forward 或 WiFi 直连)
5. H.264 帧流开始

### 测试步骤

完整的立体视觉测试流程:

```bash
# === 宿主机 ===

# 1. 确认 PICO USB 已连接
adb devices

# === 容器内 ===
docker exec -it wuji-teleop bash

# 2. 确认 ADB reverse 端口
adb reverse --list
# 应显示:
#   (reverse) tcp:63901 tcp:63901
#   (reverse) tcp:13579 tcp:13579
# 如为空: adb reverse tcp:63901 tcp:63901 && adb reverse tcp:13579 tcp:13579

# 3. 启动相机节点 (测试模式, 仅相机)
ros2 launch camera camera_launch.py enable_pico:=true

# 4. PICO 头显: 打开 XRoboToolkit → Connect → Listen
# 日志应显示:
#   PICO client connected: 127.0.0.1:xxxxx
#   OPEN_CAMERA: 2560x720 @ 60fps, 30Mbps
#   MEDIA_DECODER_READY, video port=12345
#   ADB forward tcp:12345 → PICO:12345
#   TCP connected: 127.0.0.1:12345
#   H.264 streaming active
#   PICO H.264: xxx frames | 60.0fps

# 5. 验证 ROS2 话题同时工作
ros2 topic hz /stereo/left/compressed      # ~30fps
```

> 如果 PICO 断开重连，系统自动回退到 ROS2-only 模式，再次 Connect 即恢复 H.264 推流。

## 5. ADB 管理

### 架构

PICO 有线模式通过 ADB 建立通信:

```
宿主机 USB ← PICO 头显
        ↓
    Docker 容器
        ├── adb_watchdog (后台守护, 每 5s 检测)
        │     └── adb reverse tcp:63901 (PC-Service 控制)
        │     └── adb reverse tcp:13579 (相机命令)
        ├── PC-Service (端口 63901, XRoboToolkit Connect)
        └── unified_stereo (端口 13579 命令 + 动态 forward 视频)
              └── adb forward tcp:12345 (H.264 视频, 每次推流动态创建)
```

### 端口说明

| 方向 | 端口 | 用途 | 管理方式 |
|------|------|------|----------|
| `reverse` | 63901 | PC-Service 控制通道 | watchdog 自动 |
| `reverse` | 13579 | 相机命令通道 (XRobo 协议) | watchdog 自动 |
| `forward` | 12345 | H.264 视频流 (PC→PICO) | 每次推流动态创建 |

- **reverse** (PICO→PC): 持久性，session 级别，断连后丢失，watchdog 自动恢复
- **forward** (PC→PICO): 每次视频连接动态创建，端口号由 PICO `MEDIA_DECODER_READY` 指定

### 手动诊断

```bash
# 查看 PICO 连接
adb devices

# 查看 reverse 端口映射
adb reverse --list

# 手动设置 (watchdog 失效时)
adb reverse tcp:63901 tcp:63901
adb reverse tcp:13579 tcp:13579

# 查看 forward 端口映射
adb forward --list

# 查看 watchdog 日志
docker logs wuji-teleop 2>&1 | grep "ADB watchdog"
```

### ADB 故障排除

| 问题 | 解决 |
|------|------|
| `adb devices` 无设备 | PICO 开发者模式是否开启？USB 线是否数据线？头显内确认 USB 调试授权 |
| `adb reverse --list` 为空 | 等待 5 秒 (watchdog 周期)，或手动 `adb reverse tcp:63901 tcp:63901` |
| `unauthorized` 设备 | PICO 头显内点击"允许 USB 调试" |
| PICO 拔插后无法连接 | `adb kill-server && sudo adb start-server`，等待 watchdog 恢复 |

## 6. WiFi 模式 (无 USB 线)

PICO 也支持 WiFi 无线连接 (无需 ADB):

1. PC 和 PICO 连接同一局域网
2. PICO XRoboToolkit 中配置 PC 的 IP 地址
3. 启动相机节点时，系统自动检测到无 ADB 设备，使用 WiFi 直连

> WiFi 模式下 ADB 相关步骤全部跳过。视频流直接通过局域网 TCP 连接，无需 ADB forward。
>
> WiFi 延迟较高 (RTT ~5-15ms vs USB ~1ms)，推荐有线模式。

## 7. 常见问题

| 问题 | 解决 |
|------|------|
| PICO Connect 失败 | 确认 `adb reverse --list` 有 63901 端口 |
| H.264 无画面 | 确认 `adb reverse --list` 有 13579，查看容器日志有无 `OPEN_CAMERA` |
| 视频推流中断 | 系统自动回退 ROS2-only，PICO 重新 Connect 即恢复 |
| `TCP connect failed` | ADB forward 端口未建立，检查 USB 连接 |
| NVENC 编码失败 | 容器无 GPU 时自动回退 libx264，不影响功能 |
| Manus 手套无数据 | 宿主机 `git lfs pull` 确保 SDK 文件完整 |
| `pico_input` 初始化超时 | PICO XRoboToolkit 是否已连接？检查 PC-Service 日志 |

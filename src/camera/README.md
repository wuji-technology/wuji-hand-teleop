# Camera Package

Wuji 遥操作系统相机集成包。

## 硬件配置

| 位置 | 默认相机 | 传感器 | 快门 | 接口 | 设备符号链接 |
|------|----------|--------|------|------|-------------|
| 头部 | HBVCAM-F2439GS-2 V11 | AR0234CS | 全局快门 | USB UVC | `/dev/stereo_camera` |
| 左腕 | RealSense D405 | - | 全局快门 | USB | `/dev/cam_left_wrist` |
| 右腕 | RealSense D405 | - | 全局快门 | USB | `/dev/cam_right_wrist` |

> 灵巧手遥操作选用全局快门相机，避免高速运动时的果冻效应。

## 架构

两条数据路径，统一由 `camera_launch.py` 管理：

### Path 1: 腕部 RealSense → ROS2

```
camera_launch.py → realsense2_camera 驱动 → /cam_{left,right}_wrist/color/image_rect_raw
```

配置: `config/camera_config.yaml`

### Path 2: 头部双目 → unified_stereo (ROS2 + PICO)

```
camera_launch.py → unified_stereo (单进程, OpenCV 采集)
  ├── ROS2: MJPEG→BGR→JPEG → /stereo/{left,right}/compressed (30fps)
  └── PICO: MJPEG→H.264 (FFmpeg) → TCP:12345 → PICO VR (60fps, on-demand)
```

- ROS2 发布始终启用 (`enable_head:=true`)
- PICO H.264 推流按需启用 (`enable_pico:=true`)，PICO 连接时自动握手推流
- 配置: `config/stereo_head/stereo_head_config.yaml`

## 首次配置

```bash
# 安装 udev 规则，创建固定设备符号链接
bash src/camera/setup_cameras.sh
```

## 启动命令

```bash
# 腕部 + 头部双目 → ROS2 (默认)
ros2 launch camera camera_launch.py

# 仅腕部 RealSense (禁用头部)
ros2 launch camera camera_launch.py enable_head:=false

# 腕部使用 D435 测试
ros2 launch camera camera_launch.py wrist_type:=d435i

# 腕部 + 头部双目 → ROS2 + PICO 串流
ros2 launch camera camera_launch.py enable_pico:=true

# D435 测试 + 头部
ros2 launch camera camera_launch.py wrist_type:=d435i enable_head:=true

# 头部双目独立启动 (向后兼容)
ros2 launch camera stereo_head_launch.py

# 完整遥操作 (含 PICO 串流)
ros2 launch wuji_teleop_bringup pico_teleop.launch.py
ros2 launch wuji_teleop_bringup wuji_teleop_camera.launch.py
```

## ROS2 话题

| 话题 | 类型 | 来源 |
|------|------|------|
| `/cam_left_wrist/color/image_rect_raw` | sensor_msgs/Image | RealSense D405 驱动 |
| `/cam_right_wrist/color/image_rect_raw` | sensor_msgs/Image | RealSense D405 驱动 |
| `/stereo/left/compressed` | CompressedImage | unified_stereo |
| `/stereo/right/compressed` | CompressedImage | unified_stereo |

## 入口点

| 名称 | 模块 | 调用方 |
|------|------|--------|
| `unified_stereo` | stereocamera.unified_stereo | camera_launch.py (enable_head/enable_pico) |

## 启动参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `wrist_type` | (YAML) | 覆盖腕部相机类型 (d435i/d405) |
| `enable_head` | true | 启用头部双目相机 → ROS2 发布 |
| `enable_pico` | false | 启用头部双目相机 → PICO H.264 串流 |
| `head_device` | /dev/stereo_camera | 头部双目相机设备路径 |
| `head_fps` | 30 | 头部双目相机 ROS2 发布帧率 |
| `head_quality` | 70 | 头部双目相机 JPEG 压缩质量 |
| `head_width` | 2560 | 头部双目相机帧宽度 (左+右拼接) |
| `head_height` | 720 | 头部双目相机帧高度 |

## 支持的相机

### 头部 (双目 RGB, side-by-side)

| 相机 | 传感器 | 快门 | 基线 | H-FOV | 分辨率 | 备注 |
|------|--------|------|------|-------|--------|------|
| **HBVCAM-F2439GS-2 V11** | AR0234CS | **全局** | 60mm | 125° | 2560x720@60 | **默认**，灵巧手推荐 |
| ZED Mini | OV4689 | 卷帘 | 63mm | 85° | 2560x720@60 | 备选，USB 3.0 必须 |
| ZED 2i (4mm) | - | 卷帘 | 120mm | 65° | 2560x720@60 | FOV 较窄 |
| 其他 USB 双目 | - | - | - | - | 自定义 | 需 side-by-side 输出 |

> 头部相机通过 OpenCV UVC 读取，**无需专用 SDK 或 NVIDIA GPU**。
> 任何输出 side-by-side 双目画面的 USB 相机均可使用。
> ZED 系列相机**必须接 USB 3.0 端口**，USB 2.0 只暴露 IMU (HID)，无视频。

### 腕部 (RealSense)

| 相机 | 快门 | 分辨率 | 配置类型 | 备注 |
|------|------|--------|----------|------|
| **RealSense D405** | **全局** | 848x480@30 | `d405` | **默认**，近距离，灵巧手推荐 |
| RealSense D435 | 卷帘 | 848x480@30 | `d435i` | 测试用，launch 参数切换 |

## 切换相机

### 方法 1: 启动参数覆盖 (临时切换腕部相机型号)

无需修改配置文件，通过 launch 参数临时覆盖腕部相机类型：

```bash
# 默认使用 YAML 中配置的类型 (d405)
ros2 launch camera camera_launch.py

# 测试阶段使用 D435
ros2 launch camera camera_launch.py wrist_type:=d435i

# 显式指定 D405
ros2 launch camera camera_launch.py wrist_type:=d405
```

### 方法 2: 修改配置文件 (永久切换)

编辑 `config/camera_config.yaml`：

```yaml
# 切换腕部相机类型: "d405" (默认) 或 "d435i"
left_wrist:
  type: "d405"    # ← 修改此字段
right_wrist:
  type: "d405"    # ← 修改此字段
```

头部相机切换只需更改 `video_device` 和 `resolution`（所有 UVC 双目相机使用相同 `type: "usb"`）：

```yaml
head:
  type: "usb"
  video_device: "/dev/stereo_camera"   # udev 符号链接
  resolution:
    width: 2560       # 双目总宽度 (单眼 1280)
    height: 720       # 需匹配实际相机规格
    fps: 60
```

### 方法 3: 更换物理相机硬件

**更换腕部 RealSense (同型号 D405):**

只需改序列号。两个 D405 的 vendor/product ID 相同，必须用序列号区分左右。

1. 查序列号: `rs-enumerate-devices | grep "Serial Number"`
2. 修改 `config/camera_config.yaml` 中 `serial_number` (**必须**, RealSense SDK 靠此打开相机)
3. 修改 `config/udev/99-teleop-cameras.rules` 中对应序列号 (Docker 挂载需要 symlink 时)
4. 重新运行 `bash src/camera/setup_cameras.sh` (仅改了 udev 规则时需要)

> **注意:** 步骤 2 是必须的，步骤 3-4 仅在需要 `/dev/cam_left_wrist` 等符号链接时才需要。
> 不使用 Docker 时，只做步骤 1-2 即可。

**更换头部双目相机 (同型号 HBVCAM):**

udev 按 vendor/product ID 匹配，同型号直接换，无需任何配置修改。

**更换头部双目相机 (不同型号):**

1. 确认新相机支持 UVC side-by-side 输出
2. 修改 `config/udev/99-teleop-cameras.rules` 中的 vendor/product ID
3. 修改 `config/camera_config.yaml` 中 `head.resolution` 匹配新相机规格
4. 修改 `config/stereo_head/stereo_head_config.yaml` 中对应分辨率
5. 重新运行 `bash src/camera/setup_cameras.sh`

## Docker

```bash
docker run \
  --device /dev/stereo_camera \
  --device /dev/cam_left_wrist \
  --device /dev/cam_right_wrist \
  your-image
```

## 目录结构

```
camera/
├── config/
│   ├── camera_config.yaml              # 主配置 (相机类型/设备/分辨率/序列号)
│   ├── stereo_head/stereo_head_config.yaml  # 头部双目运行时参数
│   └── udev/99-teleop-cameras.rules    # udev 规则 (设备符号链接)
├── launch/
│   ├── camera_launch.py                # 统一入口 (腕部+头部+PICO)
│   └── stereo_head_launch.py           # 头部双目独立启动 (向后兼容)
├── stereocamera/
│   ├── config_loader.py                # 配置文件加载
│   ├── unified_stereo.py               # 头部双目入口 (ROS2 + PICO H.264)
│   └── teleopVision/                   # 核心库
│       ├── ffmpeg_utils.py             # FFmpeg 编码器检测 + NAL 解析
│       ├── unified_stereo_node.py      # 主节点 (OpenCV采集 + ROS2发布 + PICO推流)
│       └── xrobo_protocol.py           # XRoboToolkit 兼容协议
└── setup_cameras.sh                    # udev 规则安装脚本
```

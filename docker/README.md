# Wuji Teleop Docker

天机臂 + Manus 手套 + 灵巧手 + 双目/腕部相机，Docker 一键部署。

支持两种遥操方案:

| 方案 | 输入设备 | 指南 |
|------|----------|------|
| **SteamVR** | HTC Vive Tracker + Manus 手套 | [STEAMVR.md](STEAMVR.md) |
| **PICO** | PICO VR 头显 + Manus 手套 | [PICO.md](PICO.md) |

## 1. 安装 Docker

```bash
sudo apt-get update
sudo apt-get install -y docker.io docker-compose-plugin git-lfs
sudo usermod -aG docker $USER
newgrp docker
```

## 2. 克隆仓库

```bash
cd ~/Desktop
git clone https://github.com/wuji-technology/wuji-hand-teleop.git
cd wuji-hand-teleop

# 拉取 Manus SDK 大文件 (~250MB)
git lfs install
git lfs pull
```

## 3. 构建镜像

```bash
cd docker
docker compose build

# 国内加速
docker compose build --build-arg PIP_INDEX_URL=https://pypi.tuna.tsinghua.edu.cn/simple
```

镜像只含运行环境 (ROS2 + 驱动 + Python 依赖)，不含业务代码。

## 4. 启动容器

```bash
docker compose up -d
```

首次启动自动执行 `colcon build` (约 2 分钟)，后续启动秒级就绪。

## 5. 进入容器

```bash
docker exec -it wuji-teleop bash
```

进入后会显示 SDK 状态:

```
SDK Status:
  [OK] PICO SDK
  [OK] PICO PC-Service
  [OK] Manus SDK
  [OK] Tianji SDK
  [OK] WujiHand SDK
  [OK] RealSense Driver
```

## 6. 选择方案

完成步骤 1-5 后，根据你的输入设备选择对应指南:

- **HTC SteamVR 方案** → [STEAMVR.md](STEAMVR.md)
- **PICO VR 方案** → [PICO.md](PICO.md)

### 全部 Launch 文件

| Launch 文件 | 头部双目 | 腕部 D405 | 说明 |
|------------|:--------:|:---------:|------|
| `wuji_teleop.launch.py` | — | — | 双臂+双手完整遥操 (SteamVR/Manus) |
| `wuji_teleop_hand.launch.py` | — | — | 仅手部控制 |
| `wuji_teleop_arm.launch.py` | — | — | 仅机械臂控制 |
| `wuji_teleop_camera.launch.py` | ✅ | ✅ | 完整遥操 + 全部相机 |
| `wuji_teleop_single.launch.py` | — | — | 单侧调试 (`side:=left` 或 `right`) |
| `pico_teleop.launch.py` | ✅ | ✅ | PICO VR 全遥操 (头部 + 腕部 + Manus 灵巧手) |
| `pico_teleop_minimal.launch.py` | — | — | PICO 最小测试 (支持录制数据回放) |
| `camera_launch.py` (camera 包) | ✅ | ✅ | 单独启动相机 |

### Manus 手套控灵巧手 (单测)

Manus 手套插上 USB Dongle，容器内启动:

```bash
# 双手控制
ros2 launch wuji_teleop_bringup wuji_teleop_hand.launch.py hand_input:=manus

# 单侧调试 (仅右手/仅左手)
ros2 launch wuji_teleop_bringup wuji_teleop_hand.launch.py hand_input:=manus \
    left_serial:=NONE left_hand_name:=disabled_left
ros2 launch wuji_teleop_bringup wuji_teleop_hand.launch.py hand_input:=manus \
    right_serial:=NONE right_hand_name:=disabled_right
```

验证数据流:

```bash
# 1. 检查 Manus 手套话题 (C++ SDK → ROS2)
ros2 topic hz /manus_glove_0    # 应 ~200Hz
ros2 topic hz /manus_glove_1

# 2. 检查手部输入话题 (Manus → MediaPipe 21-point)
ros2 topic hz /hand_input       # 应 ~50Hz
ros2 topic echo /hand_input --once | head -20

# 3. 检查灵巧手命令话题 (retarget → 驱动)
ros2 topic hz /left_hand/joint_commands   # 应 ~50Hz
ros2 topic hz /right_hand/joint_commands

# 4. 检查灵巧手状态反馈 (驱动 → 编码器)
ros2 topic hz /left_hand/joint_states     # 应 ~1000Hz
ros2 topic hz /right_hand/joint_states

# 5. 切换 TELEOP/INFERENCE 模式
ros2 service call /wuji_hand/switch_mode std_srvs/srv/SetBool "{data: true}"   # INFERENCE
ros2 service call /wuji_hand/switch_mode std_srvs/srv/SetBool "{data: false}"  # TELEOP
ros2 service call /wuji_hand/get_mode std_srvs/srv/Trigger {}
```

> **接入新手套**: 见 `src/output_devices/wujihand_output/config/wujihand_ik.yaml` 中的手套接入协议说明

## 7. 日常使用

```bash
cd docker

docker compose up -d                    # 启动
docker exec -it wuji-teleop bash        # 进入
docker compose stop                     # 停止 (保留构建产物)
docker compose start                    # 恢复 (无需重新构建)
docker compose down                     # 销毁 (下次启动需重新 colcon build)
```

宿主机 `src/` 直接挂载到容器，修改代码后容器内执行:

```bash
colcon build --symlink-install
```

> **PC 重启后:** `cd docker && docker compose up -d`，等待就绪后进入容器启动遥操作。
>
> **可选开机自启动:** `docker-compose.yml` 中添加 `restart: unless-stopped`，PC 重启后容器自动恢复:
> ```yaml
> services:
>   teleop:
>     restart: unless-stopped   # 添加此行
> ```
> 快速开发阶段不建议开启，`docker compose down` 后需重新 `colcon build`。

## 8. 多机数据采集

容器使用 CycloneDDS multicast，同一局域网内另一台机器:

```bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 topic list
```

## 9. 相机配置

### 头部双目相机

头部双目视频流由 **unified_stereo** 节点统一处理（单进程，无需 v4l2loopback）:

```
头部双目相机 (USB, /dev/stereo_camera) → OpenCV MJPEG 60fps
  ├── ROS2: split L/R → JPEG → /stereo/{left,right}/compressed (30fps)
  └── PICO: BGR24 → FFmpeg → H.264 → TCP → PICO VR (60fps, on-demand)
```

### 腕部 RealSense D405

D405 通过 USB 3.2 连接，**通过序列号绑定左右腕**。

```bash
# 验证 D405 已连接
lsusb | grep Intel    # 应看到 Intel RealSense

# 查看序列号
rs-enumerate-devices --compact
# 输出示例:
#   Intel RealSense D405    409122273982    5.15.1.55
#   Intel RealSense D405    352122272592    5.15.1.55

# 单独启动腕部相机
ros2 launch camera camera_launch.py
```

更换 D405 时修改 `src/camera/config/camera_config.yaml`:

```yaml
cameras:
  left_wrist:
    serial_number: "409122273982"    # ← 改为实际序列号
  right_wrist:
    serial_number: "352122272592"    # ← 改为实际序列号
```

> 如果左右腕画面接反，互换两个序列号即可。

### udev 规则 (可选)

固定相机设备路径，避免多相机时设备号漂移:

```bash
sudo cp src/camera/config/udev/99-teleop-cameras.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### ROS2 话题

| 话题 | 说明 |
|------|------|
| `/cam_left_wrist/color/image_rect_raw/compressed` | 左腕 D405 |
| `/cam_right_wrist/color/image_rect_raw/compressed` | 右腕 D405 |
| `/stereo/left/compressed` | 头部双目左目 |
| `/stereo/right/compressed` | 头部双目右目 |

> D405 只有 `image_rect_raw` (无 `image_raw`)，话题统一用 `image_rect_raw`。

## 10. 常见问题

| 问题 | 解决 |
|------|------|
| `docker compose` 不存在 | `sudo apt-get install docker-compose-plugin` |
| `permission denied` Docker | `sudo usermod -aG docker $USER && newgrp docker` |
| manus_ros2 链接失败 | 宿主机 `git lfs pull`，未拉取时自动跳过 Manus |
| `down` 后需重新构建 | 用 `stop/start` 替代 `down/up` |
| Monitor GUI 无法显示 | 宿主机执行 `xhost +local:docker` 允许 X11 访问 |
| D405 腕部相机不识别 | 确认 USB 3.2 接口，`lsusb | grep Intel` 检查连接 |
| NVENC 编码失败 | 容器无 GPU 访问时自动回退 libx264，见下方 GPU 说明 |

> **GPU 加速 (可选):** 有 NVIDIA 显卡时可启用 NVENC 硬编码:
> 1. 宿主机安装 `nvidia-container-toolkit`
> 2. 取消 `docker-compose.yml` 中 `deploy.resources` 段的注释
> 3. 重建容器: `docker compose up -d`
>
> 无 NVIDIA GPU 时自动使用 libx264 软编码，不影响功能。

# SteamVR 遥操方案

HTC Vive Tracker + Manus 手套，通过 SteamVR 追踪手臂位姿。

> **前置步骤:** 请先完成 [README.md](README.md) 步骤 1-5 (Docker 安装、构建、启动)。

## 1. 宿主机安装 SteamVR

SteamVR 运行在**宿主机** (非容器内)，容器通过 `.steam` 挂载访问。

```bash
# 1. 安装 Steam (如未安装)
sudo apt install steam

# 2. 打开 Steam → 库 → 搜索 "SteamVR" → 安装

# 3. 验证安装路径
ls ~/.steam/debian-installation/steamapps/common/SteamVR/
```

## 2. 配置 Null Driver (无头显模式)

遥操不需要 VR 头显，只用 Vive Tracker。配置 Null Driver 让 SteamVR 无头显启动。需修改两个文件:

**文件 1 — 启用 null driver** (驱动默认配置):

```bash
nano ~/.steam/steam/steamapps/common/SteamVR/drivers/null/resources/settings/default.vrsettings
```

将 `"enable": false` 改为 `"enable": true`:

```json
{
    "driver_null": {
        "enable": true,
        ...
    }
}
```

**文件 2 — 强制使用 null driver** (用户配置，首次启动 SteamVR 后自动生成):

```bash
nano ~/.steam/debian-installation/config/steamvr.vrsettings
```

在 `"steamvr"` 段中确保包含:

```json
{
    "steamvr": {
        "requireHmd": false,
        "forcedDriver": "null",
        "activateMultipleDrivers": true
    }
}
```

> **说明:** `forcedDriver: "null"` 使 SteamVR 跳过头显检测，`requireHmd: false` 避免无头显时报错。Vive Tracker 仍通过 Lighthouse 正常追踪。

## 3. 启动 SteamVR

```bash
# Wayland 桌面 (Ubuntu 22.04+ 默认) 必须加 XWayland 参数
GDK_BACKEND=x11 QT_QPA_PLATFORM=xcb steam steam://rungameid/250820

# X11 桌面可直接启动
# steam steam://rungameid/250820
```

验证 null driver 生效:

```bash
grep "null" ~/.steam/debian-installation/logs/vrserver.txt | tail -3
# 应看到: "Using existing HMD null.Null Serial Number"

# 验证 SteamVR 进程
ps aux | grep vrserver
```

> **Wayland 注意:** Gnome Wayland 不支持 SteamVR 所需的 DRM lease，vrmonitor 崩溃会连带 vrserver 退出。**每次启动都需要加 `GDK_BACKEND=x11 QT_QPA_PLATFORM=xcb` 前缀**。通过 `echo $XDG_SESSION_TYPE` 确认桌面类型。

## 4. HTC Vive Tracker 设置

### 硬件关系

```
基站 (Base Station)          Tracker               Dongle             PC
  发射 IR 红外光  ──→  Tracker 接收 IR 计算自身位姿  ──无线电──→  Dongle (USB) ──→  SteamVR
```

- **基站**: 发射红外激光，Tracker 靠接收基站 IR 信号定位自身。基站只需通电，无需配对
- **Dongle**: USB 无线接收器，每个 Tracker 配对一个 Dongle，负责将 Tracker 数据传回 PC
- **Tracker**: 需要在基站 IR 覆盖范围内才能定位。通过 Dongle 与 PC 通信

### 硬件准备

- 2 个 Vive Tracker (分别绑定左右手臂)
- 2 个 Lighthouse 基站 (追踪空间)
- 2 个 USB Dongle 接收器 (每个 Tracker 一个)

### 连接 Dongle

插入 Dongle，确认识别:

```bash
lsusb | grep 28de
# 应看到: Valve Software Watchman Dongle
```

### 配对 Tracker

1. 确保基站已通电，指示灯绿色常亮
2. SteamVR 状态窗口 → **≡ 菜单** → **Devices** → **Pair Controller**
3. 选择 **I want to pair a different type of controller** → **Vive Tracker**
4. **长按 Tracker 中间按钮 ~5 秒**，蓝灯快闪 → 配对完成后灯变绿
5. 配对信息保存在 Dongle 中，之后开机自动连接

### 固定 Tracker

将 Tracker 用绑带固定在双臂上:
- **左手臂:** Tracker 朝向一致
- **右手臂:** Tracker 朝向一致
- 两个 Tracker 方向需保持相同

### 验证 (容器内)

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

> **注意:** "Failed to lease display" 警告可忽略 — null driver 模式下不需要 VR 显示输出。
>
> 不使用 SteamVR 时，注释掉 `docker-compose.yml` 中 SteamVR 相关的 volumes 挂载即可，不影响 PICO 和 Manus 功能。

## 5. 进入容器启动遥操

```bash
# 1. 允许容器 X11 访问 (GUI 显示需要)
xhost +local:docker

# 2. 进入容器
docker exec -it wuji-teleop bash
```

### 启动命令

| 启动方式 | 说明 |
|----------|------|
| `ros2 launch wuji_teleop_bringup wuji_teleop_camera.launch.py` | 手 + 臂 + 相机 (推荐) |
| `ros2 launch wuji_teleop_bringup wuji_teleop.launch.py` | 手 + 臂 (无相机) |

### 参数说明

```bash
# 完整参数示例
ros2 launch wuji_teleop_bringup wuji_teleop_camera.launch.py \
    hand_input:=manus \
    arm_input:=tracker \
    enable_camera:=true \
    enable_rviz:=false
```

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `hand_input` | `manus` | 手部输入: `manus` |
| `arm_input` | `tracker` | 手臂输入: `tracker` |
| `enable_camera` | `true` | 是否启动相机 |
| `enable_rviz` | `false` | 是否启动 RViz 可视化 |
| `enable_head` | `true` | 头部双目相机 (camera launch) |

> **坐标系:** SteamVR 方案使用**胸部坐标系 IK** (`tianji_output`)，Vive Tracker 数据经 SteamVR 转换后输入。

## 6. Monitor GUI

容器内可启动图形化监控界面:

```bash
# 宿主机先执行 (允许 X11)
xhost +local:docker

# 容器内启动
ros2 run wuji_teleop_monitor monitor
```

Monitor 提供设备状态监控、话题检查、相机预览，以及一键启动遥操作的下拉菜单:

| 预设 | 说明 | 对应 Launch 文件 |
|------|------|-----------------|
| 仅手部 | Manus 手套 → Wuji 灵巧手 | `wuji_teleop_hand.launch.py` |
| 仅机械臂 | Vive Tracker → 天机臂 | `wuji_teleop_arm.launch.py` |
| 手+臂 | Manus + Tracker → 全遥操 | `wuji_teleop.launch.py` |
| 手+臂+相机 | 完整遥操 + 相机 | `wuji_teleop_camera.launch.py` |
| 单侧 (左/右) | 单臂+单手调试 | `wuji_teleop_single.launch.py` |

## 7. 验证

```bash
# 检查所有话题
ros2 topic list

# 关键话题频率
ros2 topic hz /stereo/left/compressed                              # 头部双目 (~30fps)
ros2 topic hz /cam_left_wrist/color/image_rect_raw/compressed      # 左腕 D405
ros2 topic hz /cam_right_wrist/color/image_rect_raw/compressed     # 右腕 D405
```

## 8. 常见问题

| 问题 | 解决 |
|------|------|
| SteamVR 启动后立刻退出 | Wayland 桌面需加 `GDK_BACKEND=x11 QT_QPA_PLATFORM=xcb` 前缀启动 |
| SteamVR 窗口不显示 | 宿主机 `xhost +local:docker` |
| `[--] OpenVR (SteamVR 未挂载)` | 确认宿主机已安装 SteamVR，`ls ~/.steam/debian-installation/steamapps/common/SteamVR/` |
| openvr_input 连不上 SteamVR | 确认宿主机 SteamVR 已启动，容器需 `pid: host` (docker-compose.yml 已配置) |
| Vive Tracker 未识别 | 确认 Lighthouse 基站开启、Tracker 已配对、Dongle 已插入 (`lsusb \| grep 28de`) |
| Tracker 灯不亮 | 检查 Dongle 是否插入、基站是否通电、Tracker 是否在基站 IR 范围内 |
| Null Driver 不生效 | 检查两个配置文件: driver `default.vrsettings` 中 `enable: true`，用户 `steamvr.vrsettings` 中 `forcedDriver: "null"` |
| Monitor GUI 无法显示 | 宿主机执行 `xhost +local:docker` 允许 X11 访问 |
| Manus 手套无数据 | 宿主机 `git lfs pull` 确保 SDK 文件完整 |

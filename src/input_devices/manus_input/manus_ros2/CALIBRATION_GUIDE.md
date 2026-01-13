# Manus 手套校准参数配置指南

## 1. 校准文件放置位置

将你的 `.mcal` 校准文件放置到 manus_ros2 包内的 calibration 目录：

```
manus_ros2/calibration/Calibration.mcal
```

完整路径：
```
/home/os/ros2_ws/src/wuji-hand-teleop-ros2/src/input_devices/manus_input/manus_ros2/calibration/Calibration.mcal
```

### 操作命令

```bash
# 复制你的校准文件（重命名为 Calibration.mcal）
cp /path/to/your/calibration.mcal \
   ~/ros2_ws/src/wuji-hand-teleop-ros2/src/input_devices/manus_input/manus_ros2/calibration/Calibration.mcal
```

**注意**: 文件必须命名为 `Calibration.mcal`

---

## 2. 目录结构

```
manus_ros2/
├── calibration/
│   └── Calibration.mcal    # <-- 校准文件放这里
├── CMakeLists.txt
├── ManusSDK/
├── src/
│   ├── ManusDataPublisher.cpp
│   ├── ManusDataPublisher.hpp
│   └── ...
└── package.xml
```

---

## 3. 代码修改说明

### 3.1 修改 `CMakeLists.txt`

添加 ament_index_cpp 依赖和安装校准目录：

```cmake
# 添加依赖
find_package(ament_index_cpp REQUIRED)

# 安装校准文件目录
install(DIRECTORY calibration/
  DESTINATION share/${PROJECT_NAME}/calibration)

# 更新 ament_target_dependencies
ament_target_dependencies(manus_data_publisher rclcpp std_msgs geometry_msgs manus_ros2_msgs ament_index_cpp)
```

### 3.2 修改 `ManusDataPublisher.hpp`

在 `protected` 部分添加：

```cpp
// 自动加载校准文件
bool LoadCalibrationFile(uint32_t p_GloveId, Side p_Side);

// 校准加载状态
bool m_LeftCalibrationLoaded = false;
bool m_RightCalibrationLoaded = false;
```

### 3.3 修改 `ManusDataPublisher.cpp`

#### 添加头文件

```cpp
#include <ament_index_cpp/get_package_share_directory.hpp>
```

#### 添加加载函数实现

```cpp
/// @brief 自动加载校准文件
bool ManusDataPublisher::LoadCalibrationFile(uint32_t p_GloveId, Side p_Side)
{
    // 从 ROS2 包目录获取校准文件路径
    std::string t_PackageShareDir;
    try {
        t_PackageShareDir = ament_index_cpp::get_package_share_directory("manus_ros2");
    } catch (const std::exception& e) {
        ClientLog::error("Failed to get package share directory: {}", e.what());
        return false;
    }

    std::string t_CalibrationPath = t_PackageShareDir + "/calibration/Calibration.mcal";

    // 检查文件是否存在
    std::ifstream t_File(t_CalibrationPath, std::ios::binary);
    if (!t_File) {
        ClientLog::warn("Calibration file not found: {}", t_CalibrationPath);
        return false;
    }

    // 获取文件大小
    t_File.seekg(0, std::ios::end);
    int t_FileLength = static_cast<int>(t_File.tellg());
    t_File.seekg(0, std::ios::beg);

    // 读取校准数据
    std::vector<unsigned char> t_CalibrationData(t_FileLength);
    t_File.read(reinterpret_cast<char*>(t_CalibrationData.data()), t_FileLength);
    t_File.close();

    // 应用校准数据
    SetGloveCalibrationReturnCode t_Result;
    CoreSdk_SetGloveCalibration(p_GloveId, t_CalibrationData.data(), t_FileLength, &t_Result);

    if (t_Result == SetGloveCalibrationReturnCode_Success) {
        ClientLog::print("Calibration loaded successfully for {} glove (ID: {})",
            p_Side == Side_Left ? "Left" : "Right", p_GloveId);
        return true;
    } else {
        ClientLog::error("Failed to load calibration for glove ID: {}, error code: {}",
            p_GloveId, static_cast<int>(t_Result));
        return false;
    }
}
```

#### 在 `PublishCallback` 函数中添加自动加载逻辑

在 `for (size_t i = 0; i < m_Landscape->gloveDevices.gloveCount; i++)` 循环开始处添加：

```cpp
// 自动加载校准文件（每只手只加载一次）
uint32_t t_GloveId = m_Landscape->gloveDevices.gloves[i].id;
Side t_Side = m_Landscape->gloveDevices.gloves[i].side;

if (t_Side == Side_Left && !m_LeftCalibrationLoaded) {
    if (LoadCalibrationFile(t_GloveId, t_Side)) {
        m_LeftCalibrationLoaded = true;
    }
} else if (t_Side == Side_Right && !m_RightCalibrationLoaded) {
    if (LoadCalibrationFile(t_GloveId, t_Side)) {
        m_RightCalibrationLoaded = true;
    }
}
```

---

## 4. 编译和运行

```bash
# 进入工作空间
cd ~/ros2_ws

# 编译
source /opt/ros/humble/setup.bash
colcon build --packages-select manus_ros2

# 运行
source install/setup.bash
ros2 run manus_ros2 manus_data_publisher
```

---

## 5. 工作原理

1. **编译时** → `calibration/` 目录被安装到 `install/manus_ros2/share/manus_ros2/calibration/`
2. **程序启动** → 连接到 Manus Core
3. **检测到手套** → 自动从包的 share 目录加载 `Calibration.mcal`
4. **校准应用** → 调用 `CoreSdk_SetGloveCalibration()` 将校准数据发送到 Manus Core
5. **后续使用** → 所有 SDK 调用都会使用已加载的校准参数

---

## 6. 打包说明

由于校准文件现在在包内部，打包时会自动包含：

```bash
# 打包整个 manus_ros2 目录即可
tar -czvf manus_ros2.tar.gz manus_ros2/
```

安装后校准文件位置：
```
<ros2_ws>/install/manus_ros2/share/manus_ros2/calibration/Calibration.mcal
```

---

## 7. 注意事项

- 校准文件格式为二进制 `.mcal` 文件
- 每次程序启动时会自动加载校准（无需手动操作）
- 如果校准文件不存在，程序会打印警告但继续运行
- 左右手分别跟踪加载状态，避免重复加载
- **重新编译后校准文件会被安装到 install 目录**

---

## 8. 错误代码说明

| 错误码 | 含义 |
|--------|------|
| `SetGloveCalibrationReturnCode_Success` | 加载成功 |
| `SetGloveCalibrationReturnCode_WrongSideError` | 左右手不匹配 |
| `SetGloveCalibrationReturnCode_VersionError` | 版本不兼容 |
| `SetGloveCalibrationReturnCode_GloveNotFoundError` | 手套未找到 |

---

## 9. 快速检查清单

- [ ] 校准文件已放置到 `manus_ros2/calibration/Calibration.mcal`
- [ ] 代码已修改
- [ ] 已重新编译 (`colcon build --packages-select manus_ros2`)
- [ ] Manus Core 正在运行
- [ ] 手套已连接

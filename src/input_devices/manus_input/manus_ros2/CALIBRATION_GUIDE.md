
# MANUS Glove Calibration Parameter Configuration Guide

## 1. Calibration File Placement

Place your `.mcal` calibration files in the calibration directory within the manus_ros2 package. The code loads separate files for each hand:

- Left hand: `LeftMetaglovePro.mcal`
- Right hand: `RightMetaglovePro.mcal`

```text
manus_ros2/calibration/LeftMetaglovePro.mcal
manus_ros2/calibration/RightMetaglovePro.mcal
```

### Commands

```bash
cp /path/to/left_calibration.mcal \
   ~/ros2_ws/src/wuji-hand-teleop/src/input_devices/manus_input/manus_ros2/calibration/LeftMetaglovePro.mcal
cp /path/to/right_calibration.mcal \
   ~/ros2_ws/src/wuji-hand-teleop/src/input_devices/manus_input/manus_ros2/calibration/RightMetaglovePro.mcal
```

**Note**: The filenames must match exactly (`LeftMetaglovePro.mcal` and `RightMetaglovePro.mcal`)

---

## 2. Directory Structure

```text
manus_ros2/
├── calibration/
│   ├── LeftMetaglovePro.mcal     # <-- Left hand calibration
│   └── RightMetaglovePro.mcal    # <-- Right hand calibration
├── CMakeLists.txt
├── ManusSDK/
├── src/
│   ├── ManusDataPublisher.cpp
│   ├── ManusDataPublisher.hpp
│   └── ...
└── package.xml
```

---

## 3. Code Modification Details

### 3.1 Modify `CMakeLists.txt`

Add ament_index_cpp dependency and install calibration directory:

```cmake
# Add dependency
find_package(ament_index_cpp REQUIRED)

# Install calibration file directory
install(DIRECTORY calibration/
  DESTINATION share/${PROJECT_NAME}/calibration)

# Update ament_target_dependencies
ament_target_dependencies(manus_data_publisher rclcpp std_msgs geometry_msgs manus_ros2_msgs ament_index_cpp)
```

### 3.2 Modify `ManusDataPublisher.hpp`

Add to the `protected` section:

```cpp
// Auto-load calibration file
bool LoadCalibrationFile(uint32_t p_GloveId, Side p_Side);

// Calibration load status
bool m_LeftCalibrationLoaded = false;
bool m_RightCalibrationLoaded = false;
```

### 3.3 Modify `ManusDataPublisher.cpp`

#### Add Header

```cpp
#include <ament_index_cpp/get_package_share_directory.hpp>
```

#### Add Load Function Implementation

```cpp
/// @brief Auto-load calibration file
bool ManusDataPublisher::LoadCalibrationFile(uint32_t p_GloveId, Side p_Side)
{
    // Get calibration file path from ROS2 package directory
    std::string t_PackageShareDir;
    try {
        t_PackageShareDir = ament_index_cpp::get_package_share_directory("manus_ros2");
    } catch (const std::exception& e) {
        ClientLog::error("Failed to get package share directory: {}", e.what());
        return false;
    }

    std::string t_CalibrationFile = (p_Side == Side_Left) ? "LeftMetaglovePro.mcal" : "RightMetaglovePro.mcal";
    std::string t_CalibrationPath = t_PackageShareDir + "/calibration/" + t_CalibrationFile;

    // Check if file exists
    std::ifstream t_File(t_CalibrationPath, std::ios::binary);
    if (!t_File) {
        ClientLog::warn("Calibration file not found: {}", t_CalibrationPath);
        return false;
    }

    // Get file size
    t_File.seekg(0, std::ios::end);
    int t_FileLength = static_cast<int>(t_File.tellg());
    t_File.seekg(0, std::ios::beg);

    // Read calibration data
    std::vector<unsigned char> t_CalibrationData(t_FileLength);
    t_File.read(reinterpret_cast<char*>(t_CalibrationData.data()), t_FileLength);
    t_File.close();

    // Apply calibration data
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

#### Add Auto-Load Logic in `PublishCallback` Function

Add at the beginning of the `for (size_t i = 0; i < m_Landscape->gloveDevices.gloveCount; i++)` loop:

```cpp
// Auto-load calibration file (load once per hand)
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

## 4. Build and Run

```bash
# Enter workspace
cd ~/ros2_ws

# Build
source /opt/ros/humble/setup.bash
colcon build --packages-select manus_ros2

# Run
source install/setup.bash
ros2 run manus_ros2 manus_data_publisher
```

---

## 5. How It Works

1. **At build time** → The `calibration/` directory is installed to `install/manus_ros2/share/manus_ros2/calibration/`
2. **Program starts** → Connects to MANUS Core
3. **Glove detected** → Automatically loads `LeftMetaglovePro.mcal` or `RightMetaglovePro.mcal` based on hand side
4. **Calibration applied** → Calls `CoreSdk_SetGloveCalibration()` to send calibration data to MANUS Core
5. **Subsequent usage** → All SDK calls will use the loaded calibration parameters

---

## 6. Packaging Notes

Since the calibration file is now inside the package, it will be automatically included when packaging:

```bash
# Just package the entire manus_ros2 directory
tar -czvf manus_ros2.tar.gz manus_ros2/
```

Installed calibration file locations:
```text
<ros2_ws>/install/manus_ros2/share/manus_ros2/calibration/LeftMetaglovePro.mcal
<ros2_ws>/install/manus_ros2/share/manus_ros2/calibration/RightMetaglovePro.mcal
```

---

## 7. Important Notes

- The calibration file format is binary `.mcal`
- Calibration is automatically loaded each time the program starts (no manual operation needed)
- If the calibration file does not exist, the program will print a warning but continue running
- Left and right hand loading states are tracked separately to avoid duplicate loading
- **After recompiling, the calibration file will be installed to the install directory**

---

## 8. Error Code Reference

| Error Code | Meaning |
|------------|---------|
| `SetGloveCalibrationReturnCode_Success` | Loaded successfully |
| `SetGloveCalibrationReturnCode_WrongSideError` | Left/right hand mismatch |
| `SetGloveCalibrationReturnCode_VersionError` | Version incompatible |
| `SetGloveCalibrationReturnCode_GloveNotFoundError` | Glove not found |

---

## 9. Quick Checklist

- [ ] Calibration files placed at `manus_ros2/calibration/LeftMetaglovePro.mcal` and `manus_ros2/calibration/RightMetaglovePro.mcal`
- [ ] Code has been modified
- [ ] Recompiled (`colcon build --packages-select manus_ros2`)
- [ ] MANUS Core is running
- [ ] Gloves are connected

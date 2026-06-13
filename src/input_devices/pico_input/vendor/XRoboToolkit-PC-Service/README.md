# XRoboToolkit-PC-Service

## Project Overview
XRoboToolkit-PC-Service is built upon an enterprise-level assistant application that includes multiple submodules and SDKs, developed using C++/Qt. The project aims to provide one-stop management and control services for VR teleoperated robots, including device management, data recording, demonstration, and control functionalities.

### Main Modules

#### Server
- `RoboticsServiceProcess/`
    - Service process
    - Only one instance can run at a time
    - Cross-platform application based on Qt6 (supports Windows x86, Linux x86, Linux aarch64)
    - Main functional modules:
        - Business: Business logic processing
        - DeviceManagement: Device management
        - PXREAGRPCServer: SDK server logic
        - CommonUtils: Common utility library

- `DeviceConnectionManager/` - Device Connection SDK
    - Device connection management
    - Event handling
- `Business/` - Business Logic Module
    - Device Management (DeviceManage):
      - Device connection status management
      - Real-time message communication
      - Device group management
    - Data Model (Model):
      - Device model definition
      - Device status maintenance
      - Device property management
    - Communication Protocol:
      - TCP/UDP communication protocol definition
      - Server command protocol (device control, data transmission)
      - Client message protocol (connection, status, data transmission)
      - Message encapsulation and parsing mechanism
      - Timestamp and verification mechanism
- `PXREAService/` - Background Service
    Service interfaces defined through Protocol Buffers and gRPC, including:
    - Device management and control (device discovery, status monitoring, parameter configuration)
    - Real-time data stream processing (video stream, sensor data)
    - Device status feedback (connection status)
    - Inter-device communication (device-to-device communication, broadcasting)
    - Server heartbeat detection and status maintenance
- `PXREAGRPCServer/` - Server SDK
    - Server-side core functionality encapsulation
    - Encapsulated GRPC server
- `PXREARobotSDK/` - Robot Control SDK
    - Robot device control interface
    - JSON configuration parsing and processing
    - Device status monitoring and feedback
    - Encapsulated software GRPC client
- `CommonUtils/` - Common Utility Library
    - Logging System:
      - Configurable log output (file/debug output)
      - Support for multiple log levels
    - Network Tools:
      - Local IPv4 address retrieval
      - Network connection status detection

#### SDK and Demo
- `SDK/` - SDK Core Directory
    - `include/` - SDK Header Files Directory
        - Contains all SDK public header files
        - Defines SDK interfaces and data structures
    - Platform-specific library files
        - `win/64/` - Windows x64 platform library files
        - `linux/64/` - Linux x86_64 platform library files
        - `linux_aarch64/64/` - Linux ARM64 platform library files
    - Main Features
        - Provides robot control SDK (PXREARobotSDK)
        - Device connection management
        - JSON configuration parsing and processing
        - Encapsulated gRPC client functionality

- `SDKDemo/` - SDK Demonstration and Example Programs
    - `CppSrc/` - C++ Example Programs
        - `ConsoleDemo/` - Console Example Program
            - Demonstrates basic SDK usage
            - Server connection and communication demonstration
            - Device discovery and status monitoring
        - `RobotDemoQt/` - Qt GUI Demonstration Program
            - 2D demonstration interface based on Qt6
            - SDK callback data visualization
        - `RobotDataRecorder/` - Data Recorder
            - Records robot sensor data
            - Multi-type sensor data classification storage
    - `UnityBin/` - Unity Demonstration Programs
        - `RobotWinDemo/` - Windows platform Unity demonstration
        - `RobotLinuxDemo/` - Linux platform Unity demonstration
        - Provides 3D visualization demonstration interface
    - Demonstration Features
        - Device connection and management
        - Real-time data monitoring
        - Device status feedback
        - Data recording and visualization
        - Cross-platform support (Windows, Linux x86_64, Linux ARM64)

#### Third-party Dependencies
- `Redistributable/` - Third-party Dependencies and Pre-compiled Binaries
  - `win/` - Windows Platform Dependencies
    - Visual C++ Runtime (VC_redist.x86.exe, VC_redist.x64.exe)
    - Robot Demo Program (RobotDemoQt.exe)
    - Console Example Program (ConsoleDemo.exe)
    - Data Recorder (RobotDataRecorder.exe)
    - Launch Scripts (run2D.bat, run3D.bat, runDataRecorder.bat, runService.bat)
    - Configuration File (setting.ini)
  - `linux/` - Linux x86_64 Platform Dependencies
    - Robot Demo Program (RobotDemoQt)
    - Console Example Program (ConsoleDemo)
    - Data Recorder (RobotDataRecorder)
    - Compression Library (libz.so)
    - OpenSSL Library (openssl/)
    - gRPC Directory (grpc/)
    - Configuration File (setting.ini)
  - `linux_aarch64/` - Linux ARM64 Platform Dependencies
    - Robot Demo Program (RobotDemoQt)
    - Console Example Program (ConsoleDemo)
    - Data Recorder (RobotDataRecorder)
    - OpenSSL Library (openssl/)
    - gRPC Directory (grpc/)
    - Configuration File (setting.ini)

## Build Process

### Build Scripts

The project provides build scripts for different platforms to simplify the build process:

**Note**: QT 6.6.3 is required for for this project. Please modify the QT path variables in the build scripts to your own QT installation location.

#### Windows Platform Build
- `RoboticsService\qt-msvc.bat`
  - Update the following accordingly before building
	```
	set QT_DIR=C:\Qt\6.6.3\msvc2019_64
	set QT_TOOLS_DIR=C:\Qt\Tools\CMake_64\bin
	set NINJA_PATH=C:\Qt\Tools\Ninja
	set MSVC_DIR=C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\VC\Tools\MSVC\14.29.30133\bin\Hostx64\x64
	set VS_DIR=C:\Program Files (x86)\Microsoft Visual Studio\2019\Community
	```
	
    <details>
    <summary>Qt 6.6.x Installation Guidance</summary>

    - Install [Qt Online Installer](https://doc.qt.io/qt-6/qt-online-installation.html)
    - Select Custom Installation
    - Select 
        - Qt 6.6.x
            - MSVC 2019 64-bit
            - Qt Quick 3D
			- Qt 5 Compatibility Module
            - Qt Shader Tools
            - Additional Libraries
                - Qt 3D
                - Qt Charts
                - Qt Graphs (TP)
                - Qt Multimedia
                - Qt Positioning
                - Qt Quick 3D Physics
                - Qt Quick Effect Maker
                - Qt WebChannel
                - Qt WebEngine
                - Qt WebSockets
                - Qt WebView
            - Qt Quick TimeLine
        - Build Tools
            - CMake
            - Ninja
    - Install
    </details>

#### Linux x86_64 Platform Build
- `RoboticsService\qt-gcc.sh`
  - Used for building the project with GCC compiler on Linux x86_64 platform
  - modify the following lines to your QT installation path
  ```bash
  QT_GCC_64=/home/pico/Qt6/6.6.3/gcc_64/
  export QT6_TOOLS=/home/pico/Qt6/Tools

  export PATH=/home/pico/Qt6/6.6.3/gcc_64/bin:$PATH
  export PATH=/home/pico/Qt6/6.6.3/gcc_64/include:$PATH
  export PATH=/home/pico/Qt6/Tools/QtCreator/bin:$PATH
  export PATH=/home/pico/Qt6/Tools/CMake/bin:$PATH
  ```

#### Linux ARM64 Platform Build
- `RoboticsService\qt-gcc_aarch64.sh`
  - Used for building the project with GCC compiler on Linux ARM64 (aarch64) platform
  - modify the following lines to your QT installation path
  ```
  QT_GCC_ARM64=/home/orin_pico/Qt/6.7.3/gcc_arm64
  export QT6_TOOLS=/home/orin_pico/Qt/Tools
  export PATH=/home/orin_pico/Qt/6.7.3/gcc_arm64/bin:$PATH
  export PATH=/home/orin_pico/Qt/6.7.3/gcc_arm64/include:$PATH
  export PATH=/home/orin_pico/Qt/Tools/QtCreator/bin:$PATH
  export PATH=/home/orin_pico/Qt/Tools/CMake/bin:$PATH
  ```

### Packaging Process

The project provides packaging scripts for different platforms to generate distributable installation packages:

#### Windows Platform Packaging
- `RoboticsService\Package\innosetup\robot.iss`
  - Creates Windows installer using Inno Setup
  - Automatically copies binary files, dependency libraries, and resource files
  - Creates desktop shortcuts and start menu items

#### Linux x86_64 Platform Packaging
- `RoboticsService\Package\debPack\setup.sh`
  - Creates Debian package (.deb) for Linux x86_64 platform
  - Automatically copies binary files, dependency libraries, and resource files
  - Sets up application icons and desktop shortcuts
  - Configures launch scripts:
    - `run2D.sh` - Launches 2D demo program
    - `run3D.sh` - Launches 3D demo program
    - `runRobotDataRecorder.sh` - Launches data recorder
    - `runService.sh` - Launches service process only

#### Linux ARM64 Platform Packaging
- `RoboticsService\Package\debPackAArch64\setup.sh`
  - Creates Debian package (.deb) for Linux ARM64 platform
  - Automatically copies binary files, dependency libraries, and resource files
  - Sets up application icons and desktop shortcuts
  - Configures launch scripts (similar to x86_64, but optimized for ARM64 architecture)


### SDK Interface Description

#### Connect to Service

```c
/**
 * @brief SDK initialization interface
 * @brief Connects to the service and registers the callback
 * @param context Callback context, used to pass user-defined data to the callback function
 * @param cliCallback Callback function pointer, used to listen to server messages
 * @param mask Callback mask, used to filter certain server messages. Pass 0xffffffff
 */
PXREACLIENTSDK_EXPORT int PXREAInit(void* context,pfPXREAClientCallback cliCallback,unsigned mask);
```

#### Disconnect from Service

```c
/**
 * @brief Termination interface
 * @brief Disconnects from the service
 */
PXREACLIENTSDK_EXPORT int PXREADeinit();
```

#### Send JSON Message to VR Device

```c
/**
 * @brief Sends a JSON command to the device
 * @param devID Device SN, refer to device discovery callback
 * @param parameterJson Function and parameters, in JSON format
 * @return 0 Success
 * @return -1 Failure
 */
PXREACLIENTSDK_EXPORT int PXREADeviceControlJson(const char *devID,const char *parameterJson);
```

#### VR Data Reception

Mainly obtained through callback functions

The callback function is specified when connecting to the service (see PXREAInit function)

Format as follows

```c
/**
 * @brief Client callback, used to receive server messages
 * @param context Callback context, passed by parameter 1 context of #PXREAInit
 * @param type Callback type
 * @param status Callback status code, reserved field, currently meaningless
 * @param userData Callback data pointer, determined by parameter 2 type
 */
typedef void(*pfPXREAClientCallback)(void* context,PXREAClientCallbackType type,int status,void* userData);
```

Callback type description

```c
enum PXREAClientCallbackType
{
    /// @brief Server connected
    PXREAServerConnect          = 1<<2,
    /// @brief Server disconnected
    PXREAServerDisconnect       = 1<<3,
    /// @brief Device online
    PXREADeviceFind             = 1<<4,
    /// @brief Device offline
    PXREADeviceMissing          = 1<<5,
    /// @brief Device connected
    PXREADeviceConnect          = 1<<9,
    /// @brief Device state JSON description
    PXREADeviceStateJson        = 1<<25,
    /// @brief Mask, used to enable all callbacks
    PXREAFullMask               = 0xffffffff
};

```

The device data obtained by the user is mainly through the PXREADeviceStateJson callback, and the corresponding callback function parameters are as follows

| Callback Type         | context | type                 | userData | status |
| ------------ | ------- | -------------------- | -------- | ------ |
| JSON format device state description | User-defined   | PXREADeviceStateJson |          | 0      |



### PICO Device JSON Data Description

Currently, there are five types of tracking data: head, controller, hand gesture, full-body motion capture, and independent tracking. All data will be sent as a JSON string with the following basic structure.

```json
{"functionName":"Tracking","value":"{"Head":"Head","Controller":"Controller","Hand":"Hand","Body":"Body Motion Capture","Motion":"Tracker Independent Tracking","timeStampNs":"Timestamp","Input":"Input method 0 head 1 controller 2 gesture"}"}
```

Field description

| Key         | value type | Description                                                                                                                        |
| ----------- | ------- | ------------------------------------------------------------------------------------------------------------------------- |
| function    | string  | Tracking indicates the data is tracking data                                                                                                        |
| value       | Json    | Contains specific data                                                                                                                   |
| Head        | Json    | Headset tracking data |
| Controller  | Json    | Controller tracking data         |
| Hand        | Json    | Hand gesture data           |
| Body        | Json    | Full-body motion capture data         |
| Motion      | Json    | Tracker independent tracking data |
| timeStampNs | int64   | Timestamp when the data was obtained, Unix (nanoseconds)                                                                                                      |
| Input       | int     | Current input method, 0 head input, 1 controller input, 2 gesture input                                                                                                 |

* If the control panel does not enable tracking for the corresponding part, the corresponding key will not exist.

* value is the actual data string. When using, please convert the string to JSON and remove "\\".

```json
JsonData data = JsonMapper.ToObject(HeadJson);
string valueStr = data["value"].ToString().Replace("\\", "");
JsonData valueJson = JsonMapper.ToObject(valueStr);
```

**Pose**: A string representing seven float data for pose, separated by commas. The first three floats represent position (x, y, z), and the last four floats represent rotation (quaternion: x, y, z, w);

**Coordinate System**: Right-handed coordinate system (X right, Y up, **Z out / toward the viewer**), origin set at the head position when the application starts. The following figure marks the position and orientation of the Head point.

<!-- wuji-hand-teleop patch (2026-06-02): upstream wrote "Z in", which is
     left-handed; corrected to "Z out" so the handedness label and the axis
     directions agree. The Unity-axis note further down (line ~423) still
     describes Unity's separate z-out convention vs. the PICO-data z-in
     mapping. -->


<div align="center">
  <img src="https://github.com/user-attachments/assets/34239571-e9d5-4135-ac6b-c6d5014f44b8" width="50%" alt="head_pose">
</div>

#### 1. **Headset Pose:**

```json
{"pose":"0.0,0.0,0.0,0.0,-0.0,0.0,0.0","status":3,"timeStampNs":1732613222842776064,"handMode":0}"
```



| key         | type      | description                   |
| ----------- | ------- | -------------------- |
| pose        | seven float | Pose (right-handed, X right, Y up, Z out)    |
| status      | int     | Indicates confidence (0 not reliable, 1 reliable)      |
| handMode    | Current hand gesture type | 0 not enabled, 1 controller enabled, 2 hand gesture tracking enabled |
| timeStampNs | int64   | Unix timestamp (nanoseconds)          |

#### 2. **Controller**

```json
{"axisX":0.0,"axisY":0.0,"grip":0.0,"trigger":0.0,"primaryButton":false,"secondaryButton":false,"menuButton":false,"pose":"0.0,0.0,0.0,0.0,0.0,0.0"},"right":{"axisX":0.0,"axisY":0.0,"grip":0.0,"trigger":0.0,"primaryButton":false,"secondaryButton":false,"menuButton":false,"pose":"0.0,0.0,0.0,0.0,0.0,0.0,0.0"},"timeStampNs":1732613438765715200}"
```

* left and right represent the left and right controllers respectively

* pose: controller pose (left-handed, X right, Y up, Z in), with the same orientation system as the head pose. 

* Controller buttons

| **Key**         | **Type** | **Left Controller**      | **Right Controller**      |
| --------------- | ------ | ------------ | ------------ |
| menuButton      | bool   | Menu button          | Screenshot/Record button        |
| trigger         | float  | Trigger button          | Trigger button          |
| grip            | float  | Grip button          | Grip button          |
| axisX, axisY     | float  | Joystick           | Joystick           |
| axisClick       | bool   | Joystick click or press | Joystick click or press |
| primaryButton   | bool   | X            | A            |
| secondaryButton | bool   | Y            | B            |
| timeStampNs     | int64  | Unix timestamp (nanoseconds)  |              |

#### 3. **Hand Gesture**

```json
"leftHand":{"isActive":0,"count":26,"scale":1.0,"timeStampNs":1732613438765715200,"HandJointLocations":[{"p":"0,0,0,0,0,0,0","s":0.0,"r":0.0}, ...]},"rightHand":{"isActive":0,"count":26,"scale":1.0,"HandJointLocations":[{"p":"0,0,0,0,0,0,0","s":0.0,"r":0.0}, ...]}
```

* leftHand and rightHand represent left and right hand data respectively

| **key**            | **Type** | **Description**                         |
| ------------------ | ------ | ------------------------------ |
| isActive           | int    | Hand tracking quality (0 low, 1 high)                 |
| count              | int    | Number of finger joints (fixed = 26)                   |
| scale              | float  | Hand scale                           |
| HandJointLocations | Array  | Array of joint pose data, length = count (see joint description below) |
| timeStampNs        | int64  | Unix timestamp (nanoseconds)                    |

**Finger Joint Data**

| **key** | **Type**  | **Description**                                                                                                                                 |
| ------- | ------- | ---------------------------------------------------------------------- |
| p       | seven float | Pose (left-handed, X right, Y up, Z in)                                                       |
| s       | ulong   | Hand joint tracking status, currently only four values: OrientationValid = 0x00000001, PositionValid = 0x00000002, OrientationTracked = 0x00000004, PositionTracked = 0x00000008 |
| r       | float   | Joint radius                                                                 |

**Hand Joint Description**

Below is the description of the 26 finger joints in the HandJointLocations array.


The figure below shows finger joint locations in Unity coordinate system (x right, y up, z out). In the actual data, z-axis follows the z-in direction, same as controller pose and head pose.
![finger_joints](https://github.com/user-attachments/assets/47f1e10d-9e78-4297-a110-a0254b100908)


|     |                       |           |
| --- | --------------------- | --------- |
| 0   | Palm                  | Palm center     |
| 1   | Wrist                 | Wrist joint     |
| 2   | Thumb_metacarpal     | Thumb metacarpal joint   |
| 3   | Thumb_proximal       | Thumb proximal joint  |
| 4   | Thumb_distal         | Thumb distal joint  |
| 5   | Thumb_tip            | Thumb tip joint  |
| 6   | Index_metacarpal     | Index metacarpal joint    |
| 7   | Index_proximal       | Index proximal joint   |
| 8   | Index_intermediate   | Index intermediate joint   |
| 9   | Index_distal         | Index distal joint   |
| 10  | Index_tip            | Index tip joint   |
| 11  | Middle_metacarpal    | Middle metacarpal joint    |
| 12  | Middle_proximal      | Middle proximal joint   |
| 13  | Middle_intermediate  | Middle intermediate joint   |
| 14  | Middle_distal        | Middle distal joint   |
| 15  | Middle_tip           | Middle tip joint   |
| 16  | Ring_metacarpal      | Ring metacarpal joint   |
| 17  | Ring_proximal        | Ring proximal joint  |
| 18  | Ring_intermediate    | Ring intermediate joint  |
| 19  | Ring_distal          | Ring distal joint  |
| 20  | Ring_tip             | Ring tip joint  |
| 21  | Little_metacarpal    | Little metacarpal joint    |
| 22  | Little_proximal      | Little proximal joint   |
| 23  | Little_intermediate  | Little intermediate joint   |
| 24  | Little_distal        | Little distal joint   |
| 25  | Little_tip           | Little tip joint   |

#### 4. Full-body Motion Capture Tracking


Full-body motion capture requires additional Pico Swift devices (at least two) and proper adaptation and calibration in the Pico headset.

![](images/image-25.png)

> Note: Each time the headset is activated, calibration is required.

**Human Joint Reference**

The full-body motion capture function of the PICO SDK supports tracking the 24 human joints shown in the figure below.
<div align="center">
  <img src="https://github.com/user-attachments/assets/36636b6d-4a13-4bd5-980d-299169fb36c9" width="50%" alt="body_joints">
</div>

The following are related concept descriptions:

| **Concept**   | **Description**                                                                     |
| -------- | -------------------------------------------------------------------------- |
| Coordinate System      | Same world coordinate system as the headset data.                                                         |
| Root Joint Node    | 0 (Pelvis)                                                                 |
| Parent/Child Joint Node  | Nodes 1 to 23, the one closer to the root joint is the parent, the one closer to the limb end is the child.                                        |
| Bone       | A rigid body between two nodes, its pose is stored in the parent node structure closer to the root joint. For example: the pose angle of the lower leg bone is stored in the Knee joint structure. More examples:   |

The following is the BodyTrackerRole enumeration, corresponding one-to-one with the joints in the reference diagram.

```c#
public enum BodyTrackerRole
    {
        Pelvis = 0,  // Pelvis
        LEFT_HIP = 1,  // Left hip
        RIGHT_HIP = 2,  // Right hip
        SPINE1 = 3,  // Spine
        LEFT_KNEE = 4,  // Left knee
        RIGHT_KNEE = 5,  // Right knee
        SPINE2 = 6,  // Spine
        LEFT_ANKLE = 7,  // Left ankle
        RIGHT_ANKLE = 8,  // Right ankle
        SPINE3 = 9,  // Spine
        LEFT_FOOT = 10,  // Left foot
        RIGHT_FOOT = 11,  // Right foot
        NECK = 12,  // Neck
        LEFT_COLLAR = 13,  // Left collarbone
        RIGHT_COLLAR = 14,  // Right collarbone
        HEAD = 15,  // Head
        LEFT_SHOULDER = 16,  // Left shoulder
        RIGHT_SHOULDER = 17,  // Right shoulder
        LEFT_ELBOW = 18,  // Left elbow
        RIGHT_ELBOW = 19,  // Right elbow
        LEFT_WRIST = 20,  // Left wrist
        RIGHT_WRIST = 21,  // Right wrist
        LEFT_HAND = 22,  // Left hand
        RIGHT_HAND = 23  // Right hand
    }
```

JSON data description

```bash
{"dt":0,"flags":0,"timeStampNs":1732613438765715200,
    "joints":[{
    "p":"0.0,0.0,0.0,0.0,0.0,0.0,0.0",
    "t":0,
    "va":"0,0,0,0,0,0",
    "wva":"0,0,0,0,0,0"},.....}]}

```

| **key**     | **Type** | **Description**                   |
| ----------- | ------ | ------------------------ |
| joints      | Json   | Array, represents 24 bones              |
| timeStampNs | int64  | Unix timestamp (nanoseconds)              |
| p           | string | Current bone's Pose (position and rotation, seven values)       |
| t           | long   | *IMU timestamp.*         |
| va          | string | Position velocity (x,y,z) angular velocity (x,y,z)   |
| wva         | string | Position acceleration (x,y,z) angular acceleration (x,y,z) |

#### 5. Tracker Independent Tracking

Original data of tracking Tracker

JSON data description

```bash
{"joints":[{"p":"0.0,-0.0,-0.0,0.0,0.0,-0.0,-0.0","va":"0.0,0.0,-0.0,0.0,0.0,0.0","wva":"0.0,0.0,-0.0,-0.0,0,0"},{"p":"-0.0,0.0,-0.0,-0.0,0.0,0.0,-0.0","va":"-0.0,0.0,0.0,0.0,-0.0,0.0","wva":"0.0,-0.0,-1.0,-0.0,-0.0,-0"}],"len":2,"timeStampNs":1733287634681455104,"sn":PC2310MLJ6050513G}
```

| **key**     | **Type** | **Description**                                                         |
| ----------- | ------ | -------------------------------------------------------------- |
| joints      | Json   | Array, represents all Tracker data (currently supports up to 3)                                    |
| timeStampNs | int64  | Unix timestamp (nanoseconds)                                                    |
| p           | string | Current bone's Pose (position and rotation, seven values)                                             |
| va          | string | Position velocity (x,y,z) *Unit: millimeter*       Angular velocity (x,y,z) *Unit: meter*   |
| wva         | string | Position acceleration (x,y,z) *Unit: millimeter*       Angular acceleration (x,y,z) *Unit: meter* |
| sn          | string | Tracker serial number, used to distinguish different trackers                                     |


## C++ API Usage Example

1. Integrate the SDK file path into the project's CMakeList.txt:

```cmake
    target_include_directories(ConsoleDemo PUBLIC ${PROJECT_SOURCE_DIR}/../../../SDK/include)

    target_link_directories(ConsoleDemo PUBLIC ${PROJECT_SOURCE_DIR}/../../../SDK/linux/64)
```

* API call example:

```c++
#include <PXREARobotSDK.h>

// SDK message callback function
inline void OnPXREAClientCallback(void* context,PXREAClientCallbackType type,int status,void* userData)
{
    auto& sdkCaller = (*(SDKCaller*)context);
    switch (type)
    {
    case PXREAServerConnect:
         //qDebug() <<"server connect"  << Qt::endl;
    case PXREAServerDisconnect:
        //qDebug() <<"server disconnect"  << Qt::endl;
        break;
    case PXREADeviceFind:
        //qDebug() <<"device find"<<(const char*)userData<< Qt::endl;
        break;
    case PXREADeviceMissing:
        //qDebug() <<"device missing"<<(const char*)userData<< Qt::endl;
        break;
    case PXREADeviceStateJson:
        break;
    }
}

    // Initialize SDK and register callback function
    void testInit()
    {
        PXREAInit(this, OnPXREAClientCallback, PXREAFullMask);
    }

    // Release SDK
    void testDeinit()
    {
        PXREADeinit();
    }

    // Send message
    void testDeviceControlJson(const QString& devID,const QString& parameterJson)
    {
        PXREADeviceControlJson(devID.toUtf8().constData(),parameterJson.toUtf8().constData());
    }

```

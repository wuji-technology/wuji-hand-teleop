# RoboticsService 项目结构说明

## 项目概述
RoboticsService 是一个企业级助手应用程序，包含多个子模块和 SDK，采用 C++/Qt 开发。该项目旨在为VR遥操作机器人提供一站式的管理和控制服务，包括设备管理、数据记录、演示和控制等功能。

### 主要模块

#### 服务端
- `RoboticsServiceProcess/`
    - 服务进程
    - 一次只能运行一个实例
    - 基于 Qt6 的跨平台应用程序(支持 Windows x86、Linux x86、Linux aarch64)
    - 主要功能模块:
        - Business: 业务逻辑处理
        - DeviceManagement: 设备管理
        - PXREAGRPCServer: SDK服务端逻辑
        - CommonUtils: 通用工具库

- `DeviceConnectionManager/` - 设备连接SDK
    - 设备连接管理
    - 事件处理
- `Business/` - 业务逻辑模块
    - 设备管理（DeviceManage）：
      - 设备连接状态管理
      - 实时消息通信
      - 设备分组管理
    - 数据模型（Model）：
      - 设备模型定义
      - 设备状态维护
      - 设备属性管理
    - 通信协议：
      - TCP/UDP通信协议定义
      - 服务器命令协议（设备控制、数据传输）
      - 客户端消息协议（连接、状态、数据传输）
      - 消息封装和解析机制
      - 时间戳和校验机制
- `PXREAService/` - 后台服务
    通过 Protocol Buffers 和 gRPC 定义的服务接口，包括：
    - 设备管理和控制（设备发现、状态监控、参数配置）
    - 实时数据流处理（视频流、传感器数据）
    - 设备状态反馈（连接状态）
    - 设备间通信（设备点对点通信、广播）
    - 服务器心跳检测和状态维护
- `PXREAGRPCServer/` - 服务器SDK
    - 服务器端核心功能封装
    - 封装了 GRPC 服务端。
- `PXREARobotSDK/` - 机器人控制SDK
    - 机器人设备控制接口
    - JSON配置解析和处理
    - 设备状态监控和反馈
    - 封装软件 GRPC 客户端。
- `CommonUtils/` - 通用工具库
    - 日志系统：
      - 可配置的日志输出（文件/调试输出）
      - 支持多级别日志
    - 网络工具：
      - 本地IPv4地址获取
      - 网络连接状态检测

#### SDK和Demo
- `SDK/` - SDK核心目录
    - `include/` - SDK头文件目录
        - 包含所有SDK的公共头文件
        - 定义SDK接口和数据结构
    - 平台特定库文件
        - `win/64/` - Windows x64平台库文件
        - `linux/64/` - Linux x86_64平台库文件
        - `linux_aarch64/64/` - Linux ARM64平台库文件
    - 主要功能
        - 提供机器人控制SDK（PXREARobotSDK）
        - 设备连接管理
        - JSON配置解析和处理
        - 封装gRPC客户端功能

- `SDKDemo/` - SDK演示和示例程序
    - `CppSrc/` - C++示例程序
        - `ConsoleDemo/` - 控制台示例程序
            - 展示基本SDK使用方式
            - 服务器连接和通信演示
            - 设备发现和状态监控
        - `RobotDemoQt/` - Qt图形界面演示程序
            - 基于Qt6的2D演示界面
            - SDK回调数据可视化
        - `RobotDataRecorder/` - 数据记录器
            - 记录机器人传感器数据
            - 多类型传感器数据分类存储
    - `UnityBin/` - Unity演示程序
        - `RobotWinDemo/` - Windows平台Unity演示
        - `RobotLinuxDemo/` - Linux平台Unity演示
        - 提供3D可视化演示界面
    - 演示功能
        - 设备连接和管理
        - 实时数据监控
        - 设备状态反馈
        - 数据记录和可视化
        - 跨平台支持（Windows、Linux x86_64、Linux ARM64）

#### 第三方依赖
- `Redistributable/` - 第三方依赖及预编译二进制文件
  - `win/` - Windows平台依赖
    - Visual C++ 运行时（VC_redist.x86.exe, VC_redist.x64.exe）
    - 机器人演示程序（RobotDemoQt.exe）
    - 控制台示例程序（ConsoleDemo.exe）
    - 数据记录器（RobotDataRecorder.exe）
    - 启动脚本（run2D.bat, run3D.bat, runDataRecorder.bat, runService.bat）
    - 配置文件（setting.ini）
  - `linux/` - Linux x86_64平台依赖
    - 机器人演示程序（RobotDemoQt）
    - 控制台示例程序（ConsoleDemo）
    - 数据记录器（RobotDataRecorder）
    - 压缩库（libz.so）
    - OpenSSL库（openssl/）
    - gRPC目录（grpc/）
    - 配置文件（setting.ini）
  - `linux_aarch64/` - Linux ARM64平台依赖
    - 机器人演示程序（RobotDemoQt）
    - 控制台示例程序（ConsoleDemo）
    - 数据记录器（RobotDataRecorder）
    - OpenSSL库（openssl/）
    - gRPC目录（grpc/）
    - 配置文件（setting.ini）


## 编译构建流程

### 编译脚本

项目提供了针对不同平台的编译脚本，用于简化构建过程：

#### Windows 平台编译
- `RoboticsService\qt-msvc.bat`
  - 用于 Windows 平台下使用 MSVC 编译器构建项目
  - 自动设置 Qt 和 Visual Studio 环境变量
  - 使用 Ninja 作为构建系统

#### Linux x86_64 平台编译
- `RoboticsService\qt-gcc.sh`
  - 用于 Linux x86_64 平台下使用 GCC 编译器构建项目
  - 自动设置 Qt 环境变量


#### Linux ARM64 平台编译
- `RoboticsService\qt-gcc_aarch64.sh`
  - 用于 Linux ARM64 (aarch64) 平台下使用 GCC 编译器构建项目
  - 自动设置 Qt 环境变量

### 打包流程

项目提供了针对不同平台的打包脚本，用于生成可分发的安装包：

#### Windows 平台打包
- `RoboticsService\Package\innosetup\robot.iss`
  - 使用 Inno Setup 创建 Windows 安装程序
  - 自动复制二进制文件、依赖库和资源文件
  - 创建桌面快捷方式和开始菜单项

#### Linux x86_64 平台打包
- `RoboticsService\Package\debPack\setup.sh`
  - 创建 Debian 包 (.deb) 用于 Linux x86_64 平台
  - 自动复制二进制文件、依赖库和资源文件
  - 设置应用程序图标和桌面快捷方式
  - 配置启动脚本：
    - `run2D.sh` - 启动 2D 演示程序
    - `run3D.sh` - 启动 3D 演示程序
    - `runRobotDataRecorder.sh` - 启动数据记录器
    - `runService.sh` - 仅启动服务进程

#### Linux ARM64 平台打包
- `RoboticsService\Package\debPackAArch64\setup.sh`
  - 创建 Debian 包 (.deb) 用于 Linux ARM64 平台
  - 自动复制二进制文件、依赖库和资源文件
  - 设置应用程序图标和桌面快捷方式
  - 配置启动脚本（与 x86_64 类似，但针对 ARM64 架构优化）


### SDK接口说明

#### 连接服务

```c
/**
 * @brief SDK初始化接口
 * @brief 连接服务，同时注册回调
 * @param context 回调上下文，用于为回调函数传入用户自定义数据
 * @param cliCallback 回调函数指针，用于监听服务端消息
 * @param mask 回调掩码，用于屏蔽某些服务端消息 传入 0xffffffff即可
 */
PXREACLIENTSDK_EXPORT int PXREAInit(void* context,pfPXREAClientCallback cliCallback,unsigned mask);
```

#### 断开服务

```c
/**
 * @brief 终止接口
 * @brief 断开服务连接
 */
PXREACLIENTSDK_EXPORT int PXREADeinit();
```

#### 向VR设备发送json格式消息

```c
/**
 * @brief 向设备发送json格式指令
 * @param devID 设备sn 参考设备发现回调
 * @param parameterJson 功能及参数,json格式
 * @return 0 成功
 * @return -1 失败
 */
PXREACLIENTSDK_EXPORT int PXREADeviceControlJson(const char *devID,const char *parameterJson);
```

#### VR数据接收

主要通过回调函数获取

回调函数在连接服务时指定(参考PXREAInit函数)

格式如下

```c
/**
 * @brief 客户端回调，用户接收服务端消息
 * @param context 回调上下文，由 #PXREAInit 参数1 context 传入
 * @param type 回调类型
 * @param status 回调状态码,保留字段,暂无意义
 * @param userData 回调数据指针，由参数2 type 决定
 */
typedef void(*pfPXREAClientCallback)(void* context,PXREAClientCallbackType type,int status,void* userData);
```

回调类型说明

```c
enum PXREAClientCallbackType
{
    /// @brief 服务已连接
    PXREAServerConnect          = 1<<2,
    /// @brief 服务已断开
    PXREAServerDisconnect       = 1<<3,
    /// @brief 设备上线
    PXREADeviceFind             = 1<<4,
    /// @brief 设备离线
    PXREADeviceMissing          = 1<<5,
    /// @brief 设备连接
    PXREADeviceConnect          = 1<<9,
    /// @brief 设备状态Json描述
    PXREADeviceStateJson        = 1<<25,
    /// @brief 掩码,用于开启全部回调
    PXREAFullMask               = 0xffffffff
};

```

用户获取的设备数据主要通过PXREADeviceStateJson这一回调获得,对应的回调函数参数如下

| 回调类型         | context | type                 | userData | status |
| ------------ | ------- | -------------------- | -------- | ------ |
| Json格式设备状态描述 | 用户自定义   | PXREADeviceStateJson |          | 0      |



### PICO设备Json数据说明

当前的追踪数据总共有五种：头、手柄、手势、全身动捕、独立追踪，所有数据会以一个Json形象的字符串发送，基本结构如下。

```json
{"functionName":"Tracking","value":"{"Head":"头","Controller”:"手柄","Hand":"手势","Body":"全身动捕","Motion":"Tracker独立追踪","timeStampNs":"时间戳","Input":"输入方式0头1手柄2手势"}}
```

字段说明

| Key         | value类型 | 说明                                                                                                                        |
| ----------- | ------- | ------------------------------------------------------------------------------------------------------------------------- |
| function    | string  | Tracking 表示数据为追踪数据                                                                                                        |
| value       | Json    | 包含了具体数据                                                                                                                   |
| Head        | Json    | 头显追踪数据 |
| Controller  | Json    | 手柄追踪数据         |
| Hand        | Json    | 手势数据           |
| Body        | Json    | 全身动捕数据         |
| Motion      | Json    | Tracker的独立追踪数据 |
| timeStampNs | int64   | 当前数据获取时的时间戳，Unix（纳秒)                                                                                                      |
| Input       | int     | 当前的输入方式，0头控输入、1手柄输入、2手势输入                                                                                                 |

* 若控制面板未开启对应部位的追踪则对应的Key将会不存在。

* value为实际数据的字符串，使用时请将字符串转化为Json，注意将"\\"去除。

```json
JsonData data = JsonMapper.ToObject(HeadJson);
string valueStr = data["value"].ToString().Replace("\\", "");
JsonData valueJson = JsonMapper.ToObject(valueStr);
```

**位姿**：用一串字符表示位姿的七个float数据，float用“，”隔开，前三个float表示位置（x,y,z），后四个float表示旋转（四元数：x,y,z,w）;

**坐标系**：左手坐标系，（X右，Y上，Z里），坐标系的原点为启动应用时的头戴Head位置。下图标记了Head点的位置和朝向。


#### 1. **头显位姿：**

```json
{\"pose\":\"0.0,0.0,0.0,0.0,-0.0,0.0,0.0\",\"status\":3,\"timeStampNs\":1732613222842776064,\"handMode\":0}"}
```



| key         | 类型      | 说明                   |
| ----------- | ------- | -------------------- |
| pose        | 七个float | 位姿 （左手系，X右，Y上，Z里）    |
| status      | int     | 表示可信度（0不可信，1可信）      |
| handMode    | 当前的手势类型 | 0 未开 ，1开了手柄 ，2开了手势追踪 |
| timeStampNs | int64   | Unix时间戳（纳秒）          |

#### 2. **手柄**

```json
{\"axisX\":0.0,\"axisY\":0.0,\"grip\":0.0,\"trigger\":0.0,\"primaryButton\":false,\"secondaryButton\":false,\"menuButton\":false,\"pose\":\"0.0,0.0,0.0,0.0,0.0,0.0\"},\"right\":{\"axisX\":0.0,\"axisY\":0.0,\"grip\":0.0,\"trigger\":0.0,\"primaryButton\":false,\"secondaryButton\":false,\"menuButton\":false,\"pose\":\"0.0,0.0,0.0,0.0,0.0,0.0,0.0\"},\"timeStampNs\":1732613438765715200}"}
```

* left、right分别表示左手柄右手柄

* pose：表示位姿（左手系，X右，Y上，Z里）

* 手柄按键

| **Key**         | **类型** | **左手柄**      | **右手柄**      |
| --------------- | ------ | ------------ | ------------ |
| menuButton      | bool   | 菜单键          | 截/录屏键        |
| trigger         | float  | 扳机键          | 扳机键          |
| grip            | float  | 抓握键          | 抓握键          |
| axisX，axisY     | float  | 摇杆           | 摇杆           |
| axisClick       | bool   | 摇杆是click或者按下 | 摇杆是click或者按下 |
| primaryButton   | bool   | X            | A            |
| secondaryButton | bool   | Y            | B            |
| timeStampNs     | int64  | Unix时间戳（纳秒）  |              |

#### 3. **手势**

```json
\"leftHand\":{\"isActive\":0,\"count\":26,\"scale\":1.0,\"timeStampNs\":1732613438765715200,\"HandJointLocations\":[{\"p\":\"0,0,0,0,0,0,0\",\"s\":0.0,\"r\":0.0},{\"p\":\"0,0,0,0,0,0,0\",\"s\":0.0,\"r\":0.0},{\"p\":\"0,0,0,0,0,0,0\",\"s\":0.0,\"r\":0.0},{\"p\":\"0,0,0,0,0,0,0\",\"s\":0.0,\"r\":0.0},{\"p\":\"0,0,0,0,0,0,0\",\"s\":0.0,\"r\":0.0},{\"p\":\"0,0,0,0,0,0,0\",\"s\":0.0,\"r\":0.0},{\"p\":\"0,0,0,0,0,0,0\",\"s\":0.0,\"r\":0.0},{\"p\":\"0,0,0,0,0,0,0\",\"s\":0.0,\"r\":0.0},{\"p\":\"0,0,0,0,0,0,0\",\"s\":0.0,\"r\":0.0},{\"p\":\"0,0,0,0,0,0,0\",\"s\":0.0,\"r\":0.0},{\"p\":\"0,0,0,0,0,0,0\",\"s\":0.0,\"r\":0.0},{\"p\":\"0,0,0,0,0,0,0\",\"s\":0.0,\"r\":0.0},{\"p\":\"0,0,0,0,0,0,0\",\"s\":0.0,\"r\":0.0},{\"p\":\"0,0,0,0,0,0,0\",\"s\":0.0,\"r\":0.0},{\"p\":\"0,0,0,0,0,0,0\",\"s\":0.0,\"r\":0.0},{\"p\":\"0,0,0,0,0,0,0\",\"s\":0.0,\"r\":0.0},{\"p\":\"0,0,0,0,0,0,0\",\"s\":0.0,\"r\":0.0},{\"p\":\"0,0,0,0,0,0,0\",\"s\":0.0,\"r\":0.0},{\"p\":\"0,0,0,0,0,0,0\",\"s\":0.0,\"r\":0.0},{\"p\":\"0,0,0,0,0,0,0\",\"s\":0.0,\"r\":0.0},{\"p\":\"0,0,0,0,0,0,0\",\"s\":0.0,\"r\":0.0},{\"p\":\"0,0,0,0,0,0,0\",\"s\":0.0,\"r\":0.0},{\"p\":\"0,0,0,0,0,0,0\",\"s\":0.0,\"r\":0.0},{\"p\":\"0,0,0,0,0,0,0\",\"s\":0.0,\"r\":0.0},{\"p\":\"0,0,0,0,0,0,0\",\"s\":0.0,\"r\":0.0},{\"p\":\"0,0,0,0,0,0,0\",\"s\":0.0,\"r\":0.0}]},\"rightHand\":{\"isActive\":0,\"count\":26,\"scale\":1.0,\"HandJointLocations\":[{\"p\":\"0,0,0,0,0,0,0\",\"s\":0.0,\"r\":0.0},{\"p\":\"0,0,0,0,0,0,0\",\"s\":0.0,\"r\":0.0},{\"p\":\"0,0,0,0,0,0,0\",\"s\":0.0,\"r\":0.0},{\"p\":\"0,0,0,0,0,0,0\",\"s\":0.0,\"r\":0.0},{\"p\":\"0,0,0,0,0,0,0\",\"s\":0.0,\"r\":0.0},{\"p\":\"0,0,0,0,0,0,0\",\"s\":0.0,\"r\":0.0},{\"p\":\"0,0,0,0,0,0,0\",\"s\":0.0,\"r\":0.0},{\"p\":\"0,0,0,0,0,0,0\",\"s\":0.0,\"r\":0.0},{\"p\":\"0,0,0,0,0,0,0\",\"s\":0.0,\"r\":0.0},{\"p\":\"0,0,0,0,0,0,0\",\"s\":0.0,\"r\":0.0},{\"p\":\"0,0,0,0,0,0,0\",\"s\":0.0,\"r\":0.0},{\"p\":\"0,0,0,0,0,0,0\",\"s\":0.0,\"r\":0.0},{\"p\":\"0,0,0,0,0,0,0\",\"s\":0.0,\"r\":0.0},{\"p\":\"0,0,0,0,0,0,0\",\"s\":0.0,\"r\":0.0},{\"p\":\"0,0,0,0,0,0,0\",\"s\":0.0,\"r\":0.0},{\"p\":\"0,0,0,0,0,0,0\",\"s\":0.0,\"r\":0.0},{\"p\":\"0,0,0,0,0,0,0\",\"s\":0.0,\"r\":0.0},{\"p\":\"0,0,0,0,0,0,0\",\"s\":0.0,\"r\":0.0},{\"p\":\"0,0,0,0,0,0,0\",\"s\":0.0,\"r\":0.0},{\"p\":\"0,0,0,0,0,0,0\",\"s\":0.0,\"r\":0.0},{\"p\":\"0,0,0,0,0,0,0\",\"s\":0.0,\"r\":0.0},{\"p\":\"0,0,0,0,0,0,0\",\"s\":0.0,\"r\":0.0},{\"p\":\"0,0,0,0,0,0,0\",\"s\":0.0,\"r\":0.0},{\"p\":\"0,0,0,0,0,0,0\",\"s\":0.0,\"r\":0.0},{\"p\":\"0,0,0,0,0,0,0\",\"s\":0.0,\"r\":0.0},{\"p\":\"0,0,0,0,0,0,0\",\"s\":0.0,\"r\":0.0}]}}
```

* leftHand和rightHand分别表示左右手数据

| **key**            | **类型** | **说明**                         |
| ------------------ | ------ | ------------------------------ |
| isActive           | int    | 手势追踪的质量（0低，1高）                 |
| count              | int    | 手指关节数（固定=26）                   |
| scale              | float  | 手的缩放                           |
| HandJointLocations | Array  | 关节位姿数据的数组，长度=count （详细看如下关节介绍） |
| timeStampNs        | int64  | Unix时间戳（纳秒）                    |

**手指关节数据**

| **key** | **类型**  | **说明**                                                                                                                                 |
| ------- | ------- | -------------------------------------------------------------------------------------------------------------------------------------- |
| p       | 七个float | 位姿（左手系，X右，Y上，Z里）                                                                                                                       |
| s       | ulong   | 手势关节的追踪状态，目前只有四个值：OrientationValid = 0x00000001PositionValid = 0x00000002,OrientationTracked = 0x00000004,PositionTracked = 0x00000008 |
| r       | float   | 手关节的半径                                                                                                                                 |

**手势关节介绍**

如下是HandJointLocations数组内，26根手势关节的介绍。


上图为Unity坐标系（x右y上z前）下的图，实际数据z轴为上图的反方向。

|     |                       |           |
| --- | --------------------- | --------- |
| 0   | Palm                  | 手掌中心点     |
| 1   | Wrist                 | 手腕关节点     |
| 2   | Thumb\_metacarpal     | 大拇指掌骨关节   |
| 3   | Thumb\_proximal       | 大拇指近端骨关节  |
| 4   | Thumb\_distal         | 大拇指远端骨关节  |
| 5   | Thumb\_tip            | 大拇指顶端骨关节  |
| 6   | Index\_metacarpal     | 食指掌骨关节    |
| 7   | Index\_proximal       | 食指近端骨关节   |
| 8   | Index\_intermediate   | 食指中端骨关节   |
| 9   | Index\_distal         | 食指远端骨关节   |
| 10  | Index\_tip            | 食指顶端骨关节   |
| 11  | Middle\_metacarpal    | 中指掌骨关节    |
| 12  | Middle\_proximal      | 中指近端骨关节   |
| 13  | Middle\_intermediate  | 中指中端骨关节   |
| 14  | Middle\_distal        | 中指远端骨关节   |
| 15  | Middle\_tip           | 中指顶端骨关节   |
| 16  | Ring\_metacarpal      | 无名指掌骨关节   |
| 17  | Ring\_proximal        | 无名指近端骨关节  |
| 18  | Ring\_intermediate    | 无名指中端骨关节  |
| 19  | Ring\_distal          | 无名指远端骨关节  |
| 20  | Ring\_tip             | 无名指顶端骨关节  |
| 21  | Little\_metacarpal    | 小指掌骨关节    |
| 22  | Little\_proximal      | 小指近端骨关节   |
| 23  | Little\_intermediate  | 小指中端骨关节   |
| 24  | Little\_distal        | 小指远端骨关节   |
| 25  | Little\_tip           | 小指顶端骨关节   |

#### 4.全身动捕追踪


全身动捕需要额外配的Pico Swift设备（至少两个），并在Pico一体机内做好适配校准。

![](images/image-25.png)

> 注意：每次激活头戴都需要进行一次校准。

**人体关节点参考**

PICO SDK 的全身动捕功能支持追踪下图中的 24 个人体关节点。


以下为相关概念说明：

| **概念**   | **说明**                                                                     |
| -------- | -------------------------------------------------------------------------- |
| 坐标系      | 均为和头戴设备数据相同的世界坐标系。                                                         |
| 根关节节点    | 0 (Pelvis)                                                                 |
| 父/子关节节点  | 1 至 23 号节点，靠近根关节节点的为父节点，靠近肢体端的为子节点。                                        |
| 骨骼       | 位于两个节点之间的一段刚体，其姿态存储在靠近根节点一侧的父节点结构内。比如：小腿骨骼的姿态角度，会存储在关节点 Knee 结构中。 更多例子 :   |

以下为 BodyTrackerRole 枚举，与参考图中的关节点一一对应。

```c#
public enum BodyTrackerRole
    {
        Pelvis = 0,  // 骨盆
        LEFT_HIP = 1,  // 左侧臀部
        RIGHT_HIP = 2,  // 右侧臀部
        SPINE1 = 3,  // 脊柱
        LEFT_KNEE = 4,  // 左腿膝盖
        RIGHT_KNEE = 5,  // 右腿膝盖
        SPINE2 = 6,  // 脊柱
        LEFT_ANKLE = 7,  // 左脚踝
        RIGHT_ANKLE = 8,  // 右脚踝
        SPINE3 = 9,  // 脊柱
        LEFT_FOOT = 10,  // 左脚
        RIGHT_FOOT = 11,  // 右脚
        NECK = 12,  // 颈部
        LEFT_COLLAR = 13,  // 左侧锁骨
        RIGHT_COLLAR = 14,  // 右侧锁骨
        HEAD = 15,  // 头部
        LEFT_SHOULDER = 16,  // 左肩膀
        RIGHT_SHOULDER = 17,  // 右肩膀
        LEFT_ELBOW = 18,  // 左手肘
        RIGHT_ELBOW = 19,  // 右手肘
        LEFT_WRIST = 20,  // 左手腕
        RIGHT_WRIST = 21,  // 右手腕
        LEFT_HAND = 22,  // 左手
        RIGHT_HAND = 23  // 右手
    }
```

Json数据说明

```bash
{\"dt\":0,\"flags\":0,\"timeStampNs\":1732613438765715200,
    \"joints\":[{
    \"p\":\"0.0,0.0,0.0,0.0,0.0,0.0,0.0\",
    \"t\":0,
    \"va\":\"0,0,0,0,0,0\",
    \"wva\":\"0,0,0,0,0,0\"},.....}]}

```

| **key**     | **类型** | **说明**                   |
| ----------- | ------ | ------------------------ |
| joints      | Json   | 数组，代表了24根骨骼              |
| timeStampNs | int64  | Unix时间戳（纳秒）              |
| p           | string | 当前骨骼的Pose（位置和旋转七位）       |
| t           | long   | *IMU timestamp.*         |
| va          | string | 位置速度（x,y,z）角度速度(x,y,z)   |
| wva         | string | 位置加速度（x,y,z）角度加速度(x,y,z) |

#### 5.Tracker独立追踪

追踪Tracker的原始数据

Json数据说明

```bash
{\"joints\":[{\"p\":\"0.0,-0.0,-0.0,0.0,0.0,-0.0,-0.0\",\"va\":\"0.0,0.0,-0.0,0.0,0.0,0.0\",\"wva\":\"0.0,0.0,-0.0,-0.0,0,0\"},{\"p\":\"-0.0,0.0,-0.0,-0.0,0.0,0.0,-0.0\",\"va\":\"-0.0,0.0,0.0,0.0,-0.0,0.0\",\"wva\":\"0.0,-0.0,-1.0,-0.0,-0.0,-0\"}],\"len\":2,\"timeStampNs\":1733287634681455104,\"sn\":PC2310MLJ6050513G}
```

| **key**     | **类型** | **说明**                                                         |
| ----------- | ------ | -------------------------------------------------------------- |
| joints      | Json   | 数组，代表所有Tracker数据 （目前最多支持3个）                                    |
| timeStampNs | int64  | Unix时间戳（纳秒）                                                    |
| p           | string | 当前骨骼的Pose（位置和旋转七位）                                             |
| va          | string | 位置速度（x,y,z）*Unit: millimeter*       角度速度(x,y,z)*Unit: meter*   |
| wva         | string | 位置加速度（x,y,z）*Unit: millimeter*       角度加速度(x,y,z)*Unit: meter* |
| sn          | string | Tracker的序列号，用于区分不同的trakcer                                     |


## C++使用API示例

1. 将SDK文件路径集成到工程CMakeList.txt中:

```cmake
    target_include_directories(ConsoleDemo PUBLIC ${PROJECT_SOURCE_DIR}/../../../SDK/include)

    target_link_directories(ConsoleDemo PUBLIC ${PROJECT_SOURCE_DIR}/../../../SDK/linux/64)
```

* API调用示例：

```c++
#include <PXREARobotSDK.h>

//SDK消息回调函数
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

    //初始化SDK 并注册回调函数
    void testInit()
    {
        PXREAInit(this, OnPXREAClientCallback, PXREAFullMask);
    }

    //释放SDK
    void testDeinit()
    {
        PXREADeinit();
    }

    //发消息
    void testDeviceControlJson(const QString& devID,const QString& parameterJson)
    {
        PXREADeviceControlJson(devID.toUtf8().constData(),parameterJson.toUtf8().constData());
    }

```
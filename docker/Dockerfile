# ===========================================
# Wuji Teleop ROS2 Docker Image
# PICO + Manus + Tianji Arm + Camera
# ===========================================
#
# 功能:
#   - PICO VR 遥操天机机械臂 (PC-Service → xrobotoolkit_sdk → pico_input → tianji_world_output)
#   - Manus 手套控制 Wuji 灵巧手 (ManusSDK → manus_ros2 → retargeting → wujihand)
#   - D405 腕部 + 头部双目相机 → ROS2 compressed topics
#   - CycloneDDS 多机通信 (wuji-data-record 远程监控/录制)
#
# 构建:
#   cd docker && docker compose build
#
# 运行:
#   cd docker && docker compose up -d
#   docker exec -it wuji-teleop bash
#
# GPU 说明:
#   H.264 编码自动检测: 有 NVIDIA GPU 用 NVENC 硬编码，否则回退 libx264 软编码。
#   NVENC 需要: 1) 宿主机安装 nvidia-container-toolkit  2) 取消 docker-compose.yml 中 GPU 段注释
#   检测方法: 宿主机执行 nvidia-smi，有输出则可启用
#   无 NVIDIA GPU 时 libx264 也能满足 2560x720@60fps 编码需求 (约 1 核 CPU)。
#
# ===========================================

FROM osrf/ros:humble-desktop

LABEL maintainer="zhuliang"
LABEL description="Wuji Teleop: PICO arm + Manus hand + camera ROS2 system"

# -------------------------------------------
# 1. Build Arguments
# -------------------------------------------
ARG USERNAME=wuji
ARG USER_UID=1000
ARG USER_GID=$USER_UID

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# 时区
ENV TZ=Asia/Shanghai
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# 国内用户: 取消注释以使用清华镜像
# RUN sed -i 's|http://archive.ubuntu.com|http://mirrors.tuna.tsinghua.edu.cn|g' /etc/apt/sources.list

# -------------------------------------------
# 2. System Dependencies
# -------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    # 构建工具
    build-essential cmake git \
    python3-pip python3-dev python3-pybind11 \
    # 硬件访问
    libusb-1.0-0-dev libncursesw5-dev udev usbutils \
    # 相机驱动
    ros-humble-realsense2-camera \
    ros-humble-compressed-image-transport \
    # H.264 编码 (头部双目 → PICO VR 推流)
    ffmpeg \
    # DDS (多机通信: wuji-data-record 远程监控)
    ros-humble-rmw-cyclonedds-cpp \
    # XRoboToolkit PC-Service 依赖
    xdg-utils \
    # ADB (PICO 有线连接: USB 数据传输, 端口转发)
    adb \
    # 网络工具 (PICO 连接调试)
    net-tools iputils-ping \
    # GUI: 中文字库 (Qt5 显示) + OpenGL 软渲染
    fonts-noto-cjk \
    libgl1-mesa-glx libgl1-mesa-dri \
    # 内核工具 (wuji_teleop_monitor lsmod 检测)
    kmod \
    # 开发工具
    vim tmux wget curl sudo \
    && rm -rf /var/lib/apt/lists/*

# -------------------------------------------
# 3. XRoboToolkit PC-Service (PICO 数据接收)
# -------------------------------------------
# 官方 GitHub: https://github.com/XR-Robotics/XRoboToolkit-PC-Service
# 安装后: /opt/apps/roboticsservice/runService.sh
ARG XROBOTOOLKIT_VERSION=1.0.0
RUN wget -q https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases/download/v${XROBOTOOLKIT_VERSION}/XRoboToolkit_PC_Service_${XROBOTOOLKIT_VERSION}_ubuntu_22.04_amd64.deb \
        -O /tmp/xrobotoolkit-pc-service.deb \
    && (dpkg -i /tmp/xrobotoolkit-pc-service.deb || apt-get update && apt-get install -f -y) \
    && rm /tmp/xrobotoolkit-pc-service.deb

# -------------------------------------------
# 3b. WujiHandCpp C++ SDK (wujihand_driver ROS2 节点的编译依赖)
# -------------------------------------------
# ROS2 驱动: https://github.com/wuji-technology/wujihandros2 (子模块, src/wujihandros2/)
# C++ SDK deb: https://github.com/wuji-technology/wujihandpy/releases
# 安装后: /usr/lib/libwujihandcpp.so + /usr/include/wujihandcpp/
ARG WUJIHANDCPP_VERSION=1.5.1
RUN wget -q https://github.com/wuji-technology/wujihandpy/releases/download/v${WUJIHANDCPP_VERSION}/wujihandcpp-${WUJIHANDCPP_VERSION}-amd64.deb \
        -O /tmp/wujihandcpp.deb \
    && (dpkg -i /tmp/wujihandcpp.deb || apt-get update && apt-get install -f -y) \
    && rm /tmp/wujihandcpp.deb

# -------------------------------------------
# 4. User Setup (non-root, 硬件组权限)
# -------------------------------------------
# dialout: 串口 | plugdev: USB | input: 输入设备 | video: 相机
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && usermod -aG sudo,dialout,plugdev,input,video $USERNAME \
    && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# -------------------------------------------
# 5. Python Dependencies (版本锁定)
# -------------------------------------------
ARG PIP_INDEX_URL=""
ARG PIP_TIMEOUT=120

# 版本锁定: 验证通过的兼容组合
# numpy 2.2.6 + scipy 1.14.1: 无兼容性警告
# openvr: SteamVR Tracker 方案
# 灵巧手: C++ SDK 由 deb 提供 (见 3b), ROS2 驱动由 wujihandros2 子模块提供
RUN if [ -n "${PIP_INDEX_URL}" ]; then \
        pip3 install --no-cache-dir --timeout ${PIP_TIMEOUT} \
            numpy==2.2.6 scipy==1.14.1 pyyaml==6.0.2 \
            opencv-python==4.13.0.92 \
            openvr==2.12.1401 \
            --index-url ${PIP_INDEX_URL}; \
    else \
        pip3 install --no-cache-dir --timeout ${PIP_TIMEOUT} \
            numpy==2.2.6 scipy==1.14.1 pyyaml==6.0.2 \
            opencv-python==4.13.0.92 \
            openvr==2.12.1401; \
    fi

# -------------------------------------------
# 6. wuji-retargeting (公开版, 手部重定向)
# -------------------------------------------
# https://github.com/wuji-technology/wuji-retargeting
# --depth 1: 无 git 历史 | --recurse-submodules: URDF + meshes
# 注: 上游 pyproject.toml 缺少 [build-system], pip 22 legacy 模式无法解析 [project] 元数据
#      创建 setup.py 作为 shim, 使 pip 正确安装包及其依赖 (nlopt, pinocchio)
RUN git clone --depth 1 --recurse-submodules \
        https://github.com/wuji-technology/wuji-retargeting.git \
        /tmp/wuji-retargeting \
    && cd /tmp/wuji-retargeting \
    && printf 'from setuptools import setup, find_packages\nsetup(\n    name="wuji-retargeting",\n    version="0.1.1",\n    packages=find_packages(),\n    install_requires=["numpy==2.2.6", "nlopt==2.10.0", "pin==3.9.0", "pyyaml"],\n    package_data={"wuji_retargeting": ["urdf/*.urdf", "meshes/**/*.STL", "wuji_hand_description/**/*"]},\n)\n' > setup.py \
    && pip3 install --no-cache-dir . \
    && rm -rf /tmp/wuji-retargeting

# -------------------------------------------
# 7. ROS2 Workspace (仅目录, 源码由宿主机挂载)
# -------------------------------------------
# 源码通过 docker-compose volumes 挂载到 src/
# entrypoint.sh 负责首次 colcon build + 共享库链接
RUN mkdir -p /home/$USERNAME/ros2_ws && chown $USERNAME:$USERNAME /home/$USERNAME/ros2_ws

USER $USERNAME
WORKDIR /home/$USERNAME/ros2_ws

# -------------------------------------------
# 8. Entrypoint & Config
# -------------------------------------------
USER root
# 兼容 legacy builder (不依赖 BuildKit --chmod)
COPY docker/entrypoint.sh /entrypoint.sh
COPY docker/scripts/ /entrypoint-scripts/
RUN chmod 755 /entrypoint.sh /entrypoint-scripts/*.sh
COPY docker/cyclonedds.xml /etc/cyclonedds/cyclonedds.xml

USER $USERNAME

# Auto-source ROS2 + workspace
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
    && echo "source /home/$USERNAME/ros2_ws/install/setup.bash" >> ~/.bashrc

# CycloneDDS for multi-machine communication
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=file:///etc/cyclonedds/cyclonedds.xml
ENV LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
# xrobotoolkit_sdk: vendored .cpython-310.so, 不被 colcon install 安装
ENV PYTHONPATH=/home/$USERNAME/ros2_ws/src/input_devices/pico_input/prebuilt/x86_64:$PYTHONPATH
# Qt5/GUI: 消除 XDG_RUNTIME_DIR 警告
ENV XDG_RUNTIME_DIR=/tmp/runtime-$USERNAME
RUN mkdir -p /tmp/runtime-$USERNAME && chmod 700 /tmp/runtime-$USERNAME

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

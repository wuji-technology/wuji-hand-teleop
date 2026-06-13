# ===========================================
# Wuji Hand Teleop ROS2 Docker Image
# Wuji Glove + Wuji Hand (default) + optional HTC Tracker / PICO arm + cameras
# ===========================================
#
# Features:
#   - Wuji Glove drives the Wuji Hand (wuji_sdk UDP -> wuji_retargeting -> wujihand_driver)
#   - Optional arm teleop: HTC Vive Tracker (docs/STEAMVR.md) or PICO 4 (docs/PICO.md)
#   - D405 wrist + head stereo cameras -> ROS2 compressed topics
#   - MANUS Glove path remains in-tree as community-supported, feature-frozen
#     (see src/input_devices/manus_input/README.md)
#
# Build:
#   cd docker && docker compose build
#
# Run:
#   cd docker && docker compose up -d
#   docker exec -it wuji-hand-teleop bash
#
# GPU notes:
#   H.264 encoding is auto-detected: NVENC when an NVIDIA GPU is present, otherwise libx264 software encoding.
#   NVENC requires: 1) nvidia-container-toolkit installed on the host  2) uncomment the GPU section in docker-compose.yml
#   Detection: run `nvidia-smi` on the host; any output means NVENC is available.
#   Without an NVIDIA GPU, libx264 still keeps up with 2560x720@60fps (about 1 CPU core).
#
# ===========================================

FROM osrf/ros:humble-desktop

LABEL description="Wuji Hand Teleop: Wuji Glove + Wuji Hand (default), optional HTC / PICO arm input, cameras"

# -------------------------------------------
# 1. Build Arguments
# -------------------------------------------
ARG USERNAME=wuji
ARG USER_UID=1000
ARG USER_GID=$USER_UID

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# Timezone
ENV TZ=Asia/Shanghai
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# China users: uncomment to switch to the Tsinghua mirror.
# RUN sed -i 's|http://archive.ubuntu.com|http://mirrors.tuna.tsinghua.edu.cn|g' /etc/apt/sources.list

# -------------------------------------------
# 2. System Dependencies
# -------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Build tools
    build-essential cmake git \
    python3-pip python3-dev python3-pybind11 \
    # Hardware access
    libusb-1.0-0-dev libncursesw5-dev udev usbutils \
    # Camera drivers
    ros-humble-realsense2-camera \
    ros-humble-compressed-image-transport \
    # H.264 encoding (head stereo -> PICO VR stream)
    ffmpeg \
    # DDS (remote monitoring; off by default in single-machine)
    ros-humble-rmw-cyclonedds-cpp \
    # XRoboToolkit PC-Service dependency
    xdg-utils \
    # ADB (wired PICO link: USB data transfer, port forwarding)
    adb \
    # Network tools (PICO connectivity debugging)
    net-tools iputils-ping \
    # GUI: CJK font (for Qt5) + OpenGL software rendering
    fonts-noto-cjk \
    libgl1-mesa-glx libgl1-mesa-dri \
    # Kernel tools (wuji_teleop_monitor lsmod probe)
    kmod \
    # Dev tools
    vim tmux wget curl sudo \
    && rm -rf /var/lib/apt/lists/*

# -------------------------------------------
# 3. XRoboToolkit PC-Service (PICO data receiver)
# -------------------------------------------
# Upstream: https://github.com/XR-Robotics/XRoboToolkit-PC-Service (Apache-2.0)
# Source vendored at src/input_devices/pico_input/vendor/XRoboToolkit-PC-Service/
# The .deb ships via Git LFS at docker/prebuilt/ and is COPYed in at build time.
# After install: /opt/apps/roboticsservice/runService.sh
COPY docker/prebuilt/XRoboToolkit_PC_Service_*_ubuntu_22.04_amd64.deb /tmp/xrobotoolkit-pc-service.deb
# Fail fast if the .deb is still an LFS pointer (forgot to run `git lfs pull`):
# without this check dpkg below would silently fail with a cryptic
# "archive corrupt" error.
RUN if head -c 100 /tmp/xrobotoolkit-pc-service.deb | grep -q 'git-lfs.github.com/spec'; then \
        echo "ERROR: xrobotoolkit-pc-service.deb is a Git LFS pointer, not the real binary." >&2; \
        echo "       Run \`git lfs install && git lfs pull\` on the host before \`docker compose build\`." >&2; \
        exit 1; \
    fi \
    && (dpkg -i /tmp/xrobotoolkit-pc-service.deb || apt-get update && apt-get install -f -y) \
    && rm /tmp/xrobotoolkit-pc-service.deb

# -------------------------------------------
# 3b. WujiHandCpp C++ SDK (build-time dependency of wujihand_driver ROS2 node)
# -------------------------------------------
# ROS2 driver: https://github.com/wuji-technology/wujihandros2 (submodule at src/wujihandros2/)
# C++ SDK deb: https://github.com/wuji-technology/wujihandpy/releases
# After install: /usr/lib/libwujihandcpp.so + /usr/include/wujihandcpp/
ARG WUJIHANDCPP_VERSION=1.5.1
RUN wget -q https://github.com/wuji-technology/wujihandpy/releases/download/v${WUJIHANDCPP_VERSION}/wujihandcpp-${WUJIHANDCPP_VERSION}-amd64.deb \
        -O /tmp/wujihandcpp.deb \
    && (dpkg -i /tmp/wujihandcpp.deb || apt-get update && apt-get install -f -y) \
    && rm /tmp/wujihandcpp.deb

# -------------------------------------------
# 4. User Setup (non-root, hardware group permissions)
# -------------------------------------------
# dialout: serial | plugdev: USB | input: input devices | video: cameras
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && usermod -aG sudo,dialout,plugdev,input,video $USERNAME \
    && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# -------------------------------------------
# 4.5 Pinocchio (cmeel pip wheel, pinned to 4.0.0)
# -------------------------------------------
# pin 3.9.0 wheels bundled an urdfdom build with a soname (4.0) that wasn't
# carried by the cmeel-urdfdom dep, so `import pinocchio` failed at runtime
# with "liburdfdom_sensor.so.4.0: cannot open". pin 4.0.0 fixed the bundled
# urdfdom + rebuilt eigenpy against the NumPy 2 ABI, so the wheel imports
# cleanly with numpy==2.2.6 below (§5). Pinned to 4.0.0 for reproducibility.

# -------------------------------------------
# 5. Python Dependencies (versions pinned)
# -------------------------------------------
ARG PIP_INDEX_URL=""
ARG PIP_TIMEOUT=120

# Pinned versions: validated compatible combo.
# numpy 2.2.6 + scipy 1.14.1: no compatibility warnings.
# openvr: SteamVR Tracker pathway.
# pin==4.0.0: Pinocchio rigid-body library used by wuji_retargeting (cmeel wheel
#   bundles eigenpy + urdfdom; 4.0.0 is the first release whose bundled urdfdom
#   sonames resolve cleanly and whose eigenpy ABI is rebuilt against NumPy 2).
# Dexterous hand: C++ SDK comes from the deb (see 3b); ROS2 driver via the wujihandros2 submodule.
RUN if [ -n "${PIP_INDEX_URL}" ]; then \
        pip3 install --no-cache-dir --timeout ${PIP_TIMEOUT} \
            numpy==2.2.6 scipy==1.14.1 pyyaml==6.0.2 ruamel.yaml==0.18.6 \
            opencv-python==4.13.0.92 \
            openvr==2.12.1401 \
            pin==4.0.0 \
            --index-url ${PIP_INDEX_URL}; \
    else \
        pip3 install --no-cache-dir --timeout ${PIP_TIMEOUT} \
            numpy==2.2.6 scipy==1.14.1 pyyaml==6.0.2 ruamel.yaml==0.18.6 \
            opencv-python==4.13.0.92 \
            openvr==2.12.1401 \
            pin==4.0.0; \
    fi

# -------------------------------------------
# 5.5 wuji-sdk (Wuji Glove SDK, UDP transport)
# -------------------------------------------
# Required by the wuji_glove input path: wujihand_node imports wuji_sdk
# lazily when wujihand_ik.yaml::input_source is "wuji_glove" (the default).
# Public PyPI release tracks Wuji Studio (current: 2026.5.26 <-> Studio 5.18).
ARG WUJI_SDK_VERSION=2026.5.26
RUN pip3 install --no-cache-dir --timeout ${PIP_TIMEOUT} wuji-sdk==${WUJI_SDK_VERSION}

# -------------------------------------------
# 6. wuji-retargeting (Wuji-owned, hand retargeting algorithm)
# -------------------------------------------
# Vendored via the `src/wuji-retargeting` git submodule — same pattern as
# `src/wujihandros2`. The Docker build context is `public/` (set by docker-
# compose.yml `context: ..`), so the submodule appears here at
# `src/wuji-retargeting`. Users MUST initialise submodules before building:
#   git clone --recurse-submodules https://github.com/.../wuji-hand-teleop.git
# or, post-clone: `git submodule update --init --recursive`.
#
# We pip-install (not colcon-build) at image-build time because wuji-retargeting
# is a pure-Python package. Upstream `pyproject.toml` declares [project] but
# omits [build-system], which pip 22's legacy mode rejects; the shim setup.py
# below restores legacy compat without mutating the submodule source. Pinocchio
# is supplied by the cmeel `pin==4.0.0` wheel in §5.
#
# URDF sideload: wuji-retargeting expects URDFs at
# `wuji_retargeting/wuji-description/hand/body/urdf/{left,right}.urdf`, normally
# populated by its `wuji-description` sub-submodule. `git submodule update
# --init --recursive` from the outer repo doesn't traverse that nested submodule
# reliably, so we copy the same URDFs from `wujihandros2/external/wuji-description`
# (same upstream, same tag-tracked content) into the installed package after pip.
COPY src/wuji-retargeting /tmp/wuji-retargeting
COPY src/wujihandros2/external/wuji-description/hand/body/urdf /tmp/wuji-description-urdf
RUN printf 'from setuptools import setup, find_packages\nsetup(\n    name="wuji-retargeting",\n    version="2026.05.26",\n    packages=find_packages(include=["wuji_retargeting*"]),\n    install_requires=["numpy==2.2.6", "nlopt==2.10.0", "pyyaml", "scipy>=1.11.0"],\n)\n' > /tmp/wuji-retargeting/setup.py \
    && pip3 install --no-cache-dir /tmp/wuji-retargeting \
    && WUJI_RT_DIR=$(python3 -c "import wuji_retargeting, os; print(os.path.dirname(wuji_retargeting.__file__))") \
    && mkdir -p "$WUJI_RT_DIR/wuji-description/hand/body/urdf" \
    && cp /tmp/wuji-description-urdf/*.urdf "$WUJI_RT_DIR/wuji-description/hand/body/urdf/" \
    && rm -rf /tmp/wuji-retargeting /tmp/wuji-description-urdf

# -------------------------------------------
# 7. ROS2 Workspace (directory only; source mounted from host)
# -------------------------------------------
# Sources are bind-mounted to src/ via docker-compose volumes.
# entrypoint.sh handles first-run colcon build + shared-library symlinks.
RUN mkdir -p /home/$USERNAME/ros2_ws && chown $USERNAME:$USERNAME /home/$USERNAME/ros2_ws

USER $USERNAME
WORKDIR /home/$USERNAME/ros2_ws

# -------------------------------------------
# 8. Entrypoint & Config
# -------------------------------------------
USER root
# Legacy-builder compatible (no BuildKit --chmod).
COPY docker/entrypoint.sh /entrypoint.sh
COPY docker/scripts/ /entrypoint-scripts/
RUN chmod 755 /entrypoint.sh /entrypoint-scripts/*.sh
COPY docker/cyclonedds.xml /etc/cyclonedds/cyclonedds.xml

USER $USERNAME

# Auto-source ROS2 + workspace
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
    && echo "source /home/$USERNAME/ros2_ws/install/setup.bash" >> ~/.bashrc

# CycloneDDS RMW (used when distributed nodes are added)
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=file:///etc/cyclonedds/cyclonedds.xml
ENV LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
# xrobotoolkit_sdk: vendored .cpython-310.so; not installed by colcon install.
ENV PYTHONPATH=/home/$USERNAME/ros2_ws/src/input_devices/pico_input/prebuilt/x86_64:$PYTHONPATH
# Qt5/GUI: silence the XDG_RUNTIME_DIR warning.
ENV XDG_RUNTIME_DIR=/tmp/runtime-$USERNAME
RUN mkdir -p /tmp/runtime-$USERNAME && chmod 700 /tmp/runtime-$USERNAME

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

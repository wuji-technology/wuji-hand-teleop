#!/bin/bash
# =============================================================================
# PICO Input SDK 安装脚本
#
# 功能: 安装 xrobotoolkit_sdk (PICO VR 数据读取 SDK)
#
# 使用方法:
#   ./install_sdk.sh          # 使用预编译二进制 (推荐)
#   ./install_sdk.sh --build  # 从源码编译
#
# 支持平台: Ubuntu 22.04 x86_64
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PREBUILT_DIR="$SCRIPT_DIR/prebuilt"

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo_info() { echo -e "${GREEN}[INFO]${NC} $1"; }
echo_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
echo_error() { echo -e "${RED}[ERROR]${NC} $1"; }

# =============================================================================
# 检查系统环境
# =============================================================================
check_system() {
    echo_info "检查系统环境..."

    # 检查架构
    ARCH=$(uname -m)
    if [ "$ARCH" != "x86_64" ]; then
        echo_error "当前仅支持 x86_64 架构，你的架构是: $ARCH"
        echo_warn "请使用 --build 选项从源码编译"
        exit 1
    fi

    # 检查 Python 版本
    PYTHON_VERSION=$(/usr/bin/python3 --version 2>&1 | cut -d' ' -f2 | cut -d'.' -f1,2)
    if [ "$PYTHON_VERSION" != "3.10" ]; then
        echo_warn "Python 版本: $PYTHON_VERSION (推荐 3.10)"
        echo_warn "如果遇到问题，请使用 --build 选项重新编译"
    fi

    echo_info "系统检查通过: $ARCH, Python $PYTHON_VERSION"
}

# =============================================================================
# 使用预编译二进制安装 (推荐)
# =============================================================================
install_prebuilt() {
    echo_info "使用预编译二进制安装..."

    # 检查预编译文件是否存在
    if [ ! -f "$PREBUILT_DIR/x86_64/xrobotoolkit_sdk.cpython-310-x86_64-linux-gnu.so" ]; then
        echo_error "预编译文件不存在: $PREBUILT_DIR/x86_64/"
        echo_warn "请使用 --build 选项从源码编译"
        exit 1
    fi

    # 创建目标目录
    mkdir -p ~/.local/lib/python3.10/site-packages
    mkdir -p ~/.local/lib

    # 复制文件
    echo_info "复制 SDK 文件..."
    cp "$PREBUILT_DIR/x86_64/xrobotoolkit_sdk.cpython-310-x86_64-linux-gnu.so" \
       ~/.local/lib/python3.10/site-packages/
    cp "$PREBUILT_DIR/x86_64/libPXREARobotSDK.so" ~/.local/lib/

    echo_info "SDK 文件已安装到:"
    echo "  - ~/.local/lib/python3.10/site-packages/xrobotoolkit_sdk.*.so"
    echo "  - ~/.local/lib/libPXREARobotSDK.so"
}

# =============================================================================
# 从源码编译安装
# =============================================================================
install_from_source() {
    echo_info "从源码编译安装..."

    # 检查依赖
    echo_info "安装编译依赖..."
    sudo apt install -y cmake build-essential pybind11-dev

    # 克隆并编译 C++ SDK
    echo_info "编译 PXREARobotSDK..."
    cd /tmp
    rm -rf pc-service
    git clone https://github.com/lzhu686/XRoboToolkit-PC-Service.git pc-service
    cd pc-service/RoboticsService/PXREARobotSDK
    bash build.sh

    # 克隆 Pybind SDK (如果不存在)
    PYBIND_DIR=~/Desktop/XRoboToolkit-PC-Service-Pybind
    if [ ! -d "$PYBIND_DIR" ]; then
        echo_info "克隆 Pybind SDK..."
        cd ~/Desktop
        git clone https://github.com/lzhu686/XRoboToolkit-PC-Service-Pybind.git
    fi

    # 复制 C++ 库文件
    echo_info "复制 C++ 库文件..."
    cd "$PYBIND_DIR"
    mkdir -p include lib
    cp /tmp/pc-service/RoboticsService/PXREARobotSDK/PXREARobotSDK.h include/
    cp -r /tmp/pc-service/RoboticsService/PXREARobotSDK/nlohmann include/
    cp /tmp/pc-service/RoboticsService/PXREARobotSDK/build/libPXREARobotSDK.so lib/

    # 编译 Python SDK
    echo_info "编译 Python SDK..."
    rm -rf build_user
    mkdir -p build_user && cd build_user
    cmake .. -DCMAKE_LIBRARY_OUTPUT_DIRECTORY=$(pwd)/output \
             -DPYTHON_EXECUTABLE=/usr/bin/python3 \
             -DCMAKE_BUILD_TYPE=Release
    make -j$(nproc)

    # 安装
    echo_info "安装 SDK..."
    mkdir -p ~/.local/lib/python3.10/site-packages
    mkdir -p ~/.local/lib
    cp output/xrobotoolkit_sdk.cpython-310-x86_64-linux-gnu.so \
       ~/.local/lib/python3.10/site-packages/
    cp "$PYBIND_DIR/lib/libPXREARobotSDK.so" ~/.local/lib/

    echo_info "编译安装完成!"
}

# =============================================================================
# 配置环境变量
# =============================================================================
setup_env() {
    echo_info "配置环境变量..."

    # 检查是否已配置
    if grep -q '/.local/lib' ~/.bashrc 2>/dev/null; then
        echo_info "LD_LIBRARY_PATH 已配置"
    else
        echo 'export LD_LIBRARY_PATH=$HOME/.local/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
        echo_info "已添加 LD_LIBRARY_PATH 到 ~/.bashrc"
    fi
}

# =============================================================================
# 安装 Python 依赖
# =============================================================================
install_python_deps() {
    echo_info "安装 Python 依赖..."
    /usr/bin/python3 -m pip install numpy scipy --user --quiet
    echo_info "Python 依赖安装完成"
}

# =============================================================================
# 验证安装
# =============================================================================
verify_install() {
    echo_info "验证安装..."

    export LD_LIBRARY_PATH=$HOME/.local/lib:$LD_LIBRARY_PATH

    if /usr/bin/python3 -c "import xrobotoolkit_sdk as xrt; print('SDK 版本检查通过')" 2>/dev/null; then
        echo_info "✓ xrobotoolkit_sdk 安装成功!"
        return 0
    else
        echo_error "✗ SDK 导入失败"
        echo_warn "请检查错误信息并重试"
        return 1
    fi
}

# =============================================================================
# 主程序
# =============================================================================
main() {
    echo "=============================================="
    echo "  PICO Input SDK 安装脚本"
    echo "  https://github.com/lzhu686"
    echo "=============================================="
    echo ""

    check_system

    if [ "$1" == "--build" ]; then
        install_from_source
    else
        install_prebuilt
    fi

    setup_env
    install_python_deps
    verify_install

    echo ""
    echo "=============================================="
    echo_info "安装完成!"
    echo ""
    echo "下一步:"
    echo "  1. source ~/.bashrc"
    echo "  2. 安装 PC-Service: sudo dpkg -i XRoboToolkit_PC_Service_*.deb"
    echo "  3. 编译 ROS2: colcon build --packages-select pico_input"
    echo "=============================================="
}

main "$@"

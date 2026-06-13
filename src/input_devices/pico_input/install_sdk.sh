#!/bin/bash
# =============================================================================
# PICO Input SDK Installation Script
#
# Function: Install xrobotoolkit_sdk (PICO VR data reading SDK)
#
# Usage:
#   ./install_sdk.sh          # Use prebuilt binary (recommended)
#   ./install_sdk.sh --build  # Build from source
#
# Supported platform: Ubuntu 22.04 x86_64
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PREBUILT_DIR="$SCRIPT_DIR/prebuilt"

# Color output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo_info() { echo -e "${GREEN}[INFO]${NC} $1"; }
echo_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
echo_error() { echo -e "${RED}[ERROR]${NC} $1"; }

# =============================================================================
# Check system environment
# =============================================================================
check_system() {
    echo_info "Checking system environment..."

    # Check architecture
    ARCH=$(uname -m)
    if [ "$ARCH" != "x86_64" ]; then
        echo_error "Currently only x86_64 architecture is supported, your architecture is: $ARCH"
        echo_warn "Please use the --build option to build from source"
        exit 1
    fi

    # Check Python version
    PYTHON_VERSION=$(/usr/bin/python3 --version 2>&1 | cut -d' ' -f2 | cut -d'.' -f1,2)
    if [ "$PYTHON_VERSION" != "3.10" ]; then
        echo_warn "Python version: $PYTHON_VERSION (recommended 3.10)"
        echo_warn "If you encounter issues, please use the --build option to recompile"
    fi

    echo_info "System check passed: $ARCH, Python $PYTHON_VERSION"
}

# =============================================================================
# Install using prebuilt binary (recommended)
# =============================================================================
install_prebuilt() {
    echo_info "Installing using prebuilt binary..."

    # Check if prebuilt files exist
    if [ ! -f "$PREBUILT_DIR/x86_64/xrobotoolkit_sdk.cpython-310-x86_64-linux-gnu.so" ]; then
        echo_error "Prebuilt files not found: $PREBUILT_DIR/x86_64/"
        echo_warn "Please use the --build option to build from source"
        exit 1
    fi

    # Create target directories
    mkdir -p ~/.local/lib/python3.10/site-packages
    mkdir -p ~/.local/lib

    # Copy files
    echo_info "Copying SDK files..."
    cp "$PREBUILT_DIR/x86_64/xrobotoolkit_sdk.cpython-310-x86_64-linux-gnu.so" \
       ~/.local/lib/python3.10/site-packages/
    cp "$PREBUILT_DIR/x86_64/libPXREARobotSDK.so" ~/.local/lib/

    echo_info "SDK files installed to:"
    echo "  - ~/.local/lib/python3.10/site-packages/xrobotoolkit_sdk.*.so"
    echo "  - ~/.local/lib/libPXREARobotSDK.so"
}

# =============================================================================
# Build and install from source (vendored under vendor/)
# =============================================================================
install_from_source() {
    echo_info "Building and installing from vendored source..."

    # Check dependencies
    echo_info "Installing build dependencies..."
    sudo apt install -y cmake build-essential pybind11-dev

    # Vendored sources live alongside this script under vendor/.
    VENDOR_PC_SERVICE="$SCRIPT_DIR/vendor/XRoboToolkit-PC-Service"
    VENDOR_PYBIND="$SCRIPT_DIR/vendor/XRoboToolkit-PC-Service-Pybind"
    if [ ! -d "$VENDOR_PC_SERVICE" ] || [ ! -d "$VENDOR_PYBIND" ]; then
        echo_error "Vendored sources not found under $SCRIPT_DIR/vendor/"
        echo_warn "Run this script from inside a complete wuji-hand-teleop checkout."
        exit 1
    fi

    # Copy vendored PC-Service to /tmp and build PXREARobotSDK
    echo_info "Building PXREARobotSDK from $VENDOR_PC_SERVICE ..."
    cd /tmp
    rm -rf pc-service
    cp -r "$VENDOR_PC_SERVICE" pc-service
    cd pc-service/RoboticsService/PXREARobotSDK
    bash build.sh

    # Copy vendored Pybind to a stable working location (~/Desktop)
    PYBIND_DIR=~/Desktop/XRoboToolkit-PC-Service-Pybind
    if [ ! -d "$PYBIND_DIR" ]; then
        echo_info "Copying vendored Pybind SDK to $PYBIND_DIR ..."
        mkdir -p ~/Desktop
        cp -r "$VENDOR_PYBIND" "$PYBIND_DIR"
    fi

    # Copy C++ library files
    echo_info "Copying C++ library files..."
    cd "$PYBIND_DIR"
    mkdir -p include lib
    cp /tmp/pc-service/RoboticsService/PXREARobotSDK/PXREARobotSDK.h include/
    cp -r /tmp/pc-service/RoboticsService/PXREARobotSDK/nlohmann include/
    cp /tmp/pc-service/RoboticsService/PXREARobotSDK/build/libPXREARobotSDK.so lib/

    # Build Python SDK
    echo_info "Building Python SDK..."
    rm -rf build_user
    mkdir -p build_user && cd build_user
    cmake .. -DCMAKE_LIBRARY_OUTPUT_DIRECTORY=$(pwd)/output \
             -DPYTHON_EXECUTABLE=/usr/bin/python3 \
             -DCMAKE_BUILD_TYPE=Release
    make -j$(nproc)

    # Install
    echo_info "Installing SDK..."
    mkdir -p ~/.local/lib/python3.10/site-packages
    mkdir -p ~/.local/lib
    cp output/xrobotoolkit_sdk.cpython-310-x86_64-linux-gnu.so \
       ~/.local/lib/python3.10/site-packages/
    cp "$PYBIND_DIR/lib/libPXREARobotSDK.so" ~/.local/lib/

    echo_info "Build and installation complete!"
}

# =============================================================================
# Configure environment variables
# =============================================================================
setup_env() {
    echo_info "Configuring environment variables..."

    # Check if already configured
    if grep -q '/.local/lib' ~/.bashrc 2>/dev/null; then
        echo_info "LD_LIBRARY_PATH already configured"
    else
        echo 'export LD_LIBRARY_PATH=$HOME/.local/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
        echo_info "Added LD_LIBRARY_PATH to ~/.bashrc"
    fi
}

# =============================================================================
# Install Python dependencies
# =============================================================================
install_python_deps() {
    echo_info "Installing Python dependencies..."
    /usr/bin/python3 -m pip install numpy scipy --user --quiet
    echo_info "Python dependencies installed"
}

# =============================================================================
# Verify installation
# =============================================================================
verify_install() {
    echo_info "Verifying installation..."

    export LD_LIBRARY_PATH=$HOME/.local/lib:$LD_LIBRARY_PATH

    if /usr/bin/python3 -c "import xrobotoolkit_sdk as xrt; print('SDK version check passed')" 2>/dev/null; then
        echo_info "xrobotoolkit_sdk installed successfully!"
        return 0
    else
        echo_error "SDK import failed"
        echo_warn "Please check error messages and retry"
        return 1
    fi
}

# =============================================================================
# Main program
# =============================================================================
main() {
    echo "=============================================="
    echo "  PICO Input SDK Installation Script"
    echo "  https://github.com/wuji-technology/wuji-hand-teleop"
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
    echo_info "Installation complete!"
    echo ""
    echo "Next steps:"
    echo "  1. source ~/.bashrc"
    echo "  2. Install PC-Service: sudo dpkg -i XRoboToolkit_PC_Service_*.deb"
    echo "  3. Build ROS2: colcon build --packages-select pico_input"
    echo "=============================================="
}

main "$@"

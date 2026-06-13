#!/bin/bash

# Build script for PXREARobotSDK for ARM64 architecture
# Usage: ./build_aarch64.sh [--clean] [--debug|--release]

# Default settings
BUILD_TYPE="RelWithDebInfo"
CLEAN_BUILD=false

# Parse command line arguments
for arg in "$@"; do
    case $arg in
        --clean)
            CLEAN_BUILD=true
            ;;
        --debug)
            BUILD_TYPE="Debug"
            ;;
        --release)
            BUILD_TYPE="Release"
            ;;
        *)
            echo "Unknown option: $arg"
            echo "Usage: ./build_aarch64.sh [--clean] [--debug|--release]"
            exit 1
            ;;
    esac
done

# Get script directory
DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
cd $DIR
echo "Working directory: $DIR"

# Setting up Qt environment for ARM64
echo "Setting up Qt environment for ARM64..."
QT_PATH=/media/bytedance/newSpace/Qt6
export PATH=$QT_PATH/bin:$PATH
export PATH=$QT_PATH/include:$PATH
export PATH=/media/bytedance/newSpace/Qt/Tools/QtCreator/bin:$PATH
export PATH=/media/bytedance/newSpace/Qt/Tools/CMake/bin:$PATH

# Set SDK installation directory for ARM64
SDK_INSTALL_DIR="$DIR/../Redistributable/linux_aarch64/SDK/clientso/64"

# Create build directory
BUILD_DIR="$DIR/build"
if [ ! -d "$BUILD_DIR" ]; then
    echo "Creating build directory..."
    mkdir -p $BUILD_DIR
fi

# Clean build if requested
if [ "$CLEAN_BUILD" = true ]; then
    echo "Cleaning build directory..."
    rm -rf $BUILD_DIR/*
fi

# Configure with CMake
echo "Configuring project with CMake..."
echo "Build type: $BUILD_TYPE"
cmake -S $DIR -B $BUILD_DIR -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DCMAKE_PREFIX_PATH=$QT_PATH

# Build the project
echo "Building project..."
cmake --build $BUILD_DIR --config $BUILD_TYPE

# Check if build was successful
if [ $? -eq 0 ]; then
    echo "Build successful!"
    
    # Create SDK directory if it doesn't exist
    if [ ! -d "$SDK_INSTALL_DIR" ]; then
        echo "Creating SDK installation directory..."
        mkdir -p $SDK_INSTALL_DIR
    fi
    
    # Copy library and header to SDK directory
    echo "Copying library and header to SDK directory..."
    cp $BUILD_DIR/libPXREARobotSDK.so $SDK_INSTALL_DIR/
    cp $DIR/PXREARobotSDK.h $SDK_INSTALL_DIR/
    
    # Create SDK include directory if it doesn't exist
    SDK_INCLUDE_DIR="$DIR/../SDK/include"
    if [ ! -d "$SDK_INCLUDE_DIR" ]; then
        echo "Creating SDK include directory..."
        mkdir -p $SDK_INCLUDE_DIR
    fi
    
    # Copy header to SDK include directory
    echo "Copying header to SDK include directory..."
    cp $DIR/PXREARobotSDK.h $SDK_INCLUDE_DIR/
    
    echo "Installation completed. Files installed to:"
    echo "- $SDK_INSTALL_DIR/libPXREARobotSDK.so"
    echo "- $SDK_INSTALL_DIR/PXREARobotSDK.h"
    echo "- $SDK_INCLUDE_DIR/PXREARobotSDK.h"
else
    echo "Build failed!"
    exit 1
fi

echo "Build process completed."

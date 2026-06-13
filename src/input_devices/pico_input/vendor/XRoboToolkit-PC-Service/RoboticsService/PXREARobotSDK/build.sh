#!/bin/bash

echo "Setting up environment for building PXREARobotSDK..."

# 获取脚本所在目录
DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
cd $DIR
echo "Working directory: $DIR"

# 创建构建目录
BUILD_DIR="$DIR/build"
if [ ! -d "$BUILD_DIR" ]; then
    echo "Creating build directory..."
    mkdir -p $BUILD_DIR
fi

# 清理构建
if [ "$1" = "--clean" ]; then
    echo "Cleaning build directory..."
    rm -rf $BUILD_DIR/*
fi

# 配置 CMake
echo "Configuring project with CMake..."
cmake -S $DIR -B $BUILD_DIR -DCMAKE_BUILD_TYPE=Release

# 构建项目
echo "Building PXREARobotSDK..."
cmake --build $BUILD_DIR --config Release

# 检查构建结果
if [ $? -eq 0 ]; then
    echo "Build successful!"
    
    # 检查输出文件
    INSTALL_DIR="$DIR/../Redistributable/linux/SDK/clientso/64"
    if [ -f "$INSTALL_DIR/libPXREARobotSDK.so" ]; then
        echo "SDK library successfully installed to: $INSTALL_DIR/libPXREARobotSDK.so"
        
        # 复制到 SDK 目录以便 ConsoleDemo 可以找到它
        echo "Copying library to SDK directory..."
        mkdir -p "$DIR/../SDK/linux/64"
        cp "$INSTALL_DIR/libPXREARobotSDK.so" "$DIR/../SDK/linux/64/"
        cp "$DIR/PXREARobotSDK.h" "$DIR/../SDK/include/"
        
        echo "Library copied to $DIR/../SDK/linux/64/libPXREARobotSDK.so"
        echo "Header copied to $DIR/../SDK/include/PXREARobotSDK.h"
    else
        echo "Warning: Library file not found at expected location: $INSTALL_DIR/libPXREARobotSDK.so"
        echo "Searching for the library..."
        find $DIR/../ -name "libPXREARobotSDK.so"
    fi
else
    echo "Build failed!"
    exit 1
fi

echo "Build process completed."

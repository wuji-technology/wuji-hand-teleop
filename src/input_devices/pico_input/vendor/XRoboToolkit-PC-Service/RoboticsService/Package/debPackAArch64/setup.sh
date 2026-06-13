#!/bin/bash
echo "开始为ARM64架构构建增量包..."
DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
echo $DIR

# 只在必要时创建目录结构
if [ ! -d "$DIR/package_arm64" ]; then
    echo "创建目录结构..."
    mkdir -p $DIR/package_arm64/opt/apps/roboticsservice
    mkdir -p $DIR/package_arm64/usr/share/applications
    mkdir -p $DIR/package_arm64/usr/share/icons
    mkdir -p $DIR/package_arm64/DEBIAN
    echo "目录结构已创建。"
else
    echo "使用现有目录结构。"
fi

# 检查控制文件是否已复制
if [ ! -f "$DIR/package_arm64/DEBIAN/control" ]; then
    echo "复制控制文件..."
    cp $DIR/control $DIR/package_arm64/DEBIAN/control
    cp $DIR/postinst $DIR/package_arm64/DEBIAN/
    cp $DIR/postrm $DIR/package_arm64/DEBIAN/
    cp $DIR/prerm $DIR/package_arm64/DEBIAN/
    # 确保脚本有执行权限
    chmod +x $DIR/package_arm64/DEBIAN/postinst
    chmod +x $DIR/package_arm64/DEBIAN/postrm
    chmod +x $DIR/package_arm64/DEBIAN/prerm
    echo "控制文件已复制。"
else
    echo "控制文件已存在，检查更新..."
    # 检查控制文件是否有更新
    if [ "$DIR/control" -nt "$DIR/package_arm64/DEBIAN/control" ]; then
        echo "更新控制文件..."
        cp $DIR/control $DIR/package_arm64/DEBIAN/control
    fi
    if [ "$DIR/postinst" -nt "$DIR/package_arm64/DEBIAN/postinst" ]; then
        echo "更新postinst脚本..."
        cp $DIR/postinst $DIR/package_arm64/DEBIAN/
        chmod +x $DIR/package_arm64/DEBIAN/postinst
    fi
    if [ "$DIR/postrm" -nt "$DIR/package_arm64/DEBIAN/postrm" ]; then
        echo "更新postrm脚本..."
        cp $DIR/postrm $DIR/package_arm64/DEBIAN/
        chmod +x $DIR/package_arm64/DEBIAN/postrm
    fi
    if [ "$DIR/prerm" -nt "$DIR/package_arm64/DEBIAN/prerm" ]; then
        echo "更新prerm脚本..."
        cp $DIR/prerm $DIR/package_arm64/DEBIAN/
        chmod +x $DIR/package_arm64/DEBIAN/prerm
    fi
fi

# 检查桌面文件是否已复制或有更新
if [ ! -f "$DIR/package_arm64/usr/share/applications/roboticsservice.desktop" ] || [ "$DIR/roboticsservice.desktop" -nt "$DIR/package_arm64/usr/share/applications/roboticsservice.desktop" ]; then
    echo "复制/更新桌面文件..."
    # 直接复制桌面文件，不修改Exec条目
    cp $DIR/roboticsservice.desktop $DIR/package_arm64/usr/share/applications/
    echo "桌面文件已复制/更新。"
fi

# 检查图标是否已复制
if [ ! -d "$DIR/package_arm64/usr/share/icons/hicolor" ] || [ "$DIR/hicolor" -nt "$DIR/package_arm64/usr/share/icons/hicolor" ]; then
    echo "复制/更新图标..."
    cp -rf $DIR/hicolor $DIR/package_arm64/usr/share/icons/
    echo "图标已复制/更新。"
fi

# 检查脚本文件是否已复制或有更新
for script in run2D.sh runRobotDataRecorder.sh runService.sh RobotDemoQt RobotDataRecorder; do
    if [ ! -f "$DIR/package_arm64/opt/apps/roboticsservice/$script" ] || [ "$DIR/$script" -nt "$DIR/package_arm64/opt/apps/roboticsservice/$script" ]; then
        echo "复制/更新 $script..."
        cp $DIR/$script $DIR/package_arm64/opt/apps/roboticsservice/
        # 确保脚本有执行权限
        chmod +x $DIR/package_arm64/opt/apps/roboticsservice/$script
        echo "$script 已复制/更新。"
    fi
done

# 检查二进制文件是否需要更新
BIN_DIR="$DIR/../../bin"
TARGET_DIR="$DIR/package_arm64/opt/apps/roboticsservice"

# 始终复制二进制文件，确保包含最新的可执行文件
echo "复制二进制文件..."
# 确保目标目录存在
mkdir -p $TARGET_DIR
# 复制所有二进制文件和依赖项
cp -rf $BIN_DIR/* $TARGET_DIR/
# 确保主可执行文件存在并有执行权限
if [ -f "$BIN_DIR/RoboticsServiceProcess" ]; then
    cp -f $BIN_DIR/RoboticsServiceProcess $TARGET_DIR/
    chmod +x $TARGET_DIR/RoboticsServiceProcess
    echo "主可执行文件 RoboticsServiceProcess 已复制并设置为可执行。"
else
    echo "警告：在 $BIN_DIR 中未找到主可执行文件 RoboticsServiceProcess！"
fi
echo "二进制文件已复制。"

# 构建 Debian 包
echo "构建 Debian 包..."
VERSION=$(grep "Version:" $DIR/control | cut -d " " -f 2)
PACKAGE_NAME="XRoboToolkit-PC-Service_${VERSION}_arm64.deb"

cd $DIR
dpkg-deb -b package_arm64/ ./$PACKAGE_NAME
echo "Debian 包已构建: $PACKAGE_NAME"

# 确保输出目录存在
mkdir -p $DIR/../output

# 复制 Debian 包到输出目录
echo "复制包到输出目录..."
cp ./$PACKAGE_NAME $DIR/../output/
echo "包已复制到输出目录。"

# 清理临时文件
rm ./$PACKAGE_NAME
echo "临时文件已清理。"

echo "ARM64架构的增量包构建已成功完成。"

# XRoboToolkit Unity Client APK

APK 文件已从 Git 仓库移除，以保持仓库轻量。

## 下载 APK

请从 GitHub Release 下载最新版本：

**v1.3.0（最新）**
- **本地坐标系版本（推荐 - 更稳定）⭐**: [XRoboToolkit-v1.3.0-local.apk](https://github.com/lzhu686/XRoboToolkit-Unity-Client/releases/download/v1.3.0/XRoboToolkit-v1.3.0-local.apk)
- 全局坐标系版本（仅特殊场景）: [XRoboToolkit-v1.3.0-global.apk](https://github.com/lzhu686/XRoboToolkit-Unity-Client/releases/download/v1.3.0/XRoboToolkit-v1.3.0-global.apk)

**v1.2.0**
- [XRoboToolkit-v1.2.0.apk](https://github.com/lzhu686/XRoboToolkit-Unity-Client/releases/download/v1.2.0/XRoboToolkit-v1.2.0.apk)

## 坐标系模式说明

| 模式 | 稳定性 | 推荐使用 |
|------|--------|----------|
| **Local（本地坐标系）** | ⭐⭐⭐⭐⭐ 更稳定 | ✅ 推荐日常使用 |
| **Global（全局坐标系）** | ⭐⭐⭐ 可能不稳定 | 仅多设备空间对齐场景 |

**Local 模式优势：**
- 相对于设备初始位置的坐标系
- 不依赖环境特征点，追踪更稳定可靠
- 适合单设备使用、机械臂遥操作等日常开发

**Global 模式限制：**
- 依赖环境空间锚点和特征点
- 可能因光照变化、环境遮挡导致追踪不稳定
- 仅在需要多设备空间对齐时使用

## 安装

```bash
# 1. 下载 APK（推荐 local 版本）
wget https://github.com/lzhu686/XRoboToolkit-Unity-Client/releases/download/v1.3.0/XRoboToolkit-v1.3.0-local.apk

# 2. 安装到 PICO 头显
adb install -r -g XRoboToolkit-v1.3.0-local.apk
```

## 所有 Release 版本

查看所有版本: https://github.com/lzhu686/XRoboToolkit-Unity-Client/releases

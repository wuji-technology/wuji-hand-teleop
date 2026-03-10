#!/bin/bash
# =============================================================================
# ADB Reverse 端口监控
# =============================================================================
# PICO USB 连接的 ADB reverse 是 session 级别的, 断连后丢失。
# 此脚本以后台进程运行, 每 5 秒检测 PICO USB 连接状态:
#   - 连接时: 自动建立 reverse 端口映射
#   - 断开时: 静默等待重连
#
# 端口:
#   63901 → PC-Service 控制通道 (PICO Connect 按钮)
#   13579 → XRoboCompatServer 相机命令通道
#
# HTC/WiFi 模式下无 USB 设备, 空转无副作用。
# =============================================================================

# 忽略 SIGINT/SIGTERM — 不随 ros2 launch 的 Ctrl+C 退出
trap '' INT TERM

PORTS="63901 13579"
INTERVAL=5
LAST_STATE=""

while true; do
    if adb devices 2>/dev/null | grep -q "device$"; then
        # 每次循环都确认 reverse 端口存在 (可能被 PICO 断连清除)
        MISSING=false
        for PORT in $PORTS; do
            if ! adb reverse --list 2>/dev/null | grep -q "tcp:$PORT"; then
                MISSING=true
                break
            fi
        done

        if [ "$MISSING" = "true" ]; then
            SERIAL=$(adb devices 2>/dev/null | grep "device$" | head -1 | cut -f1)
            [ "$LAST_STATE" != "connected" ] && echo "[ADB watchdog] PICO detected ($SERIAL)"
            for PORT in $PORTS; do
                adb reverse tcp:$PORT tcp:$PORT 2>/dev/null
            done
            echo "[ADB watchdog] Reverse ports set: $PORTS"
        fi
        LAST_STATE="connected"
    else
        if [ "$LAST_STATE" = "connected" ]; then
            echo "[ADB watchdog] PICO disconnected, waiting for reconnect..."
        fi
        LAST_STATE="disconnected"
    fi
    sleep $INTERVAL
done

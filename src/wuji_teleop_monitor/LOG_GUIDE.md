# Teleop Monitor 日志系统使用指南

## 概述

为了帮助定位监控面板和 terminal 自动停止运行的问题，已为 Teleop Monitor 添加了完善的日志系统。

## 日志位置

日志文件自动保存在以下目录：

```
~/.wuji_teleop/logs/
```

每次启动程序时会创建一个新的日志文件，文件名格式为：

```
teleop_monitor_YYYYMMDD_HHMMSS.log
```

例如：`teleop_monitor_20260128_143025.log`

## 查看日志

### 1. 实时查看最新日志

在程序运行时，可以实时查看日志输出：

```bash
tail -f ~/.wuji_teleop/logs/teleop_monitor_*.log | tail -1
```

或者更精确地：

```bash
# 找到最新的日志文件并实时查看
tail -f $(ls -t ~/.wuji_teleop/logs/teleop_monitor_*.log | head -1)
```

### 2. 查看完整日志

```bash
# 查看最新的日志文件
cat $(ls -t ~/.wuji_teleop/logs/teleop_monitor_*.log | head -1)
```

### 3. 搜索特定错误

```bash
# 搜索所有错误日志
grep "ERROR" ~/.wuji_teleop/logs/teleop_monitor_*.log

# 搜索所有异常日志
grep "异常" ~/.wuji_teleop/logs/teleop_monitor_*.log

# 搜索特定关键词
grep "停止" ~/.wuji_teleop/logs/teleop_monitor_*.log
```

## 日志级别

日志系统使用以下级别（从低到高）：

- **DEBUG**: 详细的调试信息（仅记录到文件）
- **INFO**: 一般信息（控制台和文件）
- **WARNING**: 警告信息（控制台和文件）
- **ERROR**: 错误信息（控制台和文件）
- **CRITICAL**: 严重错误（控制台和文件）

## 关键日志位置

以下是程序中记录日志的关键位置，这些地方最有可能暴露问题：

### 1. 程序启动和退出

- 程序启动时记录所有初始化步骤
- 程序退出时记录资源清理过程
- 信号处理 (Ctrl+C) 被记录

### 2. 后台扫描线程

- 每次设备扫描的开始和结束
- 扫描过程中的所有异常
- 线程启动和退出

### 3. UI 更新循环

- ROS2 spin 异常
- 话题状态更新异常
- UI 组件更新异常
- 关节角度更新异常

### 4. Launch 进程管理

- 进程启动命令和 PID
- 进程停止的三级信号流程 (SIGINT → SIGTERM → SIGKILL)
- 进程意外退出及退出码

### 5. 机器人连接

- 连接尝试和结果
- SDK 返回的错误码
- 断开连接过程
- 松闸/抱闸操作

### 6. 设备扫描

- USB 设备扫描 (Manus 手套、Wuji 灵巧手)
- 网络设备检测 (天机机械臂 ping)
- OpenVR 设备扫描 (Vive Tracker 和基站)
- StereoVR 相机检测
- 超时和异常

## 常见问题排查

### 问题 1: 监控面板突然停止更新

查看日志中的：
```bash
grep -E "UI 更新|spin_once|定时器" ~/.wuji_teleop/logs/teleop_monitor_*.log
```

可能原因：
- ROS2 spin_once 异常
- UI 更新循环中的异常
- 定时器意外停止

### 问题 2: 后台扫描线程卡住

查看日志中的：
```bash
grep -E "扫描线程|设备扫描|scan_worker" ~/.wuji_teleop/logs/teleop_monitor_*.log
```

可能原因：
- USB 扫描超时
- OpenVR 初始化阻塞
- ping 命令超时

### 问题 3: Launch 进程意外退出

查看日志中的：
```bash
grep -E "launch|进程|SIGINT|SIGTERM|SIGKILL" ~/.wuji_teleop/logs/teleop_monitor_*.log
```

可能原因：
- 进程收到信号
- 启动文件错误
- 依赖问题

### 问题 4: 程序完全崩溃

查看日志的最后几行：
```bash
tail -50 $(ls -t ~/.wuji_teleop/logs/teleop_monitor_*.log | head -1)
```

查找 CRITICAL 或 ERROR 级别的最后日志。

## 调整日志级别

如果需要更详细的调试信息，可以修改 `logger.py` 中的日志级别。

### 临时启用控制台 DEBUG 日志

编辑 `wuji_teleop_monitor/logger.py`，找到：

```python
# Console handler
console_handler = logging.StreamHandler(sys.stdout)
console_handler.setLevel(logging.INFO)  # 改为 logging.DEBUG
```

## 日志文件管理

### 清理旧日志

日志文件会自动轮转（单个文件最大 10MB，保留 5 个备份）。

手动清理旧日志：
```bash
# 清理 7 天前的日志
find ~/.wuji_teleop/logs/ -name "teleop_monitor_*.log*" -mtime +7 -delete
```

### 查看日志磁盘占用

```bash
du -sh ~/.wuji_teleop/logs/
```

## 提交 Bug 报告

如果需要报告问题，请包含以下信息：

1. 问题描述
2. 复现步骤
3. 相关的日志文件（最好是完整的日志文件）
4. 系统环境信息

导出日志：
```bash
# 打包最新的日志文件
tar -czf teleop_monitor_logs_$(date +%Y%m%d).tar.gz ~/.wuji_teleop/logs/teleop_monitor_*.log
```

## 注意事项

1. 日志文件可能包含系统路径等敏感信息，分享前请检查
2. 日志文件使用 UTF-8 编码，确保使用支持 UTF-8 的编辑器查看
3. 长时间运行可能产生大量日志，定期清理旧日志
4. 异常堆栈信息 (exc_info=True) 会记录完整的调用栈，便于定位问题

## 快速命令参考

```bash
# 查看最新日志的最后 50 行
tail -50 $(ls -t ~/.wuji_teleop/logs/teleop_monitor_*.log | head -1)

# 实时跟踪最新日志
tail -f $(ls -t ~/.wuji_teleop/logs/teleop_monitor_*.log | head -1)

# 搜索错误
grep -i error $(ls -t ~/.wuji_teleop/logs/teleop_monitor_*.log | head -1)

# 搜索警告
grep -i warning $(ls -t ~/.wuji_teleop/logs/teleop_monitor_*.log | head -1)

# 统计日志级别分布
grep -o "\[.*\]" $(ls -t ~/.wuji_teleop/logs/teleop_monitor_*.log | head -1) | sort | uniq -c
```

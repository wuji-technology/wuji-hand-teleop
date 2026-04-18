
# Teleop Monitor Log System Usage Guide

## Overview

To help diagnose issues where the monitoring panel and terminal automatically stop running, a comprehensive logging system has been added to the Teleop Monitor.

## Log Location

Log files are automatically saved in the following directory:

```
~/.wuji_teleop/logs/
```

A new log file is created each time the program starts, with the filename format:

```
teleop_monitor_YYYYMMDD_HHMMSS.log
```

For example: `teleop_monitor_20260128_143025.log`

## Viewing Logs

### 1. View Latest Log in Real-Time

While the program is running, you can view the log output in real-time:

```bash
tail -f ~/.wuji_teleop/logs/teleop_monitor_*.log | tail -1
```

Or more precisely:

```bash
# Find the latest log file and view in real-time
tail -f $(ls -t ~/.wuji_teleop/logs/teleop_monitor_*.log | head -1)
```

### 2. View Complete Log

```bash
# View the latest log file
cat $(ls -t ~/.wuji_teleop/logs/teleop_monitor_*.log | head -1)
```

### 3. Search for Specific Errors

```bash
# Search all error logs
grep "ERROR" ~/.wuji_teleop/logs/teleop_monitor_*.log

# Search all exception logs
grep "exception" ~/.wuji_teleop/logs/teleop_monitor_*.log

# Search for specific keywords
grep "stop" ~/.wuji_teleop/logs/teleop_monitor_*.log
```

## Log Levels

The logging system uses the following levels (from low to high):

- **DEBUG**: Detailed debug information (recorded to file only)
- **INFO**: General information (console and file)
- **WARNING**: Warning messages (console and file)
- **ERROR**: Error messages (console and file)
- **CRITICAL**: Critical error messages (console and file)

## Key Log Locations

The following are key locations in the program where logs are recorded, most likely to expose issues:

### 1. Program Startup and Exit

- All initialization steps are logged at program startup
- Resource cleanup process is logged at program exit
- Signal handling (Ctrl+C) is logged

### 2. Background Scan Thread

- Start and end of each device scan
- All exceptions during scanning
- Thread startup and exit

### 3. UI Update Loop

- ROS2 spin exceptions
- Topic status update exceptions
- UI component update exceptions
- Joint angle update exceptions

### 4. Launch Process Management

- Process start command and PID
- Three-level signal sequence for process termination (SIGINT → SIGTERM → SIGKILL)
- Unexpected process exit and exit code

### 5. Robot Connection

- Connection attempts and results
- Error codes returned by SDK
- Disconnection process
- Brake release/engage operations

### 6. Device Scanning

- USB device scanning (MANUS gloves, Wuji Hand)
- Network device detection (Tianji arm ping)
- OpenVR device scanning (Vive Trackers and base stations)
- StereoVR camera detection
- Timeouts and exceptions

## Common Issue Troubleshooting

### Issue 1: Monitoring Panel Suddenly Stops Updating

Check the logs for:
```bash
grep -E "UI update|spin_once|timer" ~/.wuji_teleop/logs/teleop_monitor_*.log
```

Possible causes:
- ROS2 spin_once exception
- Exception in UI update loop
- Timer unexpectedly stopped

### Issue 2: Background Scan Thread Hangs

Check the logs for:
```bash
grep -E "scan thread|device scan|scan_worker" ~/.wuji_teleop/logs/teleop_monitor_*.log
```

Possible causes:
- USB scan timeout
- OpenVR initialization blocking
- ping command timeout

### Issue 3: Launch Process Exits Unexpectedly

Check the logs for:
```bash
grep -E "launch|process|SIGINT|SIGTERM|SIGKILL" ~/.wuji_teleop/logs/teleop_monitor_*.log
```

Possible causes:
- Process received signal
- Launch file error
- Dependency issues

### Issue 4: Program Crashes Completely

View the last few lines of the log:
```bash
tail -50 $(ls -t ~/.wuji_teleop/logs/teleop_monitor_*.log | head -1)
```

Look for the last CRITICAL or ERROR level log entries.

## Adjusting Log Levels

If more detailed debug information is needed, you can modify the log level in `logger.py`.

### Temporarily Enable Console DEBUG Logging

Edit `wuji_teleop_monitor/logger.py`, find:

```python
# Console handler
console_handler = logging.StreamHandler(sys.stdout)
console_handler.setLevel(logging.INFO)  # Change to logging.DEBUG
```

## Log File Management

### Cleaning Old Logs

Log files automatically rotate (maximum 10MB per file, 5 backups retained).

Manually clean old logs:
```bash
# Clean logs older than 7 days
find ~/.wuji_teleop/logs/ -name "teleop_monitor_*.log*" -mtime +7 -delete
```

### Check Log Disk Usage

```bash
du -sh ~/.wuji_teleop/logs/
```

## Submitting Bug Reports

If you need to report an issue, please include the following information:

1. Problem description
2. Steps to reproduce
3. Relevant log files (preferably the complete log file)
4. System environment information

Export logs:
```bash
# Package the latest log files
tar -czf teleop_monitor_logs_$(date +%Y%m%d).tar.gz ~/.wuji_teleop/logs/teleop_monitor_*.log
```

## Important Notes

1. Log files may contain sensitive information such as system paths; please review before sharing
2. Log files use UTF-8 encoding; make sure to use a UTF-8 compatible editor to view them
3. Long-running sessions may generate large amounts of logs; clean old logs periodically
4. Exception stack traces (exc_info=True) record the complete call stack for easier debugging

## Quick Command Reference

```bash
# View last 50 lines of the latest log
tail -50 $(ls -t ~/.wuji_teleop/logs/teleop_monitor_*.log | head -1)

# Follow the latest log in real-time
tail -f $(ls -t ~/.wuji_teleop/logs/teleop_monitor_*.log | head -1)

# Search for errors
grep -i error $(ls -t ~/.wuji_teleop/logs/teleop_monitor_*.log | head -1)

# Search for warnings
grep -i warning $(ls -t ~/.wuji_teleop/logs/teleop_monitor_*.log | head -1)

# Count log level distribution
grep -o "\[.*\]" $(ls -t ~/.wuji_teleop/logs/teleop_monitor_*.log | head -1) | sort | uniq -c
```

# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [2026.04.28] - 2026-04-28

### Changed

- Hand controller now runs as one process per side (no shared GIL/timer).
- **Breaking**: custom hand input now publishes `manus_ros2_msgs/ManusGlove` on `/manus_glove_{0,1}` (was `Float32MultiArray` on `/hand_input`).
- **Breaking**: hand mode services are now per-side at `/wuji_hand/{left,right}/{switch_mode,get_mode}`.
- Hand and HTC/Tianji arm controllers now default to 120 Hz (PICO arm stays at 90 Hz).
- New launch params `control_rate` and `nlopt_max_eval` (`0` keeps library default).

### Fixed

- Manus udev rule now also covers the wireless transceiver (`1915:83fd`) — without it the BLE skeleton stream was silently empty.
- Incomplete Manus frames are now dropped instead of substituted with the origin — the hand no longer snaps to `(0, 0, 0)`.

### Removed

- Removed the `manus_input` Python wrapper; controllers subscribe to MANUS topics directly.

## [2026.04.18] - 2026-04-18

### Added

- Added HTC Vive Tracker wearing guide with visual reference
- Added teleoperation demo video
- Added system dataflow architecture diagram
- Added udev rule so the MANUS USB dongle no longer requires sudo
- Added installation instructions for the Wuji Hand C++ SDK
- Documented minimum firmware version requirement with a link to the Wuji Hand firmware upgrade tool
- Added per-hand pinch threshold configuration for both left and right hands
- Added placeholder serial numbers to config templates so users fill in their own hardware values

### Changed

- Rewrote README with a linear step-by-step setup flow
- Replaced sudo-based USB permissions with udev rules so launch files no longer require elevated privileges
- Updated PICO XRoboToolkit APK download links to match current release filenames
- Changed the hand retargeting install command from editable mode to a regular install

### Fixed

- Fixed a package name conflict that could block the workspace build
- Fixed a missing system dependency required for the MANUS build
- Fixed documentation for the `/hand_input` topic QoS policy
- Fixed several inaccurate default values in configuration documentation
- Fixed a crash in the Tianji Arm controller when a required file was missing
- Fixed the MANUS calibration guide to reflect separate left- and right-hand calibration files
- Fixed camera launch documentation to reflect the actual default behavior

## [0.1.0] - 2026-03-10

### Added

- ROS2-based hand and arm teleoperation system
- Input support for MANUS data glove, HTC Vive Tracker, and PICO VR controller/tracker
- Output support for Wuji Hand retargeting and Tianji Arm IK

[Unreleased]: https://github.com/wuji-technology/wuji-hand-teleop/compare/v2026.04.28...HEAD
[2026.04.28]: https://github.com/wuji-technology/wuji-hand-teleop/compare/v2026.04.18...v2026.04.28
[2026.04.18]: https://github.com/wuji-technology/wuji-hand-teleop/compare/v0.1.0...v2026.04.18
[0.1.0]: https://github.com/wuji-technology/wuji-hand-teleop/releases/tag/v0.1.0

# MANUS Glove Input

MANUS Metaglove Pro driver and configuration for `wuji-hand-teleop`.

> [!IMPORTANT]
> **Community-supported, feature-frozen as of 2026-06-03.** MANUS is no longer
> surfaced in the Monitor cockpit, dropped from the main README, and will not
> receive new features or bug-fix releases. The package stays in the tree so
> existing users can keep running — this README is now the sole source of truth
> for setup, calibration, and troubleshooting.
>
> **Recommended hand input is now [Wuji Glove](../wuji_glove/)** (UDP via
> `wuji-sdk`). See the main [README §Quick Start](../../../README.md#quick-start-docker)
> — the cockpit, retargeting defaults, and SN scanner are all built around the
> Wuji Glove path.
>
> If you hit an issue here, please open a GitHub issue, but expect best-effort
> community responses rather than maintainer fixes.

This page consolidates all MANUS-specific setup — calibration, USB dongle udev rule,
glove ID configuration, and retargeting parameters — that used to live in the main README.

---

## 1. Prerequisites

MANUS support depends on two artifacts that ship with this repo:

| Component | Location | How to pull |
|---|---|---|
| `manus_ros2` C++ driver + `libManusSDK.so` (binary, ~250 MB) | `src/input_devices/manus_input/manus_ros2/ManusSDK/lib/` | Git LFS — run `git lfs install && git lfs pull` after cloning |
| `99-manus-libusb.rules` udev rule | `src/input_devices/manus_input/config/udev/` | Bundled with the repo |

> **Warning**: if `git lfs pull` was skipped, `libManusSDK.so` is a Git LFS pointer
> file (not a real `.so`) and `manus_ros2` will fail to link. Confirm with
> `file src/input_devices/manus_input/manus_ros2/ManusSDK/lib/libManusSDK.so` —
> output should report ELF, not ASCII text.

---

## 2. USB dongle udev rule

Grants non-root access to the MANUS USB dongle (incl. the wireless transceiver
`1915:83fd`), so `manus_data_publisher` runs without sudo:

```bash
cd ~/ros2_ws/src/wuji-hand-teleop
sudo cp src/input_devices/manus_input/config/udev/99-manus-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Replug the dongle (or reboot) after installing the rule. Verify with
`lsusb -d 3325:` — it should list the MANUS dongle.

---

## 3. Calibration

A per-user calibration file is required for each hand. MANUS Core (Windows) is
the only supported calibration tool.

1. Install **MANUS Core** on a Windows PC.
2. Connect both gloves and run the in-app calibration flow.
3. Export calibration files (`.mcal` format) for left and right hand.
4. Copy the files onto the teleop machine:

   ```bash
   cp /path/to/left_calibration.mcal  src/input_devices/manus_input/manus_ros2/calibration/LeftMetaglovePro.mcal
   cp /path/to/right_calibration.mcal src/input_devices/manus_input/manus_ros2/calibration/RightMetaglovePro.mcal
   ```

The driver auto-loads these on startup and prints
`Calibration loaded successfully for Left/Right glove`.

---

## 4. Glove ID configuration

Glove IDs are assigned by the `manus_ros2` C++ driver in the order MANUS Core
enumerates the gloves. Each controller process subscribes to both
`/manus_glove_0` and `/manus_glove_1` and routes by `msg.side`
(`"left"` / `"right"`), so glove ID order does not need to match left/right —
the side is carried in the message payload.

If you have multiple pairs of gloves and need to bind a specific physical
glove to side, set the canonical pair as primary in MANUS Core (Windows) so
they enumerate as id 0/1 first.

---

## 5. Retargeting parameters (advanced)

Per-hand config files at `src/output_devices/wujihand_output/config/`:

| Input source | Config file | Note |
|---|---|---|
| MANUS (right) | `retarget_manus_right.yaml` | `mediapipe_rotation.z = -15.0` |
| MANUS (left) | `retarget_manus_left.yaml` | `mediapipe_rotation.z = +15.0` |

Key parameters:

```yaml
retarget:
  mediapipe_rotation:
    x: 0.0
    y: 0.0
    z: -15.0             # MANUS right: -15, left: +15

  segment_scaling:       # Finger segment length scaling
    thumb:  [0.98, 1, 1]
    index:  [1.1, 0.989, 1.1]

  lp_alpha: 0.3          # Low-pass filter coefficient (smaller = smoother)
```

> The only difference between left and right is `mediapipe_rotation.z`. When
> tuning shared parameters (`pinch_thresholds`, `segment_scaling`, `lp_alpha`),
> update both files together.

---

## 6. Running with MANUS

After calibration + udev are in place:

1. Select MANUS in `src/output_devices/wujihand_output/config/wujihand_ik.yaml`:

   ```yaml
   input_source: "manus"
   ```

   (Default is `wuji_glove`. There is no longer a launch-time `hand_input:=`
   argument — edit the yaml once and every launch file picks it up.)

2. Launch:

   ```bash
   # Hand-only (MANUS + Wuji Hand)
   ros2 launch wuji_teleop_bringup wuji_teleop_hand.launch.py

   # Full teleoperation (MANUS hand + tracker arm)
   ros2 launch wuji_teleop_bringup wuji_teleop.launch.py arm_input:=tracker
   ```

Each launch spawns two independent controller processes
(`wujihand_controller_left` / `wujihand_controller_right`), each subscribing to
`/manus_glove_0` and `/manus_glove_1` and filtering by `msg.side`.

### Verify

```bash
ros2 topic hz /manus_glove_0           # raw glove (target ~120 Hz)
ros2 topic hz /manus_glove_1
ros2 topic hz /left_hand/joint_commands   # retargeted (~120 Hz)
ros2 topic hz /right_hand/joint_commands
```

---

## 7. Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `LIBUSB_ERROR_ACCESS` | udev rule missing | Step 2 above; replug dongle |
| `manus_ros2` link error referencing `libManusSDK.so` | LFS not pulled | `git lfs install && git lfs pull` |
| `no valid SDK Integrated license` | Same as above (libManusSDK.so is a 134-byte LFS pointer) | `git lfs pull` |
| No `/manus_glove_*` data | Calibration files missing or wrong path | Step 3 above |
| Skeleton stream silently empty over BLE | Wireless transceiver missing from udev (`1915:83fd`) | Re-apply Step 2 (current rule covers it) |

If `colcon build` fails for `manus_ros2`, you can mark the package as ignored
to skip MANUS support entirely:

```bash
touch src/input_devices/manus_input/COLCON_IGNORE
```

The rest of the teleop stack (PICO arm, Wuji Hand, etc.) will build fine.

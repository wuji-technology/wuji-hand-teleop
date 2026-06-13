# Camera topics & the preview

The rig has three physical cameras: a **stereo head camera** (two eyes) and two
**wrist D405s**. They publish on the topics below. Names are intentionally left
as each driver publishes them (no system-wide rename); the Camera Preview just
subscribes to these directly.

## Topic map

| Feed | Topic | Publisher |
| --- | --- | --- |
| Head — left eye  | `/stereo/left/compressed`  | `stereo_camera_publisher` |
| Head — right eye | `/stereo/right/compressed` | `stereo_camera_publisher` |
| Left wrist  | `/cam_left_wrist/color/image_raw/compressed`  | `realsense2_camera` (`cam_left_wrist`) |
| Right wrist | `/cam_right_wrist/color/image_raw/compressed` | `realsense2_camera` (`cam_right_wrist`) |

All four are `sensor_msgs/CompressedImage` (JPEG).

> The head used to be a single mono camera on `/cam_head/color/image_raw/compressed`.
> After the stereo migration the head publishes the two eyes on `/stereo/{left,right}`.
> `/cam_head/...` is **no longer published** — old code/tools that subscribe to it
> will see nothing.

## Camera Preview layout

`ros2 run wuji_teleop_monitor camera` shows a **2×2 grid**:

```
┌────────────┬────────────┐
│ Head Left  │ Head Right │   /stereo/left      /stereo/right
├────────────┼────────────┤
│ Left Wrist │ Right Wrist│   /cam_left_wrist   /cam_right_wrist
└────────────┴────────────┘
```

A tile stays blank if its topic has no publisher (camera not started / crashed).

## Troubleshooting: a tile is blank

| Tile blank | Likely cause | Check |
| --- | --- | --- |
| Both head tiles | stereo publisher not running | `ros2 topic hz /stereo/left/compressed` |
| A wrist tile | that D405 node didn't start / crashed | `ros2 node list \| grep wrist`; launch log for `realsense2_camera_node ... died` |

### Configuring the wrist D405 serials (two different serials!)

Each wrist `realsense2_camera_node` opens its D405 **by serial number**, so the
two D405s must be told apart. Beware: a D405 has **two different serials**:

- the **RealSense serial** (e.g. `409122272382`) — what the ROS driver matches.
  Read it from the node log or `rs-enumerate-devices`.
- the **USB descriptor iSerial** (e.g. `254723074373`) — what `lsusb -v` /
  `udev` see. **Different value for the same camera.**

So fill:

- `config/camera_config.yaml` → `left_wrist`/`right_wrist` `serial_number:` with
  the **RealSense** serial.
- `config/udev/99-teleop-cameras.rules` → `ATTRS{serial}==` with the **USB**
  iSerial (udev matches the USB descriptor, not the RealSense serial).

If a node logs `The requested device with serial number … is NOT found` while
clearly listing other serials it *did* find, you put a USB iSerial where the
RealSense serial belongs (or vice-versa). Left/right is physical — if the
preview shows the wrists swapped, swap the two `serial_number:` values.

### Wrist D405 node dies on start (`exit code 1`)

Both wrist `realsense2_camera_node`s can die with `exit code 1` even though the
D405s enumerate on USB (`lsusb | grep 0b5b`). This is usually a **USB bandwidth /
controller contention** issue (two D405 colour streams on one USB host) or a
stream-profile mismatch in the node params — not a missing camera. Mitigations:

- Put the two D405s on **different USB 3.0 host controllers** (different physical
  ports / a powered hub per camera).
- Lower the wrist colour resolution/FPS in the camera launch params.
- Confirm with the standalone node: `ros2 run realsense2_camera realsense2_camera_node`.

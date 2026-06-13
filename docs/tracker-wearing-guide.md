# HTC Vive Tracker Wearing Guide

## Getting Started

Before wearing the Trackers, pair them with SteamVR and confirm their serial numbers. After assigning each Tracker to a body segment, write the serial mapping into the configuration file [`openvr_input.yaml`](https://github.com/wuji-technology/wuji-hand-teleop/blob/main/src/input_devices/openvr_input/config/openvr_input.yaml) in the wuji-hand-teleop project.

```yaml
tracker_serials:
  chest: "LHR-xxxxxxx1"         # Chest
  right_wrist: "LHR-xxxxxxx2"   # Right wrist
  left_wrist: "LHR-xxxxxxx3"    # Left wrist
  left_arm: "LHR-xxxxxxx4"      # Left upper arm
  right_arm: "LHR-xxxxxxx5"     # Right upper arm
```

> [!NOTE]
> SteamVR supports pairing Tracker in headset-free mode by configuring the Null Driver. For setup instructions, see [Configure Null Driver (Headset-Free Mode)](https://github.com/wuji-technology/wuji-hand-teleop/blob/main/docs/STEAMVR.md#2-configure-null-driver-headset-free-mode). For full usage instructions of wuji-hand-teleop, see [`README.md`](https://github.com/wuji-technology/wuji-hand-teleop).

## Wear the Tracker

Use the **USB-C port** on each Tracker as the orientation reference:

- **Arm Tracker**: USB-C port aligned with the arm bone, pointing **toward the torso**
- **Chest Tracker**: USB-C port faces **upward** along the torso

The complete wearing setup is shown below.

![Tracker wearing setup](images/tracker-wearing-combined.jpg)

> [!NOTE]
>
> - Mount the upper arm Tracker on the outer side of the arm, as shown in the figure.
> - Keep the chest Tracker parallel to the torso and secure it to prevent wobbling. Maintain the torso as upright as possible.
> - Ensure all five Trackers remain visible to at least one SteamVR Base Station during operation.
> - When adjusting body posture, keep the relative position between arms and torso unchanged, and move slowly.

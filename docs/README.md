# `docs/` — User-Facing Setup Guides

Anything in this folder is meant to be read **once, before first use**, by an operator setting up the hardware. Developer-facing architecture notes live next to the code they describe (e.g. `src/input_devices/pico_input/ARCHITECTURE.md`).

The main entry point for the whole project is [`../README.md`](../README.md). The guides below cover the bits that are too long or too device-specific to put inline there.

## Which guide do I need?

| If you are setting up… | Read |
|---|---|
| The **HTC Vive Tracker** arm path (default, `wuji_teleop.launch.py`) — SteamVR null driver, base stations, dongles, tracker pairing | [STEAMVR.md](STEAMVR.md) |
| The **PICO 4 + Motion Trackers** arm path (`pico_teleop.launch.py`) — Developer Mode, XRoboToolkit APK, ADB reverse-forwarding, H.264 stereo streaming | [PICO.md](PICO.md) |
| **Mounting trackers on the body** — straps, orientation, demo video | [tracker-wearing-guide.md](tracker-wearing-guide.md) |

Pick one arm path. The two are alternatives; you don't need both.

## What this folder is not

- **Not** a code reference. For ROS2 topics, node parameters, and per-package layout, see the package READMEs under `src/`.
- **Not** an architecture deep-dive. For PICO coordinate transforms, incremental-control math, and arm-angle geometry, see [`src/input_devices/pico_input/ARCHITECTURE.md`](../src/input_devices/pico_input/ARCHITECTURE.md).
- **Not** install instructions for the codebase itself. The supported deployment is Docker — see [Quick Start (Docker)](../README.md#quick-start-docker) in the main README. The Dockerfile is the canonical recipe for host dependencies if you want to roll a bare-metal install yourself (unsupported).

## Media

`images/` holds figures linked from the guides above. `teleop-demo.mp4` and `tracker-wearing-demo.mp4` are operator-facing demo clips referenced from the main README and from `tracker-wearing-guide.md`.

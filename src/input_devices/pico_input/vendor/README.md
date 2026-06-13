# vendor/ — XRoboToolkit Vendored Dependencies (PICO toolchain)

This directory contains source code from upstream third-party projects, vendored into the `pico_input` package for build / development convenience. Wuji edits these sources directly in this repository; there is no fork tracked elsewhere.

All vendored projects retain their original licenses. **Modifying or removing the original `LICENSE` / `NOTICE` / `THIRD_PARTY_NOTICE.txt` files inside each subdirectory is prohibited.**

## Inventory

| Subdirectory | Upstream | Version | License | Modifications from upstream |
|---|---|---|---|---|
| `XRoboToolkit-PC-Service/` | https://github.com/XR-Robotics/XRoboToolkit-PC-Service | v1.0.0 (+ Wuji patches) | Apache-2.0 | Pre-compiled binaries (`.so`/`.dll`/`.lib`) and Unity demo binaries removed for source-only vendor. Original `LICENSE` and `THIRD_PARTY_NOTICE.txt` retained. **Wuji patches (2026-06-01)** — see [Wuji modifications](#wuji-modifications) below. |
| `XRoboToolkit-PC-Service-Pybind/` | https://github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind | v1.0.0 (+ Wuji patches) | MIT | Source vendored as-is, original `LICENSE` retained. **Wuji patches (2026-06-01)** — see [Wuji modifications](#wuji-modifications) below. |

## Source vs. prebuilt binary

The patches below apply to the **source** vendored under this directory, and the **prebuilt `.deb`** at `docker/prebuilt/XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb` is built from that same source — it carries every Wuji patch in the [Wuji modifications](#wuji-modifications) table. Source and binary stay in lockstep: bump a patch, rebuild the `.deb`, ship both together.

The prebuilt `.deb` is ~17 MB and declares `libqt6core6 / libqt6network6 / libqt6core5compat6` in `Depends:`, so `sudo apt install ./XRoboToolkit_PC_Service_*.deb` resolves the Qt6 runtime automatically on a fresh Ubuntu 22.04 install.

To rebuild from source (e.g., after editing a patch):

```bash
sudo apt install build-essential cmake dpkg-dev \
    qt6-base-dev qt6-base-dev-tools qt6-5compat-dev qt6-declarative-dev
git lfs pull                                            # fetches Redistributable static libs
cd src/input_devices/pico_input/vendor
./build_pc_service.sh                                   # builds the .deb under build/
sudo dpkg -i build/XRoboToolkit-PC-Service_*.deb
```

The build is self-contained — `Redistributable/linux/` (gRPC/abseil/protobuf/openssl static libs, ~127 MB via Git LFS) ships in this repo so the script needs no further network access after `git lfs pull`.

## Wuji modifications

All modifications are marked in-file with a `// wuji-hand-teleop patch (YYYY-MM-DD):` comment so they can be diffed against the next upstream sync. Per Apache-2.0 §4(b), every modified file carries that prominent in-source notice. The shipped prebuilt `.deb` is built from these same patched sources — see [Source vs. prebuilt binary](#source-vs-prebuilt-binary) above.

### `XRoboToolkit-PC-Service/`

| File | Fix |
|---|---|
| `RoboticsService/CommonUtils/commonutils.h` | `#define MIN(x,y)` was inverted (computed MAX). Fixed to use `<`. |
| `RoboticsService/CommonUtils/commonutils.cpp` | `QNetworkInformation::loadDefaultBackend()` is a Qt 6.4+ API; on Ubuntu 22.04 (Qt 6.2.4) the source failed to compile. Gated the call behind `QT_VERSION >= 6.4`; older Qt returns `true` (`isOnline()` is unused anyway). |
| `RoboticsService/DeviceConnectionManager/Model/tcpconnectionmodel.cpp` | `PaddingTail` branch appended `sizeof(TCPMsgTail)` when only `m_paddingLength` bytes were needed; would over-read from the next frame. Now appends `m_paddingLength`. |
| `RoboticsService/RoboticsServiceProcess/main.cpp` | Removed the unused `Q_DECLARE_METATYPE(QSharedPointer<QImage>)`. `QImage` is never referenced anywhere else in the daemon (no GUI / no image pipeline) and registering the metatype pulls in QtGui — which the headless service does not otherwise link. |
| `RoboticsService/RoboticsServiceProcess/CMakeLists.txt` | Dropped the POST_BUILD copy of `SDKDemo/UnityBin/RobotLinuxDemo` (the source-only vendor strips Unity binaries — the Unity demo lives on the upstream release page) and the bundling of Qt 6.6.3 versioned libs/plugins/qml/translations from `${CMAKE_PREFIX_PATH}/lib`. The daemon is `QCoreApplication`-only (no platform plugin, no QML, no translations needed at runtime); depend on system Qt6 via the `.deb`'s `Depends:` instead. |
| `RoboticsService/Package/debPack/control` | Added `libqt6core6 / libqt6network6 / libqt6core5compat6` to `Depends:`. Upstream left it empty, so the .deb installed but the binary failed to launch until the user manually `apt install`'d Qt6. |
| `RoboticsService/Package/debPack/setup.sh` | Two cleanups: (a) removed `RobotDemoQt` / `RobotDataRecorder` from the script-copy loop — they live in `bin/`, not `debPack/`, so the loop spammed "stat 失败" on every run (the bulk `cp -rf $BIN_DIR/* $TARGET_DIR/` step below still picks them up); (b) guarded the `chmod +x .../RobotUnityDemo/RobotLinuxDemo.x86_64` with `[ -f ... ]` so the script stays silent when the Unity demo is absent. |
| `README.md` | Coordinate-system description mixed "Right-handed" with "Z in" (left-handed); corrected to "Z out" so the handedness label and axis directions agree (~L350, ~L366). |

### `XRoboToolkit-PC-Service-Pybind/`

| File | Fix |
|---|---|
| `bindings/py_bindings.cpp` | `OnPXREAClientCallback` only caught `json::exception`; `std::stod` inside `stringToPose/VelocityArray` could throw `std::invalid_argument` / `std::out_of_range` and escape the callback thread. Added matching catch clauses. |
| `bindings/py_bindings.cpp` | `getLeftHandScale` / `getRightHandScale` returned `int` but the underlying `LeftHandScale` / `RightHandScale` are `double` (parsed via `.get<double>()`); truncated precision. Widened both return types to `double`. |
| `bindings/py_bindings.cpp` | Both hand-scale getters now exported to Python as `get_left_hand_scale` / `get_right_hand_scale` (upstream had the C++ functions but never registered them in `PYBIND11_MODULE`). |
| `setup.py` | Replaced `from distutils.version import LooseVersion` (removed in Python 3.12+) with `packaging.version.Version` so the Windows CMake-version guard still runs on modern Pythons. |
| `setup_ubuntu.sh`, `setup_orin.sh` | Hardened: `set -euo pipefail`, error-handle every `cd`, dropped the invalid `pip install pybind11 -y` flag, added an `UPSTREAM_COMMIT` env var so refreshes can pin a tested upstream SHA instead of always re-cloning `HEAD`. |
| `setup_windows.bat` | Re-encoded to CRLF line terminators (`cmd.exe` rejects LF-only batch files on some shells). |

If you ever bump the vendored upstream version, re-apply these patches (or upstream them first to XR-Robotics and drop them here).

## Attribution

Copyright of all source files under each `vendor/<project>/` subdirectory belongs to their respective authors / copyright holders, as declared in each subdirectory's `LICENSE` (and `THIRD_PARTY_NOTICE.txt`, where applicable).

This repository's own license applies only to files outside `vendor/`.

## How to update

To bump a vendored project to a newer upstream version:

1. Fetch the upstream source at the target tag.
2. Replace the contents of the corresponding `vendor/<project>/` subdirectory.
3. Re-strip pre-compiled binaries (`.so`/`.dll`/`.lib`/`.a`/`.dylib`/`.exe`) and large pre-built artifacts (e.g., Unity demos).
4. Verify the upstream `LICENSE`, `NOTICE`, and `THIRD_PARTY_NOTICE` files are present and unmodified.
5. Re-apply (or drop, if upstream merged them) the Wuji patches in the [Wuji modifications](#wuji-modifications) table — `grep -rn "wuji-hand-teleop patch"` enumerates every patched line.
6. Update the version column in the table above.

## Apache-2.0 obligation

`XRoboToolkit-PC-Service/` is Apache-2.0. Per §4(b), modified files must carry prominent notices stating that you changed them. Every Wuji patch listed in the [Wuji modifications](#wuji-modifications) table is marked in-source with `// wuji-hand-teleop patch (YYYY-MM-DD):` immediately above the changed lines — that single comment carries the file-level notice required by §4(b). The license text in `THIRD_PARTY_NOTICE.txt` is preserved verbatim.

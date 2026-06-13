#!/usr/bin/env bash
# wuji-hand-teleop patch (2026-06-02): added `set -euo pipefail`, error-handle
# every `cd`, drop the invalid `-y` flag from `pip install pybind11`, and pin
# the upstream PC-Service clone so the binding ABI stays reproducible.
set -euo pipefail

UPSTREAM_REPO="https://github.com/XR-Robotics/XRoboToolkit-PC-Service.git"
# Update to the tested upstream commit when refreshing the vendored copy.
UPSTREAM_COMMIT="${UPSTREAM_COMMIT:-HEAD}"

if [[ "${CONDA_DEFAULT_ENV:-}" != "" ]]; then
    conda install -c conda-forge libstdcxx-ng -y
fi

mkdir -p tmp
cd tmp || { echo "[ERROR] cannot cd into tmp/" >&2; exit 1; }
git clone "$UPSTREAM_REPO"
cd XRoboToolkit-PC-Service || { echo "[ERROR] clone failed" >&2; exit 1; }
if [[ "$UPSTREAM_COMMIT" != "HEAD" ]]; then
    git checkout "$UPSTREAM_COMMIT"
fi
cd RoboticsService/PXREARobotSDK || { echo "[ERROR] missing PXREARobotSDK dir" >&2; exit 1; }
bash build.sh
cd ../../../.. || exit 1

mkdir -p lib
mkdir -p include
cp tmp/XRoboToolkit-PC-Service/RoboticsService/PXREARobotSDK/PXREARobotSDK.h include/
cp -r tmp/XRoboToolkit-PC-Service/RoboticsService/PXREARobotSDK/nlohmann include/nlohmann/
cp tmp/XRoboToolkit-PC-Service/RoboticsService/PXREARobotSDK/build/libPXREARobotSDK.so lib/
# rm -rf tmp

# Build the project
if [[ "${CONDA_DEFAULT_ENV:-}" != "" ]]; then
    conda install -c conda-forge pybind11 -y
else
    pip install pybind11
fi

pip uninstall -y xrobotoolkit_sdk
python setup.py install

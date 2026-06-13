#!/bin/bash
# Back-compat shim — kept so the README and any existing teleop-monitor.desktop
# shortcut keep working. The real implementation moved to launch_ui_docker.sh
# when brake and camera UIs gained their own desktop entries.
exec "$(dirname "${BASH_SOURCE[0]}")/launch_ui_docker.sh" monitor

#!/usr/bin/env bash
# Build the ROS 2 package. Usage: ./scripts/build.sh
set -euo pipefail
source "$(dirname "$0")/ros_env.sh"
cd "$BVI_PROJECT/ros2_ws"
colcon build            # (--symlink-install is incompatible with this setuptools)
set +u
source install/setup.bash
echo "built: $(ros2 pkg executables bvi_planner | tr '\n' ' ')"

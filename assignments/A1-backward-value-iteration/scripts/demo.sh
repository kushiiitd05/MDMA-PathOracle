#!/usr/bin/env bash
# Full live demo: map + BVI/ABVI values + paths + 2 animated robots + live
# optimality checker + RViz2.
# Usage: ./scripts/demo.sh [easy|medium|hard] [seed] [bvi.rviz|bvi_policy.rviz]
# Ctrl+C to stop.
set -euo pipefail
level="${1:-medium}"
seed="${2:--1}"
rviz_config="${3:-bvi.rviz}"
case "$level" in easy|medium|hard) ;; *) echo "difficulty must be easy|medium|hard"; exit 1;; esac
source "$(dirname "$0")/ros_env.sh"
exec ros2 launch bvi_planner bvi.launch.py difficulty:="$level" seed:="$seed" rviz_config:="$rviz_config"

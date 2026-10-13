# Usage:  source scripts/ros_env.sh
# Activates ROS 2 Humble and, if built, this project's workspace.
# Works with an apt install (/opt/ros/humble) or a conda/RoboStack env that is already active.
# ROS activation scripts reference unset variables: relax `set -u` while sourcing.
case $- in *u*) _bvi_had_u=1; set +u ;; *) _bvi_had_u=0 ;; esac
BVI_PROJECT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export BVI_PROJECT
if [ -z "${ROS_DISTRO:-}" ] && [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi
if [ -f "$BVI_PROJECT/ros2_ws/install/setup.bash" ]; then
    source "$BVI_PROJECT/ros2_ws/install/setup.bash"
fi
# Private DDS domain + localhost only, so other ROS nodes on the network never see ours.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-73}"
export ROS_LOCALHOST_ONLY=1
[ "$_bvi_had_u" = 1 ] && set -u
unset _bvi_had_u
echo "ROS 2 ${ROS_DISTRO:-?} ready | domain $ROS_DOMAIN_ID | project $BVI_PROJECT"

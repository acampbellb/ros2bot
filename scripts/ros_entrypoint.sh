#!/bin/bash
set -e

ros_env_setup="/opt/ros/$ROS_DISTRO/install/setup.bash"
echo "SOURCING.. $ros_env_setup"
source "$ros_env_setup"

ros_ws_setup="$HOME/$ROS_WORKSPACE/install/local_setup.bash"
echo "SOURCING.. $ros_ws_setup"
source "$ros_env_setup"

echo "ROS_DISTRO:    $ROS_DISTRO"
echo "ROS_ROOT:      $ROS_ROOT"
echo "ROS_WORKSPACE: $ROS_WORKSPACE"

exec "$@"
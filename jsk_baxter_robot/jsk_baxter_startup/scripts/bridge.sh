#!/bin/bash
# The mixed ROS environment belongs only to this bridge process.
set -e
if [[ -z ${ROS_MASTER_URI:-} ]]; then
  echo 'Set ROS_MASTER_URI=http://BAXTER_HOST:11311 and ROS_IP to this PC address.' >&2
  exit 2
fi
baxter_master_uri=$ROS_MASTER_URI
source "${BAXTER_ROS1_SETUP:-/opt/ros/one/setup.bash}"
source "${BAXTER_ROS2_SETUP:-$HOME/colcon_ws/install/setup.bash}"
export ROS_MASTER_URI=$baxter_master_uri
exec ros2 launch jsk_baxter_startup bridge.launch.xml "$@"

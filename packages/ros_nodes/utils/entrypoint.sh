#!/usr/bin/env bash
#
# Entry-point for app-scene-viewer
#
# Copyright 2021 Human Dataware Lab. Co. Ltd.
# Created by Daiki Hayashi (hayashi.daiki@hdwlab.co.jp)
#

# Source setup.bash if necessary
require_setup=false
[[ $1 =~ run.sh ]] \
  || [[ $1 =~ /opt/ros_nodes/utils/run.sh ]] \
  || require_setup=true

if ${require_setup}; then
  if ! ${DISABLE_ROS1:-false}; then
    echo "Using ROS1"
    # unsetting ROS_DISTRO to silence ROS_DISTRO override warning
    unset ROS_DISTRO
    # setup ros1 environment
    source "/opt/ros/${ROS1_DISTRO}/setup.bash"
    source "/opt/rosbridge-rosbag-player/devel/setup.bash"
  fi

  if ! ${DISABLE_ROS2:-false}; then
    echo "Using ROS2"
    # unsetting ROS_DISTRO to silence ROS_DISTRO override warning
    unset ROS_DISTRO
    # setup ros2 environment
    source "/opt/ros/${ROS2_DISTRO}/setup.bash"
    source "/opt/ros_nodes/install/setup.bash"
  fi
fi

# Start rosbridge for debugging
if [[ $1 =~ python3 ]]; then
  if ! ${DISABLE_ROSBRIDGE:-false}; then
    bash -c "ROS2_LAUNCH_FILE=/opt/ros_nodes/launch/ros2.dev.launch.xml /opt/ros_nodes/utils/run.sh" &
  fi
fi

# Execute commands
exec "$@"

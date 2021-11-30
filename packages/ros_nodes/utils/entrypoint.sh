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
  # unsetting ROS_DISTRO to silence ROS_DISTRO override warning
  unset ROS_DISTRO
  # setup ros1 environment
  source "/opt/ros/${ROS1_DISTRO}/setup.bash"
  source "/opt/rosbridge-rosbag-player/devel/setup.bash"

  # unsetting ROS_DISTRO to silence ROS_DISTRO override warning
  unset ROS_DISTRO
  # setup ros2 environment
  source "/opt/ros/${ROS2_DISTRO}/setup.bash"
  source "/opt/ros_nodes/install/setup.bash"
fi

# Execute commands
exec "$@"

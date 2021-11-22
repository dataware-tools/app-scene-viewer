#!/usr/bin/env bash
#
# Entry-point for app-scene-viewer
#
# Copyright 2021 Human Dataware Lab. Co. Ltd.
# Created by Daiki Hayashi (hayashi.daiki@hdwlab.co.jp)
#

# unsetting ROS_DISTRO to silence ROS_DISTRO override warning
unset ROS_DISTRO
# setup ros1 environment
source "/opt/ros/${ROS1_DISTRO}/setup.bash"

# unsetting ROS_DISTRO to silence ROS_DISTRO override warning
unset ROS_DISTRO
# setup ros2 environment
source "/opt/ros/${ROS2_DISTRO}/setup.bash"

# Start roscore for debugging
require_roscore=false
[[ $1 =~ ros* ]] || [[ $1 =~ run.sh ]] || [[ $1 =~ bash ]] || [[ $(rosnode list 2>/dev/null) ]] || require_roscore=true

if ${require_roscore}; then
  roscore &

  echo "Waiting for roscore to become ready..."
  while true;
  do
    sleep 1
    rosnode list > /dev/null 2>&1 && break
  done
fi

set -x

# Execute commands
exec "$@"

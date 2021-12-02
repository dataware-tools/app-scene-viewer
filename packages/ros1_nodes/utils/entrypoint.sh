#!/usr/bin/env bash
#
# Entry-point for app-scene-viewer
#
# Copyright 2021 Human Dataware Lab. Co. Ltd.
# Created by Daiki Hayashi (hayashi.daiki@hdwlab.co.jp)
#

source /opt/ros1_nodes/devel/setup.bash

# Activate python3
[[ ! -d /tmp/python3 ]] && mkdir -p /tmp/python3
[[ ! -L /tmp/python3/python ]] && ln -s /usr/bin/python3 /tmp/python3/python
export PATH=/tmp/python3:${PATH}

# Start rosbridge for debugging
if [[ $1 =~ python3 ]]; then
  if ! ${DISABLE_ROSBRIDGE:-false}; then
    bash -c "ROS1_LAUNCH_FILE=/opt/ros1_nodes/src/scene_viewer/launch/rosbridge.launch /opt/ros1_nodes/utils/run.sh" &
    while true; do
      sleep 1
      rosnode list > /dev/null 2>&1 && break
      echo "Waiting for roscore to be ready..."
    done
  fi
fi

exec "$@"

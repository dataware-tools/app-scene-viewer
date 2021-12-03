#!/usr/bin/env bash
#
# Run script
# Copyright 2021 Human Dataware Lab. Co. Ltd.
#

pids=()

# Start ROS1 nodes
bash <<"EOF" &
  unset ROS_DISTRO
  source "/opt/ros/${ROS1_DISTRO}/setup.bash"
  source "/opt/rosbridge-rosbag-player/devel/setup.bash"

  # Activate python3
  [[ ! -d /tmp/python3 ]] && mkdir -p /tmp/python3
  [[ ! -L /tmp/python3/python ]] && ln -s /usr/bin/python3 /tmp/python3/python
  export PATH=/tmp/python3:${PATH}

  roslaunch ${ROS1_LAUNCH_FILE:-/opt/ros2_nodes/launch/ros1.launch}

EOF
pids+=($!)

# Start ROS2 nodes
bash <<"EOF" &
  # Setup ROS1 environment
  unset ROS_DISTRO
  source "/opt/ros/${ROS1_DISTRO}/setup.bash"
  source "/opt/rosbridge-rosbag-player/devel/setup.bash"

  # Wait for roscore
  while true;
  do
    echo "waiting for ROS1's roscore to be ready"
    sleep 1
    rosnode list 2>&1 >/dev/null && break
  done

  # Setup ros2 environment
  unset ROS_DISTRO
  source "/opt/ros/${ROS2_DISTRO}/setup.bash"
  source "/opt/ros2_nodes/install/setup.bash"

  ros2 launch ${ROS2_LAUNCH_FILE:-/opt/ros2_nodes/launch/ros2.launch.xml}

EOF
pids+=($!)

# Wait for all jobs
wait ${pids[@]}
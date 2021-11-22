FROM ros:galactic-ros-base-focal
MAINTAINER Daiki Hayashi <hayashi.daiki@hdwlab.co.jp>
ENV LANG="en_US.UTF-8"
SHELL ["/bin/bash", "-c"]

ENV ROS1_DISTRO=noetic
ENV ROS2_DISTRO=galactic

# Apt update and rosdep update
RUN apt update \
    && apt upgrade -y \
    && apt install -y --no-install-recommends \
      curl \
      wget \
      vim \
      git \
      python3-pip \
    && rosdep update \
    && apt clean -y \
    && rm -rf /var/lib/apt/lists/*

# setup sources.list for ROS1
RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1-latest.list
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# install ros packages
RUN apt update && apt install -y --no-install-recommends \
    ros-${ROS1_DISTRO}-ros-comm \
    ros-${ROS1_DISTRO}-roscpp-tutorials \
    ros-${ROS1_DISTRO}-rospy-tutorials \
    ros-${ROS1_DISTRO}-rosbridge-suite \
    ros-${ROS1_DISTRO}-*-msgs \
  && rm -rf /var/lib/apt/lists/*

# install ros2 packages
RUN apt update && apt install -y --no-install-recommends \
    ros-${ROS2_DISTRO}-ros1-bridge \
    ros-${ROS2_DISTRO}-demo-nodes-cpp \
    ros-${ROS2_DISTRO}-demo-nodes-py \
    ros-${ROS2_DISTRO}-ros2bag \
    ros-${ROS2_DISTRO}-rosbag2* \
    ros-${ROS2_DISTRO}-*-msgs \
  && rm -rf /var/lib/apt/lists/*

# Install poetry
RUN python3 -m pip install --no-cache-dir --upgrade pip \
  && python3 -m pip install --no-cache-dir setuptools \
  && python3 -m pip install --no-cache-dir poetry \
  && rm -rf ~/.cache/pip \
  && poetry config virtualenvs.create false

## Install ros-nodes
#RUN mkdir -p /opt/ros_nodes/src
#COPY ./src /opt/ros_nodes/src
#RUN source /opt/ros/${ROS1_DISTRO}/setup.bash \
#  && cd /opt/ros_nodes \
#  && rosdep install --from-paths src --ignore-src -r -y \
#  && colcon build --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3 \
#  && rm -rf /var/lib/apt/lists/*

# Copy files
RUN mkdir -p /opt/ros_nodes/utils || :
COPY ./utils/* /opt/ros_nodes/utils

# Set default command
ENTRYPOINT ["/opt/ros_nodes/utils/entrypoint.sh"]
CMD ["/opt/ros_nodes/utils/run.sh"]

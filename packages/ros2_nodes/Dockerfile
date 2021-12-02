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
      python2.7 \
      python2.7-dev \
      python3-pip \
    && rosdep update \
    && apt clean -y \
    && rm -rf /var/lib/apt/lists/*

# Install pip2 for ROS1
RUN curl https://bootstrap.pypa.io/pip/2.7/get-pip.py | python2.7 \
    && pip2 install --no-cache-dir pyyaml

# Setup sources.list for ROS1
RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1-latest.list
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Install ros1 packages
RUN apt update && apt install -y --no-install-recommends \
    ros-${ROS1_DISTRO}-ros-comm \
    ros-${ROS1_DISTRO}-roscpp-tutorials \
    ros-${ROS1_DISTRO}-rospy-tutorials \
    ros-${ROS1_DISTRO}-rosbridge-suite \
    ros-${ROS1_DISTRO}-*-msgs \
  && rm -rf /var/lib/apt/lists/*

# Install ros2 packages
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

# Install rosbridge-rosbag-player
RUN source /opt/ros/${ROS1_DISTRO}/setup.bash \
  && git clone https://github.com/TakedaLab/rosbridge-rosbag-player.git /opt/rosbridge-rosbag-player \
  && cd /opt/rosbridge-rosbag-player/src \
  && git checkout v1.0.1 \
  && catkin_init_workspace \
  && cd .. \
  && apt update \
  && rosdep update \
  && rosdep install --from-paths src --ignore-src -r -y \
  && catkin_make \
  && apt -y clean \
  && rm -rf /var/lib/apt/lists/*

# Install ros2-nodes
RUN mkdir -p /opt/ros2_nodes/src
COPY ./src /opt/ros2_nodes/src
RUN source /opt/ros/${ROS2_DISTRO}/setup.bash \
  && cd /opt/ros2_nodes \
  && for dir in src/*; do [[ -f ${dir}/setup.py ]] && pip3 install --no-cache-dir -e ${dir} || :; done \
  && rosdep install --from-paths src --ignore-src -r -y \
  && colcon build --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  && rm -rf /var/lib/apt/lists/*

# Copy files
RUN mkdir -p /opt/ros2_nodes/utils || :
COPY ./utils/* /opt/ros2_nodes/utils

# Set default command
ENTRYPOINT ["/opt/ros2_nodes/utils/entrypoint.sh"]
CMD ["/opt/ros2_nodes/utils/run.sh"]

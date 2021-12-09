FROM ros:noetic
MAINTAINER Daiki Hayashi <hayashi.daiki@hdwlab.co.jp>
ENV LANG="en_US.UTF-8"
SHELL ["/bin/bash", "-c"]

ENV ROS1_DISTRO=noetic

# Apt update and rosdep update
RUN apt update \
    && apt install -y --no-install-recommends \
      curl \
      wget \
      vim \
      git \
      jq \
      python3-pip \
    && apt upgrade -y \
    && rosdep update \
    && apt clean -y \
    && rm -rf /var/lib/apt/lists/*

# Install ROS1-related packages
RUN apt update \
    && apt install -y --no-install-recommends \
      ros-${ROS1_DISTRO}-*-msgs \
    && apt clean -y \
    && rm -rf /var/lib/apt/lists/*

# Install rosbridge
ARG ROSBRIDGE_VERSION=0.11.13
RUN source /opt/ros/${ROS1_DISTRO}/setup.bash \
  && mkdir -p /opt/catkin_workspace \
  && cd /opt/catkin_workspace \
  && git clone --depth=1 -b ${ROSBRIDGE_VERSION} https://github.com/RobotWebTools/rosbridge_suite.git src \
  && cd src \
  && catkin_init_workspace \
  && cd .. \
  && apt update \
  && rosdep update \
  && rosdep install --from-paths src --ignore-src -r -y \
  && catkin_make \
  && apt -y clean \
  && rm -rf /var/lib/apt/lists/*

# Install rosbridge-rosbag-player
RUN source /opt/ros/${ROS1_DISTRO}/setup.bash \
  && source /opt/catkin_workspace/devel/setup.bash \
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

# Install ros1-nodes
RUN mkdir -p /opt/ros1_nodes/src
COPY ./src /opt/ros1_nodes/src
COPY ./test /opt/ros1_nodes/test
RUN source /opt/ros/${ROS1_DISTRO}/setup.bash \
  && source /opt/catkin_workspace/devel/setup.bash \
  && source /opt/rosbridge-rosbag-player/devel/setup.bash \
  && cd /opt/ros1_nodes/src \
  && catkin_init_workspace \
  && cd .. \
  && apt update \
  && rosdep update \
  && rosdep install --from-paths src --ignore-src -r -y \
  && for file in src/*/requirements.txt; do pip3 install -r ${file}; done \
  && catkin_make \
  && apt -y clean \
  && rm -rf /var/lib/apt/lists/*

# Copy files
RUN mkdir -p /opt/ros1_nodes/utils || :
COPY ./utils/* /opt/ros1_nodes/utils

# Set default command
WORKDIR /opt/ros1_nodes/utils
ENTRYPOINT ["/opt/ros1_nodes/utils/entrypoint.sh"]
CMD ["/opt/ros1_nodes/utils/run.sh"]

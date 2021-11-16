FROM hdwlab/rosbridge-rosbag-player:autoware-v1.0.0
MAINTAINER Daiki Hayashi <hayashi.daiki@hdwlab.co.jp>
ENV LANG="en_US.UTF-8"
SHELL ["/bin/bash", "-c"]

# Apt update and rosdep update
RUN apt update \
    && rosdep update \
    && apt clean -y \
    && rm -rf /var/lib/apt/lists/*

# Install ros-nodes
RUN mkdir -p /opt/ros_nodes/src
COPY ./src /opt/ros_nodes/src
COPY ./test /opt/ros_nodes/test
RUN bash -c '\
  source /root/.bashrc \
  && source /opt/path.sh \
  && cd /opt/ros_nodes/src \
  && catkin_init_workspace \
  && cd .. \
  && rosdep install --from-paths src --ignore-src -r -y \
  && catkin_make \
  && echo "source /opt/ros_nodes/devel/setup.bash" >> /opt/path.sh \
'

# Copy files
RUN mkdir -p /opt/utils || :
COPY ./utils/* /opt/utils

# Set default command
ENTRYPOINT ["/opt/utils/entrypoint.sh"]
CMD ["/opt/utils/run.sh"]

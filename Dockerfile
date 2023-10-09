# ROS distribution to use
ARG ROS_DISTRO=humble

########################################
# Base Image #
########################################
FROM osrf/ros:${ROS_DISTRO}-desktop as base

ENV ROS_DISTRO=${ROS_DISTRO}

ARG WS_NAME
ENV WS_NAME=${WS_NAME}

SHELL ["/bin/bash", "-c"]

# Install basic apt packages
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    git \
    python3-rosdep \
    xterm \
    libcanberra-gtk-module libcanberra-gtk3-module fuse3 libfuse2 libqt5svg5-dev \
    python3-pip python3-opencv python3-tk python3-pyqt5.qtwebengine

# Install additional Python modules
RUN pip3 install matplotlib transforms3d

# Use Cyclone DDS as middleware
RUN apt-get update && apt-get install -y --no-install-recommends \
 ros-${ROS_DISTRO}-rmw-cyclonedds-cpp
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

RUN rosdep update

# Create Colcon workspace with external dependencies
RUN mkdir -p /${WS_NAME}/src
WORKDIR /${WS_NAME}/src
#COPY dependencies.repos .
#RUN vcs import < dependencies.repos
RUN git clone --recursive https://github.com/evan-brooks/champ -b ros2
RUN git clone https://github.com/chvmp/champ_teleop -b ros2

# Build the base Colcon workspace, installing dependencies first.
WORKDIR /${WS_NAME}
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && apt-get update -y \
 && rosdep install --from-paths src --ignore-src -r --rosdistro ${ROS_DISTRO} -y \
 && colcon build --symlink-install

# Remove display warnings
RUN mkdir /tmp/runtime-root
ENV XDG_RUNTIME_DIR "/tmp/runtime-root"
RUN chmod -R 0700 /tmp/runtime-root
ENV NO_AT_BRIDGE 1

# Set up the entrypoint
WORKDIR /${WS_NAME}
COPY entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]

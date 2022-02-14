# This is an auto generated Dockerfile for ros:ros-base
# generated from docker_images_ros2/create_ros_image.Dockerfile.em
FROM ros:foxy-ros-core-focal
ARG SIM_WORLD

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    git \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    python3-pip \
    ros-foxy-gazebo-* \
    ros-foxy-turtlebot3-msgs \
    ros-foxy-turtlebot3 \
    ros-foxy-turtlebot3-gazebo \
    && rm -rf /var/lib/apt/lists/*
    
# Install dependencies
RUN pip3 install -U setuptools \
    && pip3 install git+https://github.com/microsoft/bonsai-common \
    && pip3 uninstall -y setuptools
    
RUN pip3 install scipy

# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO

# setup colcon mixin and metadata
RUN colcon mixin add default \
      https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default \
      https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update

# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-foxy-ros-base=0.9.2-1* \
    && rm -rf /var/lib/apt/lists/*

# setup workspace and install package
RUN mkdir -p /ros2_ws/src

WORKDIR /ros2_ws

ENV TURTLEBOT3_MODEL burger
ENV SIM_WORLD ${SIM_WORLD}

COPY ./turtlebot3_bonsai /ros2_ws/src/turtlebot3_bonsai
COPY ./sample_worlds /ros2_ws/src/sample_worlds

RUN . /opt/ros/foxy/setup.sh && colcon build

# command to run the simulation
CMD bash src/turtlebot3_bonsai/launch/launch.sh




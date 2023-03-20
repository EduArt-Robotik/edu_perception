FROM ros:galactic-ros-core-focal

# using bash
SHELL ["/bin/bash", "-c"]
ENV ROS_DISTRO galactic

RUN \
    apt-get update \
    && apt-get install -y software-properties-common

# Install Dependencies for Perception Package
RUN apt-get update \
    && apt-get install -y \
    ros-$ROS_DISTRO-geometry-msgs \
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-rmw-fastrtps-cpp \
    ros-$ROS_DISTRO-fastrtps \
    libeigen3-dev \
    libzbar-dev \
    python3-colcon-common-extensions \
    python3-colcon-ros \
    make \
    cmake \
    build-essential \
    libusb-1.0-0-dev

# set up iotbot workspace
ENV EDU_WORKSPACE perception_ws
RUN mkdir -p /home/$EDU_WORKSPACE/src

COPY ./edu_perception /home/$EDU_WORKSPACE/src/edu_perception
COPY ./fastrtps_profile.xml /home/$EDU_WORKSPACE

WORKDIR /home/$EDU_WORKSPACE

# build EduArt packages nodes
RUN export RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    && source /opt/ros/$ROS_DISTRO/setup.bash \
    && MAKEFLAGS="-j2" colcon build --symlink-install --executor sequential --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release

# write source commands to .bashrc -> no need to source afterwards
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc \
    && echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc \
    && echo "export FASTRTPS_DEFAULT_PROFILES_FILE=/home/$EDU_WORKSPACE/fastrtps_profile.xml" >> ~/.bashrc \
    && echo "source /home/$EDU_WORKSPACE/install/setup.bash" >> ~/.bashrc

# launch control node and joy interpreter
CMD export RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    && export FASTRTPS_DEFAULT_PROFILES_FILE=/home/$EDU_WORKSPACE/fastrtps_profile.xml \
    && source /home/$EDU_WORKSPACE/install/setup.bash \
    && ros2 launch edu_perception qr_code_detection_and_pose_estimation.launch.py

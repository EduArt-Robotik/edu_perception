FROM eduartrobotik/eduart-ros-base:humble-1.0.0

# using bash
SHELL ["/bin/bash", "-c"]
ENV ROS_DISTRO humble
ENV USER user
ENV DEBIAN_FRONTEND=noninteractive

USER root

# Install Dependencies for Perception Package
RUN apt-get update \
    && apt-get install -y \
    ros-$ROS_DISTRO-geometry-msgs \
    ros-$ROS_DISTRO-cv-bridge \
    libeigen3-dev \
    libzbar-dev \
    libusb-1.0-0-dev \
    && apt-get clean

# set up ros workspace
RUN mkdir -p /home/$USER/ros/src/
COPY ./edu_perception /home/$USER/ros/src/edu_perception
COPY ./ipc_cyclone_dds_profile.xml /home/$USER/
RUN chown -R $USER /home/$USER/ros

# change to user user
USER $USER
ENV CYCLONEDDS_URI=/home/$USER/ipc_cyclone_dds_profile.xml
WORKDIR /home/$USER/ros

# build EduArt packages nodes
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
    && source /home/$USER/.bashrc \
    && colcon build --symlink-install --executor sequential --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release

# launch control node and joy interpreter
CMD echo "launching eduard ipc127e perception software" \
    && source /home/user/ros/install/setup.bash \
    && ros2 launch edu_perception qr_code_detection_and_pose_estimation.launch.py

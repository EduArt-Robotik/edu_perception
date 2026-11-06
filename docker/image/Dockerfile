FROM eduartrobotik/eduart-robot:1.1.0

# using bash
SHELL ["/bin/bash", "-c"]
ENV ROS_DISTRO jazzy
ENV USER user
ENV DEBIAN_FRONTEND=noninteractive

USER root

# Install Dependencies for Perception Package
RUN apt-get update \
    && apt-get upgrade -y \
    && apt-get install -y \
    ros-$ROS_DISTRO-geometry-msgs \
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-aruco-opencv \
    ros-$ROS_DISTRO-aruco-opencv-msgs \
    ros-$ROS_DISTRO-apriltag-ros \
    ros-$ROS_DISTRO-zbar-ros \
    ros-$ROS_DISTRO-usb-cam \
    libzbar-dev \
    libusb-1.0-0-dev \
    python3-venv \
    && apt-get clean

# work with user user
USER $USER

# set virtual environment for RFID tag reader node
RUN python3 -m venv /home/$USER/rfid_venv \
    && source /home/$USER/rfid_venv/bin/activate \
    && pip3 install --upgrade pip \
    && pip3 install requests PyYAML jinja2 setuptools typeguard numpy


# set up ros workspace
COPY --chown=$USER ./edu_perception /home/$USER/ros/src/edu_perception
RUN cd /home/$USER/ros/src/edu_perception \
    && chown user:user -R /home/$USER/ros/src/edu_perception
    # && git submodule update --init --recursive

# build EduArt packages nodes
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
    && source /home/$USER/.bashrc \
    && colcon build --symlink-install --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release

# add entry point
COPY --chown=$USER ./entrypoint /home/$USER/
RUN chmod a+x /home/$USER/entrypoint
ENTRYPOINT [ "/home/user/entrypoint" ]

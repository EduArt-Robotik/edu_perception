FROM eduartrobotik/eduart-swarm:0.2.0

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
    libzbar-dev \
    libusb-1.0-0-dev \
    && apt-get clean

# work with user user
USER $USER

# set up ros workspace
COPY ./edu_perception /home/$USER/ros/src/edu_perception

# build EduArt packages nodes
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
    && source /home/$USER/.bashrc \
    && colcon build --symlink-install --executor sequential --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release

# add entry point
COPY --chown=$USER ./entrypoint /home/$USER/
RUN sudo chmod a+x /home/$USER/entrypoint
ENTRYPOINT [ "/home/user/entrypoint" ]

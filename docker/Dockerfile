##############
# Base image #
##############
FROM ros:noetic-perception-focal as base

ARG WS_DIR
ENV WS_DIR=${WS_DIR}
ENV QT_DEBUG_PLUGINS=1
# equal to mkdir -> cd to it
WORKDIR ${WS_DIR}

SHELL ["/bin/bash", "-c"]

# Needed for some packages installation
ENV DEBIAN_FRONTEND=noninteractive
RUN echo "source /opt/ros/noetic/setup.bash" >> /etc/bash.bashrc


# Add additional installation instructions here...


#####################
# Development image #
#####################

FROM base as dev

ARG ROS_DISTRO=noetic
ARG USERNAME=${USERNAME}
ARG UID=${UID}
ARG GID=${GID}

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
&&  apt-get install -y \
    iperf3 \
    iputils-ping \
    net-tools \
    python3-vcstool \
&& rm -rf /var/lib/apt/lists/*

# rViz and rqt are used for visualization
RUN apt-get update \
 && apt-get install -y \
    ros-${ROS_DISTRO}-rviz \
 && apt-get install ros-${ROS_DISTRO}-rqt -y\
 && apt-get install ros-${ROS_DISTRO}-rqt-common-plugins -y \
 && rm -rf /var/lib/apt/lists/*

## Required for visualization from docker to host machine
RUN apt-get update && \
apt-get install -y x11-apps \
&& apt-get install -y libcanberra-gtk* \
&& apt-get -y install libglfw3-dev \
&& apt-get -y install libglew-dev \
&& apt-get -y install mesa-utils 

RUN apt-get update && apt-get install ros-noetic-turtlesim

RUN apt-get update && apt-get install git -y
RUN apt-get update && apt install python3-catkin-tools -y

RUN apt-get update && apt-get install ros-noetic-joint-state-publisher
RUN apt-get update && apt-get install ros-noetic-robot-state-publisher -y
RUN apt-get update && apt-get install ros-noetic-xacro
RUN apt-get update && apt-get install ros-noetic-ros-controllers -y
 # Install additional developer tools here...
ENV DEBIAN_FRONTEND=dialog

RUN addgroup --gid ${GID} ${USERNAME} \
 && adduser --disabled-password --gecos '' --uid ${GID} --gid ${GID} ${USERNAME} \
 && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
 && chown -R ${UID}:${GID} /home/${USERNAME} \
 && chown -R ${UID}:${GID} ${WS_DIR}

USER ${USERNAME}
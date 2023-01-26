#
# Installs jetson ROS2 container for jetpack 35.1
#
# Alternative base images/packages: 
#
# Package: 	ros_base
# Image: 	  dustynv/ros:foxy-ros-base-l4t-r35.1.0
#
# Package: 	desktop
# Image: 	  dustynv/ros:foxy-desktop-l4t-r35.1.0
#

ARG BASE_IMAGE=dustynv/ros:foxy-desktop-l4t-r35.1.0  
ARG BASE_PACKAGE=desktop

FROM ${BASE_IMAGE} 

ENV ROS_DISTRO=foxy
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}/install
ENV ROS_WORKSPACE=ros2bot_ws
ENV DEBIAN_FRONTEND=noninteractive

#
# install missing GPG keys
#

RUN sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 42D5A192B819C5DA

#
# create a non-root user
#

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG HOME /home/${USER}

RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  # [optional] add sudo support for the non-root user
  && apt-get update \
  && apt-get install -y sudo git-core bash-completion \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  # cleanup
  && rm -rf /var/lib/apt/lists/* 

#
# switch from root to ros user
#

USER ${USERNAME}

#
# install development packages
#

RUN apt-get update && apt-get install -y --no-install-recommends \
  python3-argcomplete \
  && pyserial \
  && rm -rf /var/lib/apt/lists/* 

#
# create workspace and build ros packages
#

WORKDIR ${HOME}/${ROS_WORKSPACE}
RUN mkdir src && mkdir config \
  && COPY ./config/upstream.repos config/ \
  && vcs import src < config/upstream.repos \
  && . ${ROS_ROOT}/setup.sh \
  && rosdep update && apt-get update \
  && rosdep install -q -y \
      --from-paths src \
      --ignore-src \
      --rosdistro ${ROS_DISTRO} \
  && rm -rf /var/lib/apt/lists/* \
  && colcon build --symlink-install \
  && rm -rf build log

#
# install expansion board driver packages
#

RUN mkdir -p ${HOME}/board_drivers/dist
COPY ./libraries/board_drivers/dist/*.whl ${HOME}/board_drivers/dist/
RUN pip3 install ${HOME}/board_drivers/dist/*.whl

# 
# setup entrypoint
#

COPY ./scripts/ros_entrypoint.sh /ros_entrypoint.sh

ENV DEBIAN_FRONTEND=

RUN echo "BASE IMAGE:   ${BASE_IMAGE}"
RUN echo "BASE PACKAGE: ${BASE_PACKAGE}"
RUN echo "ROS DISTRO:   ${ROS_DISTRO}"

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
WORKDIR /
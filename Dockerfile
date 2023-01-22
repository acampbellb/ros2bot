#
# Installs jetson ROS2 container for jetpack 35.1
#

ARG BASE_IMAGE=dustynv/ros:foxy-ros-base-l4t-r35.1.0
ARG ROS_PKG=ros_base

FROM ${BASE_IMAGE}

ENV ROS_DISTRO=foxy
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}
ENV DEBIAN_FRONTEND=noninteractive

#
# create a non-root user
#

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  # [optional] add sudo support for the non-root user
  && apt-get update \
  && apt-get install -y sudo git-core bash-completion \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  # cleanup
  && rm -rf /var/lib/apt/lists/* \
  && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc

#
# create workspace
#

ENV USER ros
USER ros
ENV HOME /home/${USER}
RUN mkdir -p ${HOME}/ros_ws/src
WORKDIR ${HOME}/ros_ws
RUN echo 'source ${ROS_ROOT}/install/setup.bash'
RUN echo 'colcon build --symlink-install'

# 
# setup entrypoint
#

ENV DEBIAN_FRONTEND=

COPY ./scripts/ros_entrypoint.sh /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
WORKDIR /
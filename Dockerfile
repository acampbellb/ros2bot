#
# ros2bot DEVELOP image
#

FROM acampbellb/ros2bot:foxy-desktop-l4t-ros2bot-base

ENV DEBIAN_FRONTEND=noninteractive

#
# install dev tools
#

RUN apt-get update && apt-get install -y --no-install-recommends \
  bash-completion \
  build-essential \
  cmake \
  gdb \
  git \
  openssh-client \
  python3-argcomplete \
  python3-pip \
  ros-dev-tools \
  vim \
  && rm -rf /var/lib/apt/lists/* \
  && rosdep init || echo "rosdep already initialized"

#
# create non-root ros user
# 

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID  

RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  # [optional] add sudo support for non-root user
  && apt-get update \
  && apt-get install -y sudo git-core bash-completion \
  && echo $USERNAME ALL=\(ALL\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && chown root:root /etc/sudoers.d/$USERNAME \
  # cleanup
  && rm -rf /var/lib/apt/lists/* \
  && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc

#
# switch from root to ros user
#

ENV USER ${ROS_USER}
USER ${USER}
ENV HOME /home/${USER}

#
# setup entrypoint script
#

COPY --chown=${ROS_USER} --chmod=755 ./scripts/ros_entrypoint.sh /ros_entrypoint.sh
RUN bash -c "chmod +x /ros_entrypoint.sh"

#
# finish up
#

ENV DEBIAN_FRONTEND=
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
WORKDIR /


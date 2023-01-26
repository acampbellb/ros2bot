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

FROM dustynv/ros:foxy-desktop-l4t-r35.1.0

ENV ROS_DISTRO=foxy
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}/install
ENV ROS_WORKSPACE=ros2bot_ws
ENV DEBIAN_FRONTEND=noninteractive

#
# install missing GPG keys
#

RUN sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 42D5A192B819C5DA

#
# install vcstool
#

RUN curl -s https://packagecloud.io/install/repositories/dirk-thomas/vcstool/script.deb.sh | sudo bash \
  && sudo apt-get update \
  && sudo apt-get install python3-vcstool \
  && rm -rf /var/lib/apt/lists/* 

#
# create a non-root user
#

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG HOME /home/${USERNAME}

RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  # add sudo support for the non-root user
  && sudo apt-get update \ 
  && sudo apt-get install -y --no-install-recommends \
    sudo \ 
    git-core \
    bash-completion \
    python3-argcomplete \
    python3-autopep8 \
    pylint \    
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && rm -rf /var/lib/apt/lists/* 

#
# install development packages
#

RUN mkdir -p ${HOME}/drivers/dist
COPY ./drivers/dist/*.whl ${HOME}/drivers/dist/
RUN pip3 install \
  pyserial
#  ${HOME}/drivers/dist/rosbotmaster-0.0.1-py3-none-any.whl \
#  ${HOME}/drivers/dist/rosbotspeach-0.0.1-py3-none-any.whl  

#
# switch from root to ros user
#

USER ${USERNAME}

#
# create workspace and build ros packages
#

WORKDIR ${HOME}/${ROS_WORKSPACE}
RUN mkdir -p ${HOME}/${ROS_WORKSPACE}/config
COPY ./config/upstream.repos ${HOME}/${ROS_WORKSPACE}/config/
RUN mkdir -p ${HOME}/${ROS_WORKSPACE}/src \ 
  && vcs import ${HOME}/${ROS_WORKSPACE}/src < config/upstream.repos \
  && . ${ROS_ROOT}/setup.sh \
  && rosdep update \
  && rosdep install -q -y \
      --from-paths ${HOME}/${ROS_WORKSPACE}/src \
      --ignore-src \
      --rosdistro ${ROS_DISTRO} \
  && rm -rf /var/lib/apt/lists/* \
  && cd ${HOME}/${ROS_WORKSPACE} \
  && colcon build --symlink-install \
  && . ${HOME}/${ROS_WORKSPACE}/install/local_setup.sh

# 
# setup entrypoint
#

RUN mkdir ${HOME}/${ROS_WORKSPACE}/scripts
COPY ./scripts/ros_entrypoint.sh ${HOME}/${ROS_WORKSPACE}/scripts/ros_entrypoint.sh

ENV DEBIAN_FRONTEND=

RUN echo "BASE IMAGE:   ${BASE_IMAGE}"
RUN echo "BASE PACKAGE: ${BASE_PACKAGE}"
RUN echo "ROS DISTRO:   ${ROS_DISTRO}"

ENTRYPOINT ["${HOME}/${ROS_WORKSPACE}/scripts/ros_entrypoint.sh"]
CMD ["bash"]
WORKDIR /
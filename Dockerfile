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
  && rm -rf /var/lib/apt/lists/* \
  && echo "if [ -f /opt/ros/${ROS_DISTRO}/install/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/install/setup.bash; fi" >> /home/$USERNAME/.bashrc

#
# install development packages
#

RUN pip3 install \
  pyserial

#
# setup entrypoint script
#

COPY ./scripts/ros_entrypoint.sh /ros_entrypoint.sh
RUN bash -c "chmod +x /ros_entrypoint.sh"  

#
# switch from root to ros user
#

ENV USER ${USERNAME}
USER ${USER}
ENV HOME /home/${USER}

#
# make libraries and config directories
#

RUN mkdir -p ${HOME}/libraries/dist \
  && mkdir -p ${HOME}/${ROS_WORKSPACE}/config
COPY ./config/upstream.repos ${HOME}/${ROS_WORKSPACE}/config/
COPY ./libraries/dist/*.whl ${HOME}/libraries/dist/
RUN pip3 install ${HOME}/libraries/dist/*.whl

#
# create workspace and build ros packages
#

WORKDIR ${HOME}/${ROS_WORKSPACE}
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
  && . ${HOME}/${ROS_WORKSPACE}/install/local_setup.sh \
  && echo "if [ -f ${HOME}/${ROS_WORKSPACE}/install/setup.bash ]; then source ${HOME}/${ROS_WORKSPACE}/install/setup.bash; fi" >> /home/$USERNAME/.bashrc

# 
# setup entrypoint
#

ENV DEBIAN_FRONTEND=

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
WORKDIR /
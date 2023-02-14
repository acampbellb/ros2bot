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
# install python development packages
#

RUN pip3 install --upgrade pip
RUN pip3 install cython
RUN pip3 install numpy
RUN pip3 install opencv-python
RUN pip3 install pyopengl
RUN pip3 install pyserial

#
# install zed sdk version 3.8.2
#  

ARG L4T_MAJOR_VERSION=35
ARG L4T_MINOR_VERSION=1
ARG L4T_PATCH_VERSION=0
ARG ZED_SDK_MAJOR=3
ARG ZED_SDK_MINOR=8

# this environment variable is needed to use the streaming features on Jetson inside a container
ENV LOGNAME root

ENV LOGNAME root
RUN apt-get update || true
RUN apt-get install --no-install-recommends lsb-release wget less udev sudo apt-transport-https build-essential cmake -y&& \
    echo "# R${L4T_MAJOR_VERSION} (release), REVISION: ${L4T_MINOR_VERSION}.${L4T_PATCH_VERSION}" > /etc/nv_tegra_release ; \
    wget -q --no-check-certificate -O ZED_SDK_Linux.run https://download.stereolabs.com/zedsdk/${ZED_SDK_MAJOR}.${ZED_SDK_MINOR}/l4t${L4T_MAJOR_VERSION}.${L4T_MINOR_VERSION}/jetsons && \
    chmod +x ZED_SDK_Linux.run ; ./ZED_SDK_Linux.run silent skip_tools && \
    rm -rf /usr/local/zed/resources/* \
    rm -rf ZED_SDK_Linux.run && \
    rm -rf /var/lib/apt/lists/*

# this symbolic link is needed to use the streaming features on Jetson inside a container
RUN ln -sf /usr/lib/aarch64-linux-gnu/tegra/libv4l2.so.0 /usr/lib/aarch64-linux-gnu/libv4l2.so

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
  && ${HOME}/${ROS_WORKSPACE}/src \ 
  && vcs import ${HOME}/${ROS_WORKSPACE}/src < config/upstream.repos \
  && git clone --recursive https://github.com/stereolabs/zed-ros2-wrapper.git \
  && git clone https://github.com/stereolabs/zed-ros2-examples.git \
  && cd .. \
  && . ${ROS_ROOT}/setup.sh \
  && rosdep update \
  && rosdep install -q -r -y \
      --from-paths ${HOME}/${ROS_WORKSPACE}/src \
      --ignore-src \
      --rosdistro ${ROS_DISTRO} \
  && rm -rf /var/lib/apt/lists/* \
  && cd ${HOME}/${ROS_WORKSPACE} \
  && colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release \
  && . ${HOME}/${ROS_WORKSPACE}/install/local_setup.sh \
  && echo "if [ -f ${HOME}/${ROS_WORKSPACE}/install/setup.bash ]; then source ${HOME}/${ROS_WORKSPACE}/install/setup.bash; fi" >> /home/$USERNAME/.bashrc

# 
# setup entrypoint
#

ENV DEBIAN_FRONTEND=

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
WORKDIR /
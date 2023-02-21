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
# install zed sdk version 3.8.2
#  

ARG L4T_MAJOR_VERSION=35
ARG L4T_MINOR_VERSION=1
ARG ZED_SDK_MAJOR=3
ARG ZED_SDK_MINOR=8

# this environment variable is needed to use the streaming features on Jetson inside a container
ENV LOGNAME root
RUN sudo apt-get update || true
RUN sudo apt-get install --no-install-recommends lsb-release wget less udev sudo apt-transport-https build-essential cmake -y && \
    echo "# R${L4T_MAJOR_VERSION} (release), REVISION: ${L4T_MINOR_VERSION}.${L4T_PATCH_VERSION}" > /etc/nv_tegra_release ; \
    wget -q --no-check-certificate -O ZED_SDK_Linux.run https://download.stereolabs.com/zedsdk/${ZED_SDK_MAJOR}.${ZED_SDK_MINOR}/l4t${L4T_MAJOR_VERSION}.${L4T_MINOR_VERSION}/jetsons && \
    sudo chmod +x ZED_SDK_Linux.run ; ./ZED_SDK_Linux.run silent skip_tools && \
    sudo rm -rf /usr/local/zed/resources/* \
    sudo rm -rf ZED_SDK_Linux.run && \
    sudo rm -rf /var/lib/apt/lists/*

# this symbolic link is needed to use the streaming features on Jetson inside a container
RUN ln -sf /usr/lib/aarch64-linux-gnu/tegra/libv4l2.so.0 /usr/lib/aarch64-linux-gnu/libv4l2.so

#
# switch from root to ros user
#

ENV USER ${USERNAME}
USER ${USER}
ENV HOME /home/${USER}

# zed python api
# RUN apt-get update -y || true
# RUN apt-get install --no-install-recommends python3 python3-pip python3-dev python3-setuptools build-essential -y && \ 
#     wget -q download.stereolabs.com/zedsdk/pyzed -O /usr/local/zed/get_python_api.py && \
#     python3 /usr/local/zed/get_python_api.py && \
#     python3 -m pip install cython wheel pyserial && \
#     python3 -m pip install flask gevent pyzbar RPi.GPIO && \    
#     python3 -m pip install opencv-python numpy==1.24.2 pyopengl *.whl && \
#     python3 -m pip install ${HOME}/ros2bot/libs/*.whl && \
#     apt-get remove --purge build-essential -y && apt-get autoremove -y && \
#     rm *.whl ; rm -rf /var/lib/apt/lists/*  

#
# create app / libraries directories
#

ENV ROS_WORKSPACE=ros2bot_ws

WORKDIR ${HOME}

RUN mkdir -p ./ros2bot/libs \
  && mkdir -p ./ros2bot/app \
  && mkdir -p ./${ROS_WORKSPACE}/src

COPY --chown=${USER} --chmod=755 ./libs/dist/*.whl ./ros2bot/libs/
COPY --chown=${USER} --chmod=755 ./app ./ros2bot/app/ 

#
# create workspace and build packages
#

WORKDIR ${HOME}/${ROS_WORKSPACE}/src

RUN sudo git clone https://github.com/acampbellb/ros2bot_packages.git . \
  && sudo git clone https://github.com/Slamtec/sllidar_ros2.git \
  && sudo git clone https://github.com/ros-perception/image_common.git \
  && sudo git clone --recursive https://github.com/stereolabs/zed-ros2-wrapper.git \
  && sudo git clone https://github.com/stereolabs/zed-ros2-examples.git \
  && . ${ROS_ROOT}/setup.sh 

  # && sudo rosdep update \
  # && sudo rosdep install -q -r -y \
  #   --from-paths . \
  #   --ignore-src \
  #   --rosdistro ${ROS_DISTRO} \
  # && cd ${HOME}/${ROS_WORKSPACE} \
  # && colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release \
  # && . ${HOME}/${ROS_WORKSPACE}/install/local_setup.sh \
  # && echo "if [ -f ${HOME}/${ROS_WORKSPACE}/install/setup.bash ]; then source ${HOME}/${ROS_WORKSPACE}/install/setup.bash; fi" >> /root/.bashrc


#
# setup entrypoint script
#

COPY --chown=${USER} --chmod=755 ./scripts/ros_entrypoint.sh /ros_entrypoint.sh
RUN bash -c "chmod +x /ros_entrypoint.sh"

#
# finish up
#

ENV DEBIAN_FRONTEND=
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
WORKDIR /


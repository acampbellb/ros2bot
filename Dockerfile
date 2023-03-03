#
# Builds container image with base containing Jetpack 35.1 and ROS2 foxy desktop, 
# and workspace containing rplidar, zed camera, and ros2bot packages.
#

ARG IMAGE_NAME=dustynv/ros:foxy-desktop-l4t-r35.1.0

FROM ${IMAGE_NAME}

ARG JETPACK_MAJOR=5
ARG JETPACK_MINOR=0
ARG L4T_MAJOR=35
ARG L4T_MINOR=1

ENV ROS_DISTRO=foxy
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}
ENV ROS_PYTHON_VERSION=3
ENV DEBIAN_FRONTEND=noninteractive

# 
# disable apt-get warnings
#

RUN apt-get update || true && apt-get install -y --no-install-recommends apt-utils dialog && \
  rm -rf /var/lib/apt/lists/*

#
# define timezone
#

ENV TZ=America/New_York

#
# install development packages
#

RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone && \ 
  apt-get update \
  && apt-get install --yes lsb-release \ 
    wget \
    less \ 
    udev \
    sudo \
    git \ 
    build-essential \
    cmake \ 
    gdb \
    openssh-client \
    python3-dev \
    python3-pip \
    python3-wheel \
    python3-numpy \
    python3-setuptools \
    python3-argcomplete \
    python3-autopep8 \
    pylint \
    protobuf-compiler \
    jq \
    libpq-dev \
    zstd \
    usbutils \    
  && rm -rf /var/lib/apt/lists/* \
  && apt-get clean

#
# create non-root ros user
# 

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID  

RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  # add sudo support for non-root user
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

ENV USER ${USERNAME}
USER ${USER}
ENV HOME /home/${USER}

#
# install zed sdk 
# 

ARG ZED_SDK_MAJOR=3
ARG ZED_SDK_MINOR=8

RUN echo "# R${L4T_MAJOR} (release), REVISION: ${L4T_MINOR}" > /etc/nv_tegra_release && \
  apt-get update -y || true && \
  apt-get install -y --no-install-recommends zstd wget less cmake curl gnupg2 \
  build-essential python3 python3-pip python3-dev python3-setuptools libusb-1.0-0-dev -y && \
  pip install protobuf && \
  wget -q --no-check-certificate -O ZED_SDK_Linux_JP.run \
  https://download.stereolabs.com/zedsdk/${ZED_SDK_MAJOR}.${ZED_SDK_MINOR}/l4t${L4T_MAJOR}.${L4T_MINOR}/jetsons && \
  chmod +x ZED_SDK_Linux_JP.run ; ./ZED_SDK_Linux_JP.run silent skip_tools && \
  rm -rf /usr/local/zed/resources/* && \
  rm -rf ZED_SDK_Linux_JP.run && \
  rm -rf /var/lib/apt/lists/*

#
# install app & libs
#

ENV ROS_WORKSPACE=ros2bot_ws

WORKDIR ${HOME}
RUN mkdir -p ./ros2bot/libs \
  && mkdir -p ./ros2bot/app \
  && mkdir -p ./${ROS_WORKSPACE}/src

COPY --chown=${USER} --chmod=755 ./libs/dist/*.whl ./ros2bot/libs/
COPY --chown=${USER} --chmod=755 ./app ./ros2bot/app/ 

RUN python3 -m pip install pyserial flask gevent pyzbar RPi.GPIO && \
  python3 -m pip install ${HOME}/ros2bot/libs/*.whl

#
# create workspace and build packages
#

# zed ros2 wrapper dependency versions
ARG XACRO_VERSION=2.0.8
ARG DIAGNOSTICS_VERSION=3.0.0
ARG AMENT_LINT_VERSION=0.12.4

WORKDIR ${HOME}/${ROS_WORKSPACE}/src
RUN sudo git clone https://github.com/acampbellb/ros2bot_packages.git \
  && sudo git clone https://github.com/Slamtec/sllidar_ros2.git \
  && sudo git clone https://github.com/ros-perception/image_common.git --branch 3.0.0 --single-branch \
  && sudo git clone --recursive https://github.com/stereolabs/zed-ros2-wrapper.git \
  && sudo git clone https://github.com/stereolabs/zed-ros2-examples.git \
  && wget https://github.com/ros/xacro/archive/refs/tags/${XACRO_VERSION}.tar.gz -O - | tar -xvz && mv xacro-${XACRO_VERSION} xacro \
  && wget https://github.com/ros/diagnostics/archive/refs/tags/${DIAGNOSTICS_VERSION}.tar.gz -O - | tar -xvz && mv diagnostics-${DIAGNOSTICS_VERSION} diagnostics \
  && wget https://github.com/ament/ament_lint/archive/refs/tags/${AMENT_LINT_VERSION}.tar.gz -O - | tar -xvz && mv ament_lint-${AMENT_LINT_VERSION} ament-lint 

#
# install dependencies
#

WORKDIR ${HOME}/${ROS_WORKSPACE}
RUN sudo apt-get update -y || true \
  && sudo -H apt-get install -y ros-foxy-ament-cmake-clang-format \
  && sudo rosdep fix-permissions \
  && rosdep update \
  && rosdep install --from-paths src --ignore-src -r -y \
  && sudo rm -rf /var/lib/apt/lists/*

#
# build workspace packages
#

WORKDIR ${HOME}/${ROS_WORKSPACE}
RUN . ${ROS_ROOT}/setup.sh \
  && colcon build --parallel-workers $(nproc) --symlink-install \
    --event-handlers console_direct+ --base-paths src --rosdistro ${ROS_DISTRO} \
    --cmake-args ' -DCMAKE_BUILD_TYPE=Release' \
    ' -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs' \
    ' -DCMAKE_CXX_FLAGS="-Wl,--allow-shlib-undefined"' \
    ' --no-warn-unused-cli' \
  && . ${HOME}/${ROS_WORKSPACE}/install/local_setup.sh \
  && echo "if [ -f ${HOME}/${ROS_WORKSPACE}/install/setup.bash ]; then source ${HOME}/${ROS_WORKSPACE}/install/setup.bash; fi" >> /root/.bashrc 

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
#
# Installs jetson ROS2 container for jetpack 35.1
#

#
# BASE image - Jetpack v35.2.1 w/ ROS2
#

ARG BASE_IMAGE=nvcr.io/nvidia/l4t-jetpack:r35.2.1
FROM ${BASE_IMAGE} AS base

ARG ROS_PKG=desktop
ENV ROS_DISTRO=foxy
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}
ENV ROS_PYTHON_VERSION=3

ENV DEBIAN_FRONTEND=noninteractive
ENV SHELL /bin/bash
SHELL ["/bin/bash", "-c"]

WORKDIR /tmp

# change the locale from POSIX to UTF-8
RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV PYTHONIOENCODING=utf-8

# set Python3 as default
RUN update-alternatives --install /usr/bin/python python /usr/bin/python3 1

#
# upgrade apt
#

RUN apt-get update && apt-get -y upgrade \
    && rm -rf /var/lib/apt/lists/*

# 
# add the ROS deb repo to the apt sources list
#
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
		curl \
		wget \
		gnupg2 \
		lsb-release \
		ca-certificates \
        sudo \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 
# install development packages
#
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        apt-utils \
		build-essential \
		cmake \
		git \
		libbullet-dev \
		libpython3-dev \
		python3-colcon-common-extensions \
		python3-flake8 \
		python3-pip \
		python3-numpy=1.24.2 \
		python3-rosdep \
		python3-setuptools \
		python3-vcstool \
        python3-rosinstall-generator \
		python3-argcomplete \
        python3-autopep8 \
        pylint \
		libasio-dev \
		libtinyxml2-dev \
		libcunit1-dev \
        zstd \
        ros-foxy-desktop \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

# 
# install OpenCV (with CUDA)
#

ARG OPENCV_URL=https://nvidia.box.com/shared/static/5v89u6g5rb62fpz4lh0rz531ajo2t5ef.gz
ARG OPENCV_DEB=OpenCV-4.5.0-aarch64.tar.gz

COPY scripts/opencv_install.sh /tmp/opencv_install.sh
RUN cd /tmp && ./opencv_install.sh ${OPENCV_URL} ${OPENCV_DEB}  

# 
# upgrade cmake - https://stackoverflow.com/a/56690743
# this is needed to build some of the ROS2 packages
#

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
		  software-properties-common \
		  apt-transport-https \
		  ca-certificates \
		  gnupg2 \
		  lsb-release \
    && sudo add-apt-repository universe \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

RUN pip3 install --upgrade --no-cache-dir --verbose cmake
RUN cmake --version

#
# remove other versions of Python3
# workaround for 'Could NOT find Python3 (missing: Python3_INCLUDE_DIRS Python3_LIBRARIES'
#

RUN apt purge -y python3.9 libpython3.9* || echo "python3.9 not found, skipping removal" && \
    ls -ll /usr/bin/python*

# 
# compile yaml-cpp-0.6, which some ROS packages may use (but is not in the 18.04 apt repo)
#

RUN git clone --branch yaml-cpp-0.6.0 https://github.com/jbeder/yaml-cpp yaml-cpp-0.6 && \
    cd yaml-cpp-0.6 && \
    mkdir build && \
    cd build && \
    cmake -DBUILD_SHARED_LIBS=ON .. && \
    make -j$(nproc) && \
    cp libyaml-cpp.so.0.6.0 /usr/lib/aarch64-linux-gnu/ && \
    ln -s /usr/lib/aarch64-linux-gnu/libyaml-cpp.so.0.6.0 /usr/lib/aarch64-linux-gnu/libyaml-cpp.so.0.6    

ENV AMENT_PREFIX_PATH=/opt/ros/foxy
ENV COLCON_PREFIX_PATH=/opt/ros/foxy
ENV LD_LIBRARY_PATH=/opt/ros/foxy/lib
ENV PATH=/opt/ros/foxy/bin:$PATH
ENV PYTHONPATH=/opt/ros/foxy/lib/python3.8/site-packages
ENV ROS_VERSION=2
ENV DEBIAN_FRONTEND=    

#
# DEVELOP image
#

FROM base AS develop

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
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID --gecos="ROS" --disabled-password -m $USERNAME \
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


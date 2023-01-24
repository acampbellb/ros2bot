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
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}
ENV ROS_WORKSPACE=ros2bot_ws
ENV DEBIAN_FRONTEND=noninteractive

#
# install missing GPG keys
#

RUN sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 42D5A192B819C5DA

#
# install apt-utils
#

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
		apt-utils \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

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

ENV USER ${USERNAME}
USER ${USERNAME}
ENV HOME /home/${USER}
RUN mkdir -p ${HOME}/${ROS_WORKSPACE}/src
WORKDIR ${HOME}/${ROS_WORKSPACE}
# source underlay
RUN if [ -f /opt/ros/${ROS_DISTRO}/install/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/install/setup.bash; fi
RUN echo "if [ -f /opt/ros/${ROS_DISTRO}/install/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/install/setup.bash; fi" >> ~/.bashrc
# install dependencies 
RUN rosdep update                
RUN rosdep install -i --from-path src --rosdistro ${ROS_DISTRO} -y   
# copy source code
COPY ./src/* ${HOME}/${ROS_WORKSPACE}/src/
# build workspace 
WORKDIR ${HOME}/${ROS_WORKSPACE}
RUN colcon build --symlink-install                        
# source overlay
RUN if [ -f ${HOME}/${ROS_WORKSPACE}/install/local_setup.bash ]; then source ${HOME}/${ROS_WORKSPACE}/install/local_setup.bash; fi
RUN echo "if [ -f ${HOME}/${ROS_WORKSPACE}/install/local_setup.bash ]; then source ${HOME}/${ROS_WORKSPACE}/install/local_setup.bash; fi" >> ~/.bashrc

#
# install pip package dependencies
#

RUN python3 -m pip install -U \
		serial

#
# install board driver packages
#

RUN mkdir -p ${HOME}/board_drivers/dist
COPY ./libraries/board_drivers/dist/*.whl ${HOME}/board_drivers/dist/
RUN pip3 install ${HOME}/board_drivers/dist/*.whl

# 
# setup entrypoint
#

COPY ./scripts/ros_entrypoint.sh /ros_entrypoint.sh

ENV DEBIAN_FRONTEND=

RUN echo "Base image: ${BASE_IMAGE}"
RUN echo "Base package: ${BASE_PACKAGE}"
RUN echo "ROS distro: ${ROS_DISTRO}}"

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
WORKDIR /
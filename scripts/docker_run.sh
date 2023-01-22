#!/usr/bin/env bash

#
# This script runs the docker ROS container image.
#

CONTAINER_IMAGE=""
WORKSPACE_NAME="ros_ws"
WORKSPACE_HOST="/home/ros2bot/src"
WORKSPACE_CONTAINER="/home/ros/ros_ws/src/:rw"
INITIAL_COMMAND="/bin/bash"

echo "CONTAINER_IMAGE:     $CONTAINER_IMAGE"
echo "WORKSPACE_HOST:      $WORKSPACE_HOST"
echo "WORKSPACE_CONTAINER: $WORKSPACE_CONTAINER"
echo "INITIAL_COMMAND:     '$INITIAL_COMMAND'"

# give docker root user X11 permissions
sudo xhost +si:localuser:root

# enable SSH X11 forwarding inside container (https://stackoverflow.com/q/48235040)
XAUTH=/tmp/.docker.xauth
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
chmod 777 $XAUTH

# create a shared volume to store the ros_ws
docker volume create --driver local \
    --opt type="none" \
    --opt device="${PWD}/src/" \
    --opt o="bind" \
    "${WORKSPACE_NAME}_src_vol"

# run the container image
sudo docker run \
    --runtime nvidia \
    -it \                                                           # interactive mode, pseudo tty
    --rm \                                                          # remove container when exits
    --network host \                                                # network container connects to
    -e DISPLAY=$DISPLAY \                                           # set display environment variable
    -v /tmp/.X11-unix/:/tmp/.X11-unix \                             # x11 for SSH
    -v $XAUTH:$XAUTH -e XAUTHORITY=$XAUTH \                         # x11 authorization
    -volume="${WORKSPACE_NAME}_src_vol:/home/ros/ros_ws/src/:rw" \  # ros workspace bind mount volume
    #-v $WORKSPACE_HOST:$WORKSPACE_CONTAINER \ 
    $CONTAINER_IMAGE $INITIAL_COMMAND


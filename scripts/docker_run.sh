#!/usr/bin/env bash

#
# This script runs the docker ROS container image.
#

CONTAINER_IMAGE="acampbellb/ros2bot:foxy-desktop-l4t-ros2bot"
WORKSPACE_NAME="ros2bot_ws"
INITIAL_COMMAND=" $@ "

echo "CONTAINER_IMAGE:     $CONTAINER_IMAGE"
echo "WORKSPACE_NAME:      $WORKSPACE_NAME"
echo "INITIAL_COMMAND:     '$INITIAL_COMMAND'"

# give docker root user X11 permissions
sudo xhost +si:localuser:root

# enable SSH X11 forwarding inside container (https://stackoverflow.com/q/48235040)
XAUTH=/tmp/.docker.xauth
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
chmod 777 $XAUTH

echo ${PWD}    

# run the container image
sudo docker run --runtime nvidia -it --rm --network host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix -v $XAUTH:$XAUTH -e XAUTHORITY=$XAUTH \
    $CONTAINER_IMAGE $INITIAL_COMMAND
 


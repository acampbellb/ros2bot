#!/usr/bin/env bash

#
# This script runs the docker ROS container image.
#

CONTAINER_IMAGE="acampbellb/ros2bot:foxy-ros_base-l4t-ros2bot"
WORKSPACE_NAME="ros2bot_ws"
WORKSPACE_HOST="/home/Repos/ros2bot/src"
WORKSPACE_CONTAINER="/home/ros/${WORKSPACE_NAME}/src/:rw"
INITIAL_COMMAND=" $@ "
SOURCE_VOLUME="${WORKSPACE_NAME}_src_vol"

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
    "${SOURCE_VOLUME}"

echo ${PWD}    

# run the container image
sudo docker run --runtime nvidia -it --rm --network host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix -v $XAUTH:$XAUTH -e XAUTHORITY=$XAUTH -v "${SOURCE_VOLUME}":/home/ros/$WORKSPACE_NAME/src/:rw \
    $CONTAINER_IMAGE $INITIAL_COMMAND
 


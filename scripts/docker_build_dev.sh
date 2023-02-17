#!/usr/bin/env bash

BASE_IMAGE="foxy-desktop-l4t-ros2bot-base"  
ROS_DISTRO="foxy"
ROS_PACKAGE="desktop"

# determine the L4T version
source scripts/docker_base.sh
source scripts/opencv_version.sh

echo "ROS_DISTRO:   $ROS_DISTRO"
echo "ROS_PACKAGE:  $ROS_PACKAGE"

build_ros()
{
	local distro=$1
	local package=$2
	local base_image=$3
	local extra_tag=$4
	local repository="acampbellb/ros2bot"
    local dockerfile="Dockerfile-dev"
    local container_tag="${repository}:${distro}-${extra_tag}-l4t-ros2bot-dev"

	echo ""
	echo "Building container $container_tag"
	echo "BASE_IMAGE=$base_image"
	echo ""

	sh ./scripts/docker_build.sh $container_tag $dockerfile \
			--build-arg ROS_PKG=$package \
			--build-arg BASE_IMAGE=$base_image \
			--build-arg OPENCV_URL=$OPENCV_URL \
			--build-arg OPENCV_DEB=$OPENCV_DEB
			
	# restore opencv.csv mounts
	if [ -f "$CV_CSV.backup" ]; then
		sudo mv $CV_CSV.backup $CV_CSV
	fi
}

build_ros $ROS_DISTRO $ROS_PACKAGE $BASE_IMAGE $ROS_PACKAGE



#!/usr/bin/env bash

BASE_IMAGE="nvcr.io/nvidia/l4t-base:r35.1.0"
ROS_DISTRO="foxy"
ROS_PACKAGE="ros_base"

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
    local dockerfile="Dockerfile"
    local container_tag="ros:${distro}-${extra_tag}-l4t-ros2bot"

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



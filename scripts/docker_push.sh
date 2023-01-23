#!/usr/bin/env bash

DOCKERHUB="acampbellb"
ROS_DISTRO="foxy"
ROS_PACKAGE="ros_base"

push_retag() 
{
	local registry=$1
	local src_tag=$2
	local dst_tag=$3
	
	sudo docker rmi $registry/$dst_tag
	sudo docker tag $src_tag $registry/$dst_tag
	
	echo "pushing container $src_tag => $registry/$dst_tag"
	sudo docker push $registry/$dst_tag
	echo "done pushing $registry/$dst_tag"
}

push() 
{
	push_retag $1 $2 $2
}

ros_image = "ros:$ROS_DISTRO-$ROS_PACKAGE-l4t-ros2bot"

push $DOCKERHUB $ros_image





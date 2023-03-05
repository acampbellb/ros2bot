#!/usr/bin/env bash 

build_ros()
{
	local base_image="acampbellb/ros2bot:foxy-desktop-l4t-ros2bot-base"
    local dockerfile="Dockerfile-dev"
    local container_tag="acampbellb/ros2bot:foxy-desktop-l4t-ros2bot-dev"

	echo ""
	echo "Building container $container_tag"
	echo "BASE_IMAGE=$base_image"
	echo ""

	sh ./scripts/docker_build.sh $container_tag $dockerfile \
			--build-arg BASE_IMAGE=$base_image
			
	# restore opencv.csv mounts
	if [ -f "$CV_CSV.backup" ]; then
		sudo mv $CV_CSV.backup $CV_CSV
	fi
}

build_ros $BASE_IMAGE



# ros2bot

Jetson / Jetpack based ROS2 docker container workspace with STM32 driver, ZED camera, and RPLIDAR support.

## Getting started

1. Make scripts folder executable

    ```
    $ cd /ros2bot/scripts
    $ chmod +x ./*
    ```

2. Build docker container image

    ```
    $ cd /ros2bot
    $ ./scripts/docker_build_ros.sh
    ```

3. Login to docker hub

    ```
    $ docker login
    ```

4. Push docker image to docker hub repository

    ```
    $ docker push <username>/<repository name>:foxy-<ros package>-l4t-ros2bot
    ```

    ROS package can be either 'ros_base' or 'desktop'.

5. Run docker container

    ```
    $ sudo ./scripts/docker_run.sh
    ```

## Workspace development

The container contains an ROS workspace directory that is bound to a host directory using a docker volume
bind mount within the ros2bot repository. You can develop packages on the host and build them in the container.

```
Host workspace folder: /ros2bot/src
```

```
Container workspace: /home/ros/ros2bot_ws/src
```




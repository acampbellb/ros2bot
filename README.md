# ros2bot

Jetson / Jetpack based ROS2 docker container workspace with STM32 driver, ZED camera, and RPLIDAR support.

## Getting started

1. Make scripts folder executable

    ```
    $ cd /ros2bot/scripts
    $ chmod +x ./*
    ```

2. Build base docker container image

    ```
    $ cd /ros2bot
    $ ./scripts/docker_build_base.sh
    ```

3. Login to docker hub

    ```
    $ docker login
    ```

4. Push docker image to docker hub repository

    ```
    $ docker push acampbellb/ros2bot:foxy-desktop-l4t-ros2bot-base
    ```

5. Build develop docker container image

    ```
    $ cd /ros2bot
    $ ./scripts/docker_build_dev.sh
    ```
6. Push docker image to docker hub repository

    ```
    $ docker push acampbellb/ros2bot:foxy-desktop-l4t-ros2bot-dev
    ```

7. Run docker container

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




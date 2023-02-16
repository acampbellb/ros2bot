# ros2bot speach expansion board driver

Build and verify the speach board Library

    ```
    $ cd ros2bot/libraries/ros2bot_speach_lib         
    $ python3 setup.py bdist_wheel 
    $ check-wheel-contents ./dist
    ```

Installation of the library packages can be performed by navigating to
the directory that contains the *.whl files, and executing pip3.

    ```
    $ cd ros2bot/libraries/ros2bot_speach_lib/dist
    $ pip3 install ros2bot_speach_lib*.whl
    ```
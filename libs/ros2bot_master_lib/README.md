# ros2bot master expansion board driver package

Build and verify the master board Library

    ```
    $ cd ros2bot/libraries/ros2bot_master_lib      
    $ python3 setup.py bdist_wheel 
    $ check-wheel-contents ./dist
    ```

Installation of the library packages can be performed by navigating to
the directory that contains the *.whl files, and executing pip3.

    ```
    $ cd ros2bot/libraries/ros2bot_master_lib/dist
    $ pip3 install ros2bot_master_lib*.whl
    ```

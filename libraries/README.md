# ros2bot drivers

There are two python libraries that control the ros2bot expansion boards, the master board, 
and the speach board. These python libraries are packaged in python wheels.

1.  Install curl

    ```
    $ sudo apt update && sudo apt upgrade
    $ sudo apt install curl
    $ curl --version
    $ apt-cache search libcurl | grep python
    $ sudo apt install python3-pycurl
    $ apt-cache search libcurl
    ```

2.  Install pip for python packages

    ```
    $ curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py
    $ python3 get-pip.py
    ```

3.  Setup to build the wheel packages

    ```
    $ pip3 install wheel
    $ pip3 install check-wheel-contents
    ```

4.  Build and verify the master board Library

    ```
    $ cd ros2bot/libraries/ros2bot_master_lib      
    $ python3 setup.py bdist_wheel 
    $ check-wheel-contents ./dist
    ```

5.  Build and verify the speach board Library

    ```
    $ cd ros2bot/libraries/ros2bot_speach_lib         
    $ python3 setup.py bdist_wheel 
    $ check-wheel-contents ./dist
    ```

Installation of the library packages can be performed by navigating to
the directory that contains the *.whl files, and executing pip3.

    ```
    $ cd ros2bot/libraries/ros2bot_master_lib/dist
    $ pip3 install ros2bot_master_lib*.whl
    ```
    ```
    $ cd ros2bot/libraries/ros2bot_speach_lib/dist
    $ pip3 install ros2bot_speach_lib*.whl
    ```

To install both (in the same directory) at once:

    ```
    $ pip3 install ros2bot/libraries/dist/*.whl
    ```
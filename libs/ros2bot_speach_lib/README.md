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

## Test Script

Run the test script from this library's directory. Use `--help` to see the
available functions and options:

    ```
    $ python3 test_speach_lib.py --help
    $ python3 test_speach_lib.py speech_read --port /dev/r2bspeach
    $ python3 test_speach_lib.py void_write --value 123 --port /dev/r2bspeach
    ```

`void_write` accepts values from 0 to 999. A connected speech board is required
to test either function.
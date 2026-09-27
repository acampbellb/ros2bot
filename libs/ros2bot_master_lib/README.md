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

## Test Script

Run the test script from this library's directory. Use `--help` to see the
available functions and options:

    ```
    $ python3 test_master_lib.py --help
    $ python3 test_master_lib.py get_version --port /dev/r2bserial
    $ python3 test_master_lib.py set_beep --value 50 --port /dev/r2bserial
    ```

The script also supports telemetry functions such as `get_motion_data` and
`get_battery_voltage`. A connected master board is required. Initializing the
driver enables UART servo torque.

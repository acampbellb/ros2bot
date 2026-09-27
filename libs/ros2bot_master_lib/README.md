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

## Install & Test w/in Environment

Ubuntu 24.04 protects its system Python from pip installs (PEP 668). Install the wheel in a virtual environment instead:
```
cd ~/Ros2bot/libs/ros2bot_master_lib
python3 setup.py bdist_wheel
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install --upgrade dist/ros2bot_master_lib-0.0.2-py3-none-any.whl
```

When making a new release after changing the library, increment `version` in
`setup.py` (for example, change `0.0.2` to `0.0.3`), rebuild the wheel, and
install that exact wheel with `--upgrade`. This lets pip recognize the new
release without `--force-reinstall`. Use the wheel filename produced by the
build if its Python/platform tags differ.

Then, with the environment activated, you can test it:
```
python test_master_lib.py --help
python test_master_lib.py get_version --port /dev/r2bserial
```

If creating the environment fails because venv is unavailable, install Ubuntu’s support package first:
```
sudo apt update
sudo apt install python3-venv
```



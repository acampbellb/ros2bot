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

## Install & Test w/in Environment

Ubuntu 24.04 protects its system Python from pip installs (PEP 668). Install the wheel in a virtual environment instead:
```
cd ~/Ros2bot/libs/ros2bot_speach_lib
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install dist/ros2bot_speach_lib*.whl
```

Then, with the environment activated, you can test it:
```
python test_speach_lib.py --help
python test_speach_lib.py get_version --port /dev/r2bserial
```

If creating the environment fails because venv is unavailable, install Ubuntu’s support package first:
```
sudo apt update
sudo apt install python3-venv
```
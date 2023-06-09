#!/usr/bin/env python3

import time

from ros2bot_master_lib import Ros2botMasterDriver

# globals
_driver = Ros2botMasterDriver()

def test_version():
    print(f'ros2bot driver version: {_driver.get_version()}')     

def test_beep(count):
    for i in range(count):
        _driver.set_beep(60)
        time.sleep(0.2)   

def test_voltage():
    vol = _driver.get_battery_voltage()
    print(f'ros2bot battery voltage: {vol}')

def test_current_speed():
    speed = _driver.get_motion_data()
    num_x = int(speed[0]*100)
    num_y = int(speed[1]*100)
    num_z = int(speed[2]*20)
    print(f'ros2bot current speed x: {num_x}, y: {num_y}, z: {num_z}')        

def main():
    global _driver

    test_version()
    test_beep(3)
    test_voltage()
    test_current_speed()

if __name__ == '__main__':
    main()


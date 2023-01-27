#!/usr/bin/env python3

import math
import random
import rclpy
import sys
import threading
import time

from sensor_msgs.msg import Imu, MagneticField, JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32, Int32, Bool
# TODO: from ros2bot_master_lib import MasterDriverLib
from rclpy.node import Node

class Ros2botDriverNode(Node):
    def __init__(self):
        super().__init__("ros2bot_driver_node")
        self.get_logger().info("Hello from ros2bot driver node")
        self.RA2DE = 180 / math.pi
        # TODO: self.master = MasterDriverLib()
        # TODO: self.master.set_car_type(1)

def main(args=None):
    rclpy.init(args=args)
    node = Ros2botDriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


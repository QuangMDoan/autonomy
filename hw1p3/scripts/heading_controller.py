#!/usr/bin/env python3

import numpy as np
import rclpy

from asl_tb3_lib.control import BaseHeadingController 
from asl_tb3_lib.math_utils import wrap_angle 
from asl_tb3_msgs.msg import TurtleBotControl, TurtleBotState 

class HeadingController(BaseHeadingController):
    def __init__(self):
        super().__init__()
        self.kp = 200.0
    
    def compute_control_with_goal(self, state: TurtleBotState, goal: TurtleBotState) -> TurtleBotControl:
        control = TurtleBotControl()
        control.omega = wrap_angle(goal.theta - state.theta) * self.kp
        return control

if __name__ == "__main__":
    rclpy.init()
    controller = HeadingController()

    # Spin the node using rclpy.spin() to keep it running and listening for messages
    rclpy.spin(controller)

    # Ensure to shut down the ROS2 system with rclpy.shutdown() after spinning.
    rclpy.shutdown()

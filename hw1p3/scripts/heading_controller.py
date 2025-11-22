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
        
        heading_diff = goal.theta - state.theta
        wrapped_heading_error = wrap_angle(heading_diff)

        # Use the proportional control formula, ω = kp · err 
        # to compute the angular velocity required for the TurtleBot to correct its heading error
        omega = wrapped_heading_error * self.kp

        # Create a new TurtleBotControl message, set its omega attribute 
        # to the computed angular velocity, and return it
        control = TurtleBotControl()
        control.omega = omega

        return control


if __name__ == "__main__":
    rclpy.init()
    controller = HeadingController()

    # Spin the node using rclpy.spin() to keep it running and listening for messages
    rclpy.spin(controller)

    # Ensure to shut down the ROS2 system with rclpy.shutdown() after spinning.
    rclpy.shutdown()

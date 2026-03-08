#!/usr/bin/env python3
import numpy as np 
import rclpy 
from asl_tb3_lib.control import BaseHeadingController
from asl_tb3_lib.math_utils import wrap_angle
from asl_tb3_msgs.msg import TurtleBotControl, TurtleBotState

def HeadingController(BaseHeadingController):
    def __init__(self):
        super().__init__()
        self.kp = 2.0

    # Compute control given current robot state and goal state
    # Returns: a control command of type TurtleBotControl
    
    def compute_control_with_goal(self, 
        state: TurtleBotState,
        goal: TurtleBotState
    ) -> TurtleBotControl:
        heading_error = wrap_angle(goal.theta-state.theta)
        omega = self.kp*heading_error
        msg = TurtleBotControl()
        msg.omega = omega
        return msg 

if __name__ == "__main__":
    rclpy.init()
    node = HeadingController()
    rclpy.spin(node)
    rclpy.shutdown()

        

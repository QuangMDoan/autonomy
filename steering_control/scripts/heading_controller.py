#!/usr/bin/env python3
import numpy as np 
import rclpy 
from asl_tb3_lib.control import BaseHeadingController
from asl_tb3_lib.math_utils import wrap_angle
from asl_tb3_msgs.msg import TurtleBotState, TurtleBotControl

class HeadingController(BaseHeadingController):
    def __init__(self):
        super().__init__()
        self.kp = 2
    
    def compute_control_with_goal(self,
        state:TurtleBotState, 
        goal:TurtleBotState
    ) -> TurtleBotControl:
        msg = TurtleBotControl()
        msg.omega = wrap_angle(goal.theta-state.theta)*self.kp 
        return msg 
    
if __name__ == "__main__":
    rclpy.init()
    node = HeadingController()
    rclpy.spin(node)
    rclpy.shutdown()


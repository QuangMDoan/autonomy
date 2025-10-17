#!/usr/bin/env python3

import rclpy
from asl_tb3_lib.control import BaseHeadingController
from asl_tb3_lib.math_utils import wrap_angle
from asl_tb3_msgs.msg import TurtleBotState, TurtleBotControl

class HeadingController (BaseHeadingController):
    kp = 2.0  # Proportional control gain
    
    def __init__(self, node_name: str = "heading_controller") -> None:
        super().__init__(node_name)
        
    def compute_control_with_goal(self, state: TurtleBotState, goal: TurtleBotState) -> TurtleBotControl:
        """
        Compute the control command for the TurtleBot to achieve the desired heading.
        
        Args:
            state (TurtleBotState): The current state of the TurtleBot
            goal (TurtleBotState): The desired state (goal) of the TurtleBot
            
        Returns:
            TurtleBotControl: The control command for the TurtleBot
        """
        # Calculate heading error (wrapped to [-π,π])
        heading_error = wrap_angle(goal.theta - state.theta)
        
        # Calculate angular velocity using proportional control
        omega = self.kp * heading_error
        
        # Create and populate control message
        control_msg = TurtleBotControl()
        control_msg.omega = omega
        
        return control_msg

if __name__ == "__main__":
    # Initialize the ROS2 system
    rclpy.init()
    
    try:
        # Create an instance of the HeadingController
        controller = HeadingController()
        
        # Spin the node to keep it running and processing callbacks
        rclpy.spin(controller)
        
    finally:
        # Ensure the ROS2 system is properly shut down
        rclpy.shutdown()

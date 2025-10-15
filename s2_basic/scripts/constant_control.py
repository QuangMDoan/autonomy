#!/usr/bin/env python3

import rclpy                    # ROS2 client library
from rclpy.node import Node     # ROS2 node baseclass

# import the message type to use
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

# Use a timer for publishing Twist messages to /cmd_vel topic.
# 1.​ import message types from library
# 2.​ create and initialize a publisher
# 3.​ construct messages
# 4.​ publish messages
# Twist message type comes from geometry_msgs.msg.
#       TurtleBot is a 2D robot with differential drive dynamics, and
#       therefore, we only control a linear velocity and an angular
#       velocity, i.e. we only need to set two fields in the Twist message:
#           msg = Twist()       # initialize everything by default
#           msg.linear.x = ...  # set this to be the linear velocity
#           msg.angular.z = ... # set this to be the angular velocity
class Heartbeat(Node):
    def __init__(self) -> None:    
        super().__init__("heartbeat")

        # create publisher with: self.create_publisher(<msg type>, <topic>, <qos>)
        self.hb_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # create a timer with: self.create_timer(<second>, <callback>)
        self.hb_timer = self.create_timer(1.0, self.hb_callback)

        # create subscription with: self.create_subscription(<msg type>, <topic>, <callback>, <qos>)
        self.motor_sub = self.create_subscription(Bool, "/health/motor", self.health_callback, 10)

    def hb_callback(self) -> None:
        """
        Heartbeat callback triggered by the timer
        """
        # construct heartbeat message
        msg = Twist()   
        msg.linear.x = 0.2  # set this to be the linear velocity
        msg.angular.z = 0.1 # set this to be the angular velocity
        self.hb_pub.publish(msg)

    def health_callback(self, msg: Bool) -> None:
        """
        Sensor health callback triggered by subscription
        """
        if not msg.data:
            self.get_logger().fatal("Heartbeat stopped")
            self.hb_timer.cancel()

if __name__ == "__main__":
    # initialize ROS2 context (must run before any other rclpy call)
    rclpy.init()

    # instantiate the heartbeat node
    node = Heartbeat()

    # Use ROS2 built-in scheduler for executing the node
    rclpy.spin(node)

    # cleanly shutdown ROS2 context
    rclpy.shutdown()


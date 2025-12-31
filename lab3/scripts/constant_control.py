#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist 

class MoveStraightLine(Node):
    def __init__(self):
        super().__init__("movestraightline")
        self.v = 0.06
        self.theta = 0.0

        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.hb_timer = self.create_timer(1.0, self.hb_callback)
        self.motor_sub = self.create_subscription(Bool, "/kill", self.kill_callback, 10)

    def hb_callback(self) -> None:
        msg = Twist()
        msg.linear.x = self.v
        msg.angular.z = self.theta
        self.publisher.publish(msg)

    def kill_callback(self, msg: Bool):
        if msg.data:
            msg2 = Twist()
            self.publisher.publish(msg2)
            self.get_logger().fatal("killed the timer")
            self.hb_timer.cancel()

if __name__ == "__main__":
    rclpy.init()
    node = MoveStraightLine()
    rclpy.spin(node)
    rclpy.shutdown()

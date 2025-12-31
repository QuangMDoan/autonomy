#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist 


class HBeat(Node):
    def __init__(self):
        super().__init__("heartbeat")
        self.v = 0.5
        self.theta = 0.2

        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.hb_timer = self.create_timer(1.0, self.hb_callback)
        self.motor_sub = self.create_subscription(Bool, "/health/motor", self.health_callback, 10)

    def hb_callback(self) -> None:
        msg = Twist()
        msg.linear.x = self.v
        msg.angular.z = self.theta

        self.publisher.publish(msg)

    def health_callback(self, msg: Bool):
        if not msg.data:
            msg = Twist()
            self.publisher.publish(msg)
            
            self.get_logger().fatal("heartbeat stopped")

            self.hb_timer.cancel()

if __name__ == "__main__":
    # initialize ROS2 context 
    rclpy.init()
    node = HBeat()
    rclpy.spin(node)
    rclpy.shutdown()

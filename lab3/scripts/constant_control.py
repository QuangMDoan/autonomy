#!/usr/bin/env python3

import rclpy 
from rclpy.node import Node 
from std_msgs.msg import Bool 
from geometry_msgs.msg import Twist

class Heartbeat(Node):
    def __init__(self):
        super().__init__("heartbeat")
        self.hb_timer = self.create_timer(0.5, self.hb_callback)
        self.hb_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.kill_subscriber = self.create_subscription(Bool, "/kill", self.kill_callback, 10)
        self.get_logger.info("heartbeat node started")

    def hb_callback(self):
        msg = Twist()
        msg.linear.x = 0.2
        msg.angular.z = 0.1
        self.hb_publisher.publish(msg)
        self.get_logger().info(f"Publishing: linear.x {msg.linear.x}, angular.z {msg.angular.z}")
    
    def kill_callback(self, zero_twist: Bool):
        if zero_twist.data:
            self.hb_timer.cancel()
            zero_twist = Twist()
            self.hb_publisher.publish(zero_twist)
            self.get_logger().info(f"Published zero velocity to cmd/vel")

if __name__ == "__main__":
    rclpy.init()
    node: Heartbeat = Heartbeat()
    rclpy.spin(node)
    rclpy.shutdown()

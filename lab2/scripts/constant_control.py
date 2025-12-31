#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64, Bool

class HBeat(Node):
    def __init__(self):
        super().__init__("heartbeat")
        self.count :Int64 = 0

        self.publisher = self.create_publisher(Int64, "/heartbeat", 10)
        self.hb_timer = self.create_timer(1.0, self.hb_callback)
        self.motor_sub = self.create_subscription(Bool, "/health/motor", self.health_callback, 10)

    def hb_callback(self) -> None:
        msg = Int64()
        msg.data = self.count
        self.publisher.publish(msg)
        self.count += 1

    def health_callback(self, msg: Bool):
        if not msg.data:
            self.get_logger().fatal("heartbeat stopped")
            self.hb_timer.cancel()

if __name__ == "__main__":
    # initialize ROS2 context 
    rclpy.init()
    node = HBeat()
    rclpy.spin(node)
    rclpy.shutdown()

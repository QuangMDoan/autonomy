#!/usr/bin/env python3 

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

class ConstantControlNode(Node):
    def __init__(self) -> None: 
        super().__init__("const_ctrl_node")
        self.msg_ctr = 0
        self.const_timer = self.create_timer(0.2, self.timer_callback)
        self.ctrl_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.kill_sub = self.create_subscription(Bool, "/kill", self.kill_callback, 10)

    def timer_callback(self) -> None: 
        self.msg_ctr += 1
        msg = Twist() # 0 initialized by default
        msg.linear.x = 2.0
        #msg.angular.z = 0.0
        self.ctrl_pub.publish(msg)

    def kill_callback(self, msg: Bool):
        if msg.data:
            self.const_timer.cancel()
            self.ctrl_pub.publish(Twist())

if __name__ == "__main__":
    rclpy.init()
    node = ConstantControlNode()
    rclpy.spin(node)
    rclpy.shutdown()
#!/usr/bin/env python3 

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ConstantControlNode(Node):
    def __init__(self) -> None: 
        super().__init__("const_ctrl_node")
        self.msg_ctr = 0
        self.const_timer = self.create_timer(0.2, self.timer_callback)
        self.ctrl_pub = self.create_publisher(String, "/const_ctrl", 10)

    def timer_callback(self) -> None: 
        self.msg_ctr += 1
        msg = String()
        msg.data = "sending constant control %d.." % self.msg_ctr
        self.ctrl_pub.publish(msg)

if __name__ == "__main__":
    rclpy.init()
    node = ConstantControlNode()
    rclpy.spin(node)
    rclpy.shutdown()
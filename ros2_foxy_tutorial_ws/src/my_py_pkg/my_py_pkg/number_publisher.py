#!/usr/bin/env python3
# ros client library: rcl
import rclpy
from rclpy.node import Node

from example_interfaces.msg import Int64
 
class NumPubNode(Node): 
    def __init__(self):
        super().__init__("number_publisher") 

        self.pub_number = 2
        self.publisher_ = self.create_publisher(Int64, "number", 10)
        self.counter_ = self.create_timer(0.5, self.publish_numbers)
        self.get_logger().info("Number publisher started")
 
    def publish_numbers(self):
        msg = Int64()
        
        msg.data = self.pub_number
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = NumPubNode() 
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
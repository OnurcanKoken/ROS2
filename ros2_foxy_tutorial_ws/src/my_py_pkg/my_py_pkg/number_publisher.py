#!/usr/bin/env python3
# ros client library: rcl
import rclpy
from rclpy.node import Node

from example_interfaces.msg import Int64
 
class NumPubNode(Node): 
    def __init__(self):
        super().__init__("number_publisher") 

        # test functionality of declare_parameter
        #self.declare_parameter("test_param")
        #self.declare_parameter("second_test_param")

        self.declare_parameter("number_to_publish", 2)
        self.pub_number = self.get_parameter("number_to_publish").value
        # $ ros2 run my_py_pkg number_publisher --ros-args -p number_to_publish:=4
        
        self.declare_parameter("publish_frq", 2.0)
        self.publish_frq_ = self.get_parameter("publish_frq").value

        # self.pub_number = 2
        self.publisher_ = self.create_publisher(Int64, "number", 10)
        self.counter_ = self.create_timer(1.0/self.publish_frq_, self.publish_numbers)
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
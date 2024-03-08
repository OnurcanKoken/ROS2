#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
 
from example_interfaces.msg import String

class SmartphoneNone(Node): 
    def __init__(self):
        super().__init__("phone") 
        # message type, topic name to subscribe, callback, 
        # queue size in case they arrive late (buffer)
        self.subscriber_ = self.create_subscription(
            String, "robot_news", self.callback_robot_news, 10)
        self.get_logger().info("Smartphone has been started.")
 
    def callback_robot_news(self, msg):
        self.get_logger().info(msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = SmartphoneNone() 
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
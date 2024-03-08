#!/usr/bin/env python3

# to use ros2 functionalities
import rclpy
from rclpy.node import Node

class MyNode(Node):

    def __init__(self):
        super().__init__("py_test") # name of the node
        self.counter_ = 0
        self.get_logger().info("Hello ROS2!!!")
        # period between two callbacks, 0.5 seconds -> 2 Hertz
        self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        self.counter_ += 1
        self.get_logger().info("Hello " + str(self.counter_))

def main(args=None):
    # initialize ros2 communication
    rclpy.init(args=args) # pass arguments
    node = MyNode()
    # pause the program here, allow your node to be alive
    # later callbacks will be able to be called from spin()
    rclpy.spin(node) 
    rclpy.shutdown() # last line

if __name__ == "__main__":
    main()
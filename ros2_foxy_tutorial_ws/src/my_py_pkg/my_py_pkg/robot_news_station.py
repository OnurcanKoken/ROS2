#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
 
from example_interfaces.msg import String

class RobotNewsStationNode(Node): 
    def __init__(self):
        super().__init__("robot_news_station") 
        
        self.robot_name_ = "286"
        # robot_news is the topic
        # 10, queue size, is like a buffer, it keeps some messages before it is processed,
        #   if some messages are late, up to 10 messages will be kept, but then messages will be lost
        self.publisher_ = self.create_publisher(String, "robot_news", 10)
        self.timer_ = self.create_timer(0.5, self.publish_news) # 2 Hz
        self.get_logger().info("Robot News Station has been started")
    
    def publish_news(self):
        msg = String()
        #msg.data = "Hello" # this is the field, data
        msg.data = "Hi, this is " + str(self.robot_name_) + " from the robot news station."
        self.publisher_.publish(msg)
        
 
 
def main(args=None):
    rclpy.init(args=args)
    node = RobotNewsStationNode() 
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
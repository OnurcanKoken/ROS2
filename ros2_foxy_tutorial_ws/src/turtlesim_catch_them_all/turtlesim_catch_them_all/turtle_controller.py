#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from functools import partial

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

from my_robot_interfaces.msg import Turtle
from my_robot_interfaces.msg import TurtleArray
from my_robot_interfaces.srv import CatchTurtle

class TurtleControllerNone(Node): 
    def __init__(self):
        super().__init__("turtle_controller")
        self.declare_parameter("catch_closest_turtle", True)

        self.turtle_to_catch_ = None
        self.catch_closest_turtle_ = self.get_parameter("catch_closest_turtle").value
        self.pose_ = None
        self.pose_subscriber_ = self.create_subscription(
            Pose, "turtle1/pose", self.callback_turtle_pose, 10)
        self.cmd_vel_publisher_ = self.create_publisher(
            Twist, "turtle1/cmd_vel", 10)

        self.alive_turtles_subscriber_ = self.create_subscription(
            TurtleArray, "alive_turtles", self.callback_alive_turtles, 10)
        self.control_timer_ = self.create_timer(0.01, self.control_loop)        
        self.get_logger().info("Turtle controller has been started.")
    
    def callback_turtle_pose(self, msg):
        self.pose_ = msg
    
    def callback_alive_turtles(self, msg):
        if len(msg.turtles) > 0: 
            if self.catch_closest_turtle_:
                closest_turtle = None
                closest_turtle_distance = None

                for turtle in msg.turtles:
                    dist_x = turtle.x - self.pose_.x
                    dist_y = turtle.y - self.pose_.y
                    distance = math.sqrt(dist_x**2 + dist_y**2)
                    if closest_turtle == None or distance < closest_turtle_distance:
                        # update the closest turtle
                        closest_turtle = turtle
                        closest_turtle_distance = distance
                self.turtle_to_catch_ = closest_turtle

            else:
                self.turtle_to_catch_ = msg.turtles[0]

    def control_loop(self):
        if self.pose_ == None or self.turtle_to_catch_ == None:
            return 
        
        dist_x = self.turtle_to_catch_.x - self.pose_.x
        dist_y = self.turtle_to_catch_.y - self.pose_.y
        distance = math.sqrt(dist_x**2 + dist_y**2)

        twist_msg = Twist()

        # when the master turtle reach target with a threshold distance
        # P controller to tune linear and angular velocities of the turtle
        if distance > 0.5:
            # position
            twist_msg.linear.x = 3*distance 
            # orientation
            goal_theta = math.atan2(dist_y, dist_x)
            diff = goal_theta - self.pose_.theta 
            # normalize between -2pi to 2pi
            if diff > math.pi:
                diff -= 2*math.pi 
            elif diff < -math.pi:
                diff += 2*math.pi 
            
            twist_msg.angular.z = 6*diff 

        else: # target reached, stop the turtle
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            self.call_catch_turtle_server(self.turtle_to_catch_.name)
            self.turtle_to_catch_ = None

        self.cmd_vel_publisher_.publish(twist_msg)

    def call_catch_turtle_server(self, turtle_name):
        # create the client
        # service name is the same, the one in server
        client = self.create_client(CatchTurtle, "catch_turtle")

        while not client.wait_for_service(1.0):
            # server is not up
            self.get_logger().warn("waiting for server Catch turtle...")
        
        request = CatchTurtle.Request()
        request.name = turtle_name

        future = client.call_async(request)
        # when future got a result, server sent a response 
        future.add_done_callback(partial(
            self.callback_call_catch_turtle, turtle_name=turtle_name))
    
    def callback_call_catch_turtle(self, future, turtle_name):
        # process the result
        try:
            response = future.result()
            if not response.success:
                self.get_logger().error("Turtle " + str(turtle_name) + " could not be caught")
                        

        except Exception as e:
            self.get_logger().error("Kill service call failed %r" % (e,))

def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNone() 
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
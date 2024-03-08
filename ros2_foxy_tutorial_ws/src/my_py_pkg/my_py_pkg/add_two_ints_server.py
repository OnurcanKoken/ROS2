#!/usr/bin/env python3
# ros client library: rcl
import rclpy
from rclpy.node import Node
 
from example_interfaces.srv import AddTwoInts
 
class AddTwoIntsServerNode(Node):
    def __init__(self):
        super().__init__("add_two_ints_server") 
        # service type, name of service, callback func
        self.server_ = self.create_service(AddTwoInts, "add_two_ints", self.callback_add_two_ints)
        self.get_logger().info("Add two ints server started.")
    
    def callback_add_two_ints(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(str(request.a) + " + " + str(request.b) + " = " + str(response.sum))
        return response
 
def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsServerNode() 
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
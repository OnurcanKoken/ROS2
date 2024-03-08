#!/usr/bin/env python3
# ros client library: rcl
import rclpy
from rclpy.node import Node
from functools import partial

# service definitaion is the same, the one in server
from example_interfaces.srv import AddTwoInts
 
class AddTwoIntsClientNode(Node): 
    def __init__(self):
        super().__init__("add_two_ints_client") 
        # you can call multiple times here
        self.call_add_two_ints_server(6, 7)
        self.call_add_two_ints_server(16, 17)
        self.call_add_two_ints_server(35, 17)
    
    def call_add_two_ints_server(self, a, b):
        # create the client
        # service name is the same, the one in server
        client = self.create_client(AddTwoInts, "add_two_ints")
        # wait for the server, otherwise request might fail
        # timeout_sec float
        # if you dont add any time here, it will just wait
        # wait 1 second
        while not client.wait_for_service(1.0):
            # server is not up
            self.get_logger().warn("waiting for server Add Two Ints...")
        
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        # syncronous or asyncronous calls
        # sync one might stuck,
        # use async
        # it will send a request, then continue to be executed
        # future object contains a value that may be set in the future
        future = client.call_async(request)
        # when future got a result, server sent a response 
        future.add_done_callback(partial(self.callback_call_add_two_ints, a=a, b=b))
    
    def callback_call_add_two_ints(self, future, a, b):
        # process the result
        try:
            response = future.result()
            self.get_logger().info(str(a) + " + " + 
                                   str(b) + " = " + str(response.sum))
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsClientNode() 
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
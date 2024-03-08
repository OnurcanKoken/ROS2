#!/usr/bin/env python3
# this functionality is like calling the ros2 service in terminal

# ros client library: rcl
import rclpy
from rclpy.node import Node

# service definitaion is the same, the one in server
from example_interfaces.srv import AddTwoInts
 
def main(args=None):
    rclpy.init(args=args)
    node = Node("add_two_ints_no_opp") 
    
    # create the client
    # service name is the same, the one in server
    client = node.create_client(AddTwoInts, "add_two_ints")

    # wait for the server, otherwise request might fail
    # timeout_sec float
    # if you dont add any time here, it will just wait
    # wait 1 second
    while not client.wait_for_service(1.0):
        # server is not up
        node.get_logger().warn("waiting for server Add Two Ints...")
    
    request = AddTwoInts.Request()
    request.a = 48
    request.b = 97

    # syncronous or asyncronous calls
    # sync one might stuck,
    # use async
    # it will send a request, then continue to be executed
    # future object contains a value that may be set in the future
    future = client.call_async(request)
    # spin until future is complete
    rclpy.spin_until_future_complete(node, future)

    try:
        response = future.result()
        node.get_logger().info(str(request.a) + " + " + 
                               str(request.b) + " = " + str(response.sum))
    except Exception as e:
        node.get_logger().error("Service call failed %r" % (e,))

    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
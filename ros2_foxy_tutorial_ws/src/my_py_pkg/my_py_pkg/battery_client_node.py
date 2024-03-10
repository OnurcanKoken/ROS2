#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from functools import partial

# service definitaion is the same, the one in server
from my_robot_interfaces.srv import SetLED

class BatteryClientNode(Node): 
    def __init__(self):
        super().__init__("battery_client") 
        self.battery_state_ = "full"
        self.last_time_battery_state_changed_ = self.get_current_time_sec()
        # check the battery frequently, 10 Hz
        self.battry_timer_ = self.create_timer(0.1, self.check_battery_state)
        self.get_logger().info("Battery node has been started.")
        
    def get_current_time_sec(self):
        secs, nsecs = self.get_clock().now().seconds_nanoseconds()
        return secs + nsecs / 1000000000.0

    def check_battery_state(self):
        time_now = self.get_current_time_sec()
        if self.battery_state_ == "full":
            # after 4 seconds battery gets empty
            if time_now - self.last_time_battery_state_changed_ > 4.0:
                self.battery_state_ = "empty"
                self.get_logger().info("Battery is empty! Charging battery...")
                # update the last battery charging time
                self.last_time_battery_state_changed_ = time_now
                # change led number 3 state as true, so light 3 will turn on
                self.call_set_led_server(3, 1)
        else:
            # after 6 sec, battery gets full
            if time_now - self.last_time_battery_state_changed_ > 6.0:
                self.battery_state_ = "full"
                self.get_logger().info("Battery is full now.")
                # update the last battery charging time
                self.last_time_battery_state_changed_ = time_now
                # change led number 3 state as false, so light 3 will turn off
                self.call_set_led_server(3, 0)


    def call_set_led_server(self, led_number, state):

        # service name is the same, the one in server
        client = self.create_client(SetLED, "set_led")

        while not client.wait_for_service(1.0):
            # server is not up
            self.get_logger().warn("waiting for server LED panel...")
        
        request = SetLED.Request()
        request.led_number = led_number
        request.state = state
        
        future = client.call_async(request)
        # when future got a result, server sent a response 
        future.add_done_callback(partial(self.callback_call_set_led, led_number=led_number, state=state))
    
    def callback_call_set_led(self, future, led_number, state):
        # process the result
        try:
            response = future.result()
            
            if response.success:
                self.get_logger().info("LED state updated")
                self.get_logger().info("LED number: " + str(led_number) + ", state: " + str(state))
            else:
                self.get_logger().error("LED state is not updated")

        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

def main(args=None):
    rclpy.init(args=args)
    node = BatteryClientNode() 
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
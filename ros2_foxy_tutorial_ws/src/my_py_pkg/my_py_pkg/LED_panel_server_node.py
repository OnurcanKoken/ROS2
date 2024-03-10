#!/usr/bin/env python3
# ros client library: rcl
import rclpy
from rclpy.node import Node

from my_robot_interfaces.srv import SetLED
from my_robot_interfaces.msg import StateLED

class LEDPanelServerNode(Node):
    def __init__(self):
        super().__init__("led_panel_server") 
        self.led_status = [0, 0, 0]
        # service type, name of service, callback func
        self.publisher_ = self.create_publisher(StateLED, "led_panel_state", 10)
        self.update_ = self.create_timer(4, self.update_publish_led_state)
        self.server_ = self.create_service(SetLED, "set_led", self.callback_set_led)
        self.get_logger().info("LED panel server started.")
        
    
    def callback_set_led(self, request, response):
        
        if request.led_number > len(self.led_status) or request.led_number <= 0:
            response.success = False
            return response
        
        if request.state not in [0, 1]:
            response.success = False 
            return response
        
        self.led_status[(request.led_number-1)] = request.state
        response.success = True
        self.get_logger().info("LED panel update, LED number " + str(request.led_number) + " state is " + str(request.state))
        self.get_logger().info("LED panel update, LED array " + str(self.led_status))
        self.update_publish_led_state()

        return response
    
    def update_publish_led_state(self):
        msg = StateLED()
        msg.data = self.led_status
        self.publisher_.publish(msg)
        

def main(args=None):
    rclpy.init(args=args)
    node = LEDPanelServerNode() 
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
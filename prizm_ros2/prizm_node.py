#! /usr/bin/env python3

import rclpy
import rclpy.logging
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from serial import Serial

SPEED_MUX = 10

def twist_to_point(msg : Twist) -> tuple[float, float]:
    left = msg.linear.x * -1
    right = msg.linear.x * -1
    
    left -= msg.angular.z
    right += msg.angular.z
    
    left *= SPEED_MUX
    right *= SPEED_MUX
    
    left = max(left, 0)
    left = min(left, 100)
    right = max(right, 0)
    right = min(right, 100)
    
    return (left, right)

class PRIZM_Node(Node):

    leftMotorState = 0
    rightMotorState = 0
    greenLedState = 0
    redLedState = 0
    
    serial_conn : Serial

    def __init__(self):
        super().__init__('prizm_node')
        self.declare_parameter("serial_port", "")

        serial_param = self.get_parameter("serial_port")
        self.get_logger().log(serial_param.value, 20)
        self.serial_conn = Serial(serial_param.value, 115200)
        
        self.create_subscription(Twist, "twist_controller", self.twist_cb, 10)
        self.create_subscription(Twist, "green_led", self.green_cb, 10)
        self.create_subscription(Twist, "red_led", self.red_cb, 10)

    def writePrizmSerial(self):
        self.serial_conn.write(bytes(f"left:{self.leftMotorState},right:{self.rightMotorState},green:{self.greenLedState},red:{self.redLedState};", "ascii"))
        
    def twist_cb(self, msg : Twist):
        self.leftMotorState, self.rightMotorState = twist_to_point(msg)
        self.writePrizmSerial()
        
    def green_cb(self, msg : Bool):
        self.greenLedState = 1 if msg.data else 0
        self.writePrizmSerial()
        
    def red_cb(self, msg : Bool):
        self.redLedState = 1 if msg.data else 0
        self.writePrizmSerial()

        
def main(args=None):
    rclpy.init(args=args)
    node = PRIZM_Node()
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
        
if __name__ == '__main__':
    main()
    



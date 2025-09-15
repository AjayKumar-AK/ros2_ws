#!/usr/bin/env python3
import rclpy

from rclpy.node import Node

from geometry_msgs.msg import Twist

import serial



class SerialBridge(Node):

    def __init__(self):

        super().__init__('serial_bridge')

        self.subscription = self.create_subscription(

            Twist,

            '/cmd_vel',

            self.listener_callback,

            10)

        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)  # Replace with your port

        self.get_logger().info("Serial Bridge Node Started")



    def listener_callback(self, msg):

        vx = msg.linear.x * 100  # convert to cm/s or scale

        vy = msg.linear.y * 100

        omega = msg.angular.z * 100



        command = f"{vx:.2f},{vy:.2f},{omega:.2f}\n"

        self.ser.write(command.encode('utf-8'))

        self.get_logger().info(f"Sent: {command.strip()}")



def main(args=None):

    rclpy.init(args=args)

    bridge = SerialBridge()

    rclpy.spin(bridge)

    bridge.destroy_node()

    rclpy.shutdown()



if __name__ == '__main__':

    main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

def emulate_joystick_publisher():
    rclpy.init()

    node = rclpy.create_node('emulate_joystick_publisher')
    publisher = node.create_publisher(Joy, 'joy', 10)

    msg = Joy()
    i = 0
    while rclpy.ok():

        # Emulate joystick data (modify as needed)
        msg.axes = [0.3, 0.0, 0.0, 0.0, 0.0, 0.0]
        msg.buttons = [1, 0, 0, 0,0, 0, 0, 0, 0, 0, 0, 0]
        # while i < 5 : 
        publisher.publish(msg)
        node.get_logger().info('Emulated Joystick Message Published')

        rclpy.spin_once(node, timeout_sec=1)
        i=i+1
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    emulate_joystick_publisher()
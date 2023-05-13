#! /usr/bin/env python3

# this script is used to convert ros2 twist message to serial message and send it to arduino
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

ARDUINO_PORT = '/dev/ttyACM0'



class Twist2Serial(Node):
    def __init__(self):
        super().__init__('twist2serial')
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning
        # self.ser = serial.Serial(ARDUINO_PORT, 9600, timeout=1)
        # self.ser.flush()

    def listener_callback(self, msg):
        msg = str(msg.linear.x) + ',' + str(msg.linear.y) + ',' + str(msg.angular.z) + '\n'
        self.get_logger().info('I heard: "%s"' % msg)
        # self.ser.write(msg.encode('utf-8'))
        line = self.ser.readline().decode('utf-8').rstrip()
        print(line)

def main(args=None):
    rclpy.init(args=args)
    twist2serial = Twist2Serial()
    twist2serial.get_logger().info('twist2serial node started')
    rclpy.spin(twist2serial)

    twist2serial.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
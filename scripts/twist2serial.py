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
        print('Subscribing...')
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning
        print('Subscribed')
        self.ser = serial.Serial(ARDUINO_PORT, 9600, timeout=1)
        print('Openning serial connection')
        self.ser.flush()
        print('Done init')

    def listener_callback(self, msg):
        msg = str(msg.linear.x) + ',' + str(msg.linear.y) + ',' + str(msg.angular.z) + '\n'
        self.get_logger().info('I heard: "%s"' % msg[:-1])
        #msg = "0.6,0.0,0.0\n"
        self.get_logger().info("wrritting: " + msg)
        self.ser.write(msg.encode('utf-8'))
        self.get_logger().info("Written")
        # if self.ser.readable():
        #     self.ser.read()
        self.get_logger().info("After read")
        #print("res: " + line)

def main(args=None):
    rclpy.init(args=args)
    twist2serial = Twist2Serial()
    twist2serial.get_logger().info('twist2serial node started')
    rclpy.spin(twist2serial)
    print('After spin')

    twist2serial.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

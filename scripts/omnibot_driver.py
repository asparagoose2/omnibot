#!/usr/bin/env python3
import math
import rclpy
from rclpy.time import Duration
from geometry_msgs.msg import Twist

def move():
    # Starts a new node
    rclpy.init()
    
    node = rclpy.create_node('omnibot')
    velocity_publisher= node.create_publisher(Twist, '/cmd_vel', 10)
    vel_msg = Twist()

    #Receiveing the user's input
    while True:
        print("Let's move your robot")
        speed = float(input("Input your speed: "))
        distance = float(input("Type your distance: "))
        direction = float(input("Type your direction:(0-360): "))
        angular_vel = float(input("Type your angular_vel: "))
        # isForward = input("Foward?: ")#True or False

        vel_msg.linear.y=0.0
        vel_msg.angular.y = 0.0
        vel_msg.angular.x = 0.0
        vel_msg.angular.z = 0.0

        # calculate x and z velocity
        vel_msg.linear.x = abs(speed)*math.cos(direction*math.pi/180.0)
        vel_msg.linear.y = abs(speed)*math.sin(direction*math.pi/180.0)
        vel_msg.angular.z = angular_vel

        # if(isForward):
            # vel_msg.linear.x = abs(speed)
        # else:
            # vel_msg.linear.x = -abs(speed)
        #Since we are moving just in x-axis
        vel_msg.linear.z = 0.0
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0



        #Setting the current time for distance calculus
        t0 = node.get_clock().now()
        current_distance = 0.0

        #Loop to move the turtle in an specified distance
        while(current_distance < distance):
            #Publish the velocity
            velocity_publisher.publish(vel_msg)
            #Takes actual time to velocity calculus
            t1=node.get_clock().now()
            #Calculates distancePoseStamped
            current_distance= speed*(t1-t0).nanoseconds/1e9
        #After the loop, stops the robot
        vel_msg.linear.x = 0.0
        vel_msg.linear.y = 0.0
        vel_msg.angular.z = 0.0
        #Force the robot to stop
        velocity_publisher.publish(vel_msg)

if __name__ == '__main__':
    try:
        #Testing our function
        move()
    except rclpy.ROSInterruptException: pass
#!/usr/bin/env python3

import math
import rclpy
from rclpy.client import Client
from rclpy.node import Node
from rclpy.publisher import Publisher
import time
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.msg import EntityState
from sensor_msgs.msg import LaserScan
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.srv import GetEntityState
from math import *
import numpy as np
from tf_transformations import euler_from_quaternion, quaternion_from_euler

# Configuration
MODEL_NAME = "omnibot"


# Q-learning speed parameters
CONST_LINEAR_SPEED_FORWARD = 0.8
CONST_ANGULAR_SPEED_FORWARD = 0.0
CONST_LINEAR_SPEED_TURN = 0.6
CONST_ANGULAR_SPEED_TURN = 0.5

# Feedback control parameters
K_RO = 2
K_ALPHA = 15
K_BETA = -3
V_CONST = 0.1 # [m/s]

# Goal reaching threshold
GOAL_DIST_THRESHOLD = 0.1 # [m]
GOAL_ANGLE_THRESHOLD = 15 # [degrees]

# Get theta in [radians]
def getRotation(model_state: ModelState):
    orientation_q = model_state.state.pose.orientation
    orientation_list = [ orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    return yaw

# Get (x,y) coordinates in [m]
def getPosition(model_state: ModelState):
    x = model_state.state.pose.position.x
    y = model_state.state.pose.position.y
    return ( x , y)

# Get linear speed in [m/s]
def getLinVel(odomMsg: Odometry):
    return odomMsg.twist.twist.linear.x

# Get angular speed in [rad/s] - z axis
def getAngVel(odomMsg: Odometry):
    return odomMsg.twist.twist.angular.z

# Create rosmsg Twist()
def createVelMsg(velocity,angle,rotation=0.0):
    velMsg = Twist()

    # convert angle to radians
    angle = math.radians(angle)

    # calculate linear and angular velocities
    velMsg.linear.x = velocity * math.cos(angle)
    velMsg.linear.y = velocity * math.sin(angle)
    velMsg.linear.z = 0.0

    velMsg.angular.x = 0.0
    velMsg.angular.y = 0.0
    velMsg.angular.z = rotation

    return velMsg


def robotGoAngle(velPub: Publisher, angle, rotation):
    if angle == -1:
        velMsg = createVelMsg(0.0,0,rotation * 4)
    else:
        velMsg = createVelMsg(CONST_LINEAR_SPEED_FORWARD,angle,rotation)
    velPub.publish(velMsg)
# Go forward command
def robotGoForward(velPub: Publisher):
    velMsg = createVelMsg(CONST_LINEAR_SPEED_FORWARD,CONST_ANGULAR_SPEED_FORWARD)
    velPub.publish(velMsg)

# Turn left command
def robotTurnLeft(velPub):
    velMsg = createVelMsg(CONST_LINEAR_SPEED_TURN,+CONST_ANGULAR_SPEED_TURN)
    velPub.publish(velMsg)

# Turn right command
def robotTurnRight(velPub):
    velMsg = createVelMsg(CONST_LINEAR_SPEED_TURN,-CONST_ANGULAR_SPEED_TURN)
    velPub.publish(velMsg)

# Stop command
def robotStop(velPub):
    velMsg = createVelMsg(0.0,0.0,0.0)
    velPub.publish(velMsg)

def spin_until_service_complete(node, response, verbose=False):
    rclpy.spin_until_future_complete(node, response)
    if response.result() is not None:
        if verbose:
            node.get_logger().info('SERVICE COMPLETE! RESULT:\n{}'.format(response.result()))
        return response.result()

# Set robot position and orientation
def robotSetPos(setModelStateClient: Client, x, y, theta, node):
    
    print("Setting robot position to: x = ", x, " y = ", y, " theta = ", theta)

    request: SetEntityState.Request = SetEntityState.Request()
    checkpoint = EntityState()

    checkpoint.name = MODEL_NAME
    checkpoint.pose.position.x = x
    checkpoint.pose.position.y = y
    checkpoint.pose.position.z = 0.0

    [x_q,y_q,z_q,w_q] = quaternion_from_euler(0.0,0.0,radians(theta))

    checkpoint.pose.orientation.x = x_q
    checkpoint.pose.orientation.y = y_q
    checkpoint.pose.orientation.z = z_q
    checkpoint.pose.orientation.w = w_q

    checkpoint.twist.linear.x = 0.0
    checkpoint.twist.linear.y = 0.0
    checkpoint.twist.linear.z = 0.0

    checkpoint.twist.angular.x = 0.0
    checkpoint.twist.angular.y = 0.0
    checkpoint.twist.angular.z = 0.0

    request.state = checkpoint

    response = setModelStateClient.call_async(request)

    spin_until_service_complete(node, response)

    return ( x , y , theta )

# Set random initial robot position and orientation
def robotSetRandomPos(client: Client, node: Node):
    theta_range = np.arange(80,100, 5)
    randPos = [(-1.6269804301731562e-13, 1.5456203422175541e-13), (-0.20935094356536865, 0.20480971038341522), (0.04647219181060791, 0.16526779532432556), (-0.1246488094329834, -0.04933095723390579)]

    ind_theta = np.random.randint(0,len(theta_range))

    inx = np.random.randint(0,len(randPos))
    x = randPos[inx][0]
    y = randPos[inx][1]

    theta = theta_range[ind_theta]

    return robotSetPos(client, x, y, theta, node)

# Perform an action
def robotDoAction(velPub, action):
    # print('robotDoAction => action: ', action)
    status = 'robotDoAction => OK'
    if action[1] == 0:
        robotGoAngle(velPub, action[0],0.0)
    elif action[1] == 1:
        robotGoAngle(velPub, action[0],CONST_ANGULAR_SPEED_TURN)
    elif action[1] == -1:
        robotGoAngle(velPub, action[0], -CONST_ANGULAR_SPEED_TURN)
    else:
        status = 'robotDoAction => INVALID ACTION'
        robotGoAngle(velPub, 0,0)

    return status

def getDistanceToGoal(x, y, x_goal, y_goal):
    return sqrt( pow( ( x_goal - x ) , 2 ) + pow( ( y_goal - y ) , 2) )

# Feedback Control Algorithm
def robotFeedbackControl(velPub, x, y, theta, x_goal, y_goal, theta_goal):
    # theta goal normalization
    if theta_goal >= pi:
        theta_goal_norm = theta_goal - 2 * pi
    else:
        theta_goal_norm = theta_goal

    ro = sqrt( pow( ( x_goal - x ) , 2 ) + pow( ( y_goal - y ) , 2) )
    lamda = atan2( y_goal - y , x_goal - x )

    alpha = (lamda -  theta + pi) % (2 * pi) - pi
    beta = (theta_goal - lamda + pi) % (2 * pi) - pi

    if ro < GOAL_DIST_THRESHOLD and degrees(abs(theta-theta_goal_norm)) < GOAL_ANGLE_THRESHOLD:
        status = 'Goal position reached!'
        v = 0
        w = 0
        v_scal = 0
        w_scal = 0
    else:
        status = 'Goal position not reached!'
        v = K_RO * ro
        w = K_ALPHA * alpha + K_BETA * beta
        v_scal = v / abs(v) * V_CONST
        w_scal = w / abs(v) * V_CONST

    velMsg = createVelMsg(v_scal, w_scal)
    velPub.publish(velMsg)

    return status


def getGazeboModelState(node: Node, model_name: str):
    getModelStateClient = node.create_client(GetEntityState, '/get_entity_state')
    request = GetEntityState.Request()
    request.name = model_name
    response = getModelStateClient.call_async(request)
    spin_until_service_complete(node, response)
    return response.result()

def getGazeboModelPossitoin(node: Node, model_name: str):
    model_state = getGazeboModelState(node, model_name)
    if model_state.success:
        pos_x, pos_y = getPosition(model_state)
        tetah = getRotation(model_state)
        return (pos_x, pos_y, tetah)
    else:
        return None

def recordValidPositions(node: Node, model_name: str, valid_positions: list, num_positions: int):
    node.get_logger().info('Recording valid positions...')
    for i in range(num_positions):
        node.get_logger().info('getting model state...')
        model_state = getGazeboModelState(node, model_name)
        node.get_logger().info('model state received!')
        if model_state.success:
            print(model_state)
            thetha = getRotation(model_state)
            print('thetha: ', thetha)
            print('theta in degrees: ', degrees(thetha))
            print('orientation: ', model_state.state.pose.orientation)
            print('Valid position: ', model_state.state.pose.position)
            valid_positions.append((model_state.state.pose.position.x, model_state.state.pose.position.y))
        input('Press enter to continue...')


def callback(msg):
    print('Received message: ')
    print(msg)

if __name__ == "__main__":
    print('Starting omnibot node...')
    rclpy.init()
    GazeboNode = rclpy.create_node('GazeboNode')
    GazeboNode.get_logger().info('GazeboNode started!')
    setModelStateClient = GazeboNode.create_client(SetEntityState, '/set_entity_state')
    GazeboNode.get_logger().info('SetEntityState client created!')
    valid_positions = []

    recordValidPositions(GazeboNode, 'omnibot', valid_positions, 4)
    print('Valid positions: ', valid_positions)
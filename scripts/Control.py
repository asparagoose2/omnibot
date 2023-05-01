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

    # print(request.get_fields_and_field_types())

    checkpoint = EntityState()

    # print(checkpoint.get_fields_and_field_types())

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

    # node.get_logger().info('Waiting for service...')

    response = setModelStateClient.call_async(request)

    spin_until_service_complete(node, response)
    # node.get_logger().info('Service call complete!')
    # node.get_logger().info('\n***************\nService call response: {}\n***************'.format(response.result()))
    # node.get_logger().info('Checkpoint: {}'.format(checkpoint))

    return ( x , y , theta )

# Set random initial robot position and orientation
def robotSetRandomPos(client: Client, node: Node):
    # x_range = np.array([-0.4, 0.6, 0.6, -1.4, -1.4, 2.0, 2.0, -2.5, 1.0, -1.0])
    # y_range = np.array([-0.4, 0.6, -1.4, 0.6, -1.4, 1.0, -1.0, 0.0, 2.0, 2.0])
    theta_range = np.arange(0, 360, 15)
    #theta_range = np.array([0, 30, 45, 60, 75, 90])

    randPos = [(0.8114808201789856, -0.8423820734024048), (0.4912653267383575, -0.1094655692577362), (-0.2814577519893647, -0.011593737639486784), (-0.8663828372955322, -0.030562328174713698), (-1.7206780910491877, -0.5033520460128876), (-3.2132842540740967, -0.11692113429307938), (-2.9126062393188477, -1.2681145668029785), (-0.2523593604564667, -1.3452847003936768), (-0.19050727784633636, -2.3495564460754395), (0.2054444998502799, -3.105850458145151), (-0.9114537835121155, -2.699927568435669), (-2.346564769744873, -1.7842795848846436), (-2.4691739082336426, -2.3897945880889893), (-4.406469345092773, -2.013939619064331), (-4.70806884765625, -1.0620890855789185), (-5.671790599823004, -1.037521481513968), (-6.332071781158447, -1.2194573879241943), (-7.0239181518554625, 0.7174206376075654), (-6.216903209686273, 1.6844897270202543), (-6.104583263397217, 2.8831048011779785), (-3.8269104957580566, 2.7027716636657715), (-2.8275387287139893, 1.2168653011322021), (-2.4848532676696844, 2.1406617164611905), (-0.19280089437961578, 3.0058653354644775), (0.7550399303436279, 2.0091824531555176), (2.019897937774658, 2.7707865238189697), (2.5017614364624023, 0.08740662783384323), (2.4202516078948975, -1.0813194513320923), (2.575488805770874, -2.890676736831665), (2.338424682617194, -3.431960105896005), (1.124169945716858, -3.3821284770965576), (0.3094104528427124, -4.90765380859375), (1.7861343622207575, -5.202110767364492), (4.394073009490967, -4.830124378204346), (4.879225730895996, -2.6312358379364014), (4.403538227081299, -1.4720680713653564), (6.447063922882086, -1.7134674787521453), (7.255248546600336, -3.64595389366149), (6.8566737174987855, -5.252413272857675), (7.400508880615234, -6.834506034851074), (7.373484134674072, -7.973130226135254), (5.684469699859619, -7.812014579772949), (4.2650980949401855, -7.588253974914551), (3.885925531387329, -5.613494396209717), (1.474738359451294, -5.2145676612854), (1.5513279438018799, -7.451404571533203), (-3.5431361198425293, -6.648706912994385), (-4.258673667907715, -7.7032999992370605), (-6.745872974395752, -5.647144794464111), (-7.330343723297119, -3.7117271423339844)]

    # print("len randPos: ", len(randPos))

    # ind = np.random.randint(0,len(x_range))
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

# Stability Condition
def check_stability(k_rho, k_alpha, k_beta):
    return k_rho > 0 and k_beta < 0 and k_alpha > k_rho

# Strong Stability Condition
def check_strong_stability(k_rho, k_alpha, k_beta):
    return k_rho > 0 and k_beta < 0 and k_alpha + 5 * k_beta / 3 - 2 * k_rho / np.pi > 0


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
    for i in range(num_positions):
        model_state = getGazeboModelState(node, model_name)
        if model_state.success:
            print(model_state)
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
    # OdomNode = rclpy.create_node('OdomNode')
    # LaserNode = rclpy.create_node('LaserNode')
    GazeboNode = rclpy.create_node('GazeboNode')

    GazeboNode.get_logger().info('GazeboNode started!')

    setModelStateClient = GazeboNode.create_client(SetEntityState, '/set_entity_state')
    
    # veloPub = GazeboNode.create_publisher(Twist, '/cmd_vel', 10)
    

    # setPosPub = GazeboNode.create_publisher(ModelState, '/gazebo/set_entity_state', 10)

    # odomSub = OdomNode.create_subscription(Odometry,'/odom', callback, 10)
    # laserSub = LaserNode.create_subscription(LaserScan,'/scan', callback, 10)

    valid_positions = []

    recordValidPositions(GazeboNode, 'omnibot', valid_positions, 5)

    print('Valid positions: ', valid_positions)

    # getPosition(OdomNode)
    # res: GetEntityState.Response = getGazeboModelState(GazeboNode, 'omnibot')
    # print(res._state.pose.position.x, res._state.pose.position.y, res._state.pose.position.z)

    # print('Omnibot node started!')

    # robotSetRandomPos(setModelStateClient, GazeboNode)

    # for i in range(10):
    #     # random number between 0 and 360
    #     angle = np.random.randint(0,360)
    #     print('Angle: ', angle)
    #     robotGoAngle(veloPub, angle)
    #     time.sleep(2)

    # robotStop(veloPub)

    # robotGoForward(veloPub)
    # robotSetPos(setModelStateClient, 0.0, 0.0, 0.0,GazeboNode)
    # robotSetRandomPos(setModelStateClient, GazeboNode)
    # sleep 2 seconds
    # time.sleep(2)
    # robotSetRandomPos(setModelStateClient, GazeboNode)
    # sleep 2 seconds
    # time.sleep(2)
    # robotSetRandomPos(setModelStateClient, GazeboNode)
    
    
    # rclpy.spin_once(node=OdomNode)
    # rclpy.spin_once(node=LaserNode)
    # rclpy.spin_once(node=GazeboNode)

    

    # odomMsg = rclpy.wait_for_message('/odom', Odometry)

#! /usr/bin/env python3

import math
import rclpy
from rclpy.node import Node, Client, Publisher
from datetime import datetime
import numpy as np
from sensor_msgs.msg import LaserScan
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from Control import robotStop




from Qlearning import getBestAction, readQTable, createStateSpace, createActions, getStateIndex, getReward, epsiloGreedyExploration
from Control import getDistanceToGoal, robotSetRandomPos, robotDoAction, robotSetPos
from Lidar import lidarScan, lidarReduction, checkCrash

node: Node = None
setModelStateClient: Client = None
msgScan_ready: bool = False
msgScan: LaserScan = None
velPub: Publisher = None

robot_current_x, robot_current_y, robot_current_theta = 0.0, 0.0, 0.0

NUM_OF_EPISODES = 40
NUM_OF_STEPS_PER_EPISODE = 10000
MIN_TIME_BETWEEN_STEPS = 0.05 # [s]

ROOT_PATH = '/home/user/dev_ws/src/omnibot'
Q_TABLE_PATH = '/home/user/dev_ws/src/omnibot/Data/longest_final_log/Qtable_final_v5_02_48hrs.csv.npy'

# Goal position
X_GOAL = 0.016976527428010722
Y_GOAL = 3.2001529447737216

WORLD_TYPE = 'final_v5'

Q_table = readQTable(Q_TABLE_PATH)
state_space = createStateSpace()
action_space = createActions()


def reseetRobotPos():
    if WORLD_TYPE == '4_turns':
        robotSetPos(setModelStateClient,0.0,0.0,-180.0, node)
    elif WORLD_TYPE == 'basic':
        robotSetRandomPos(setModelStateClient, node)
    elif WORLD_TYPE == 'eval-basic':
        valid_posittions = [(0.11231100052654458, 0.37532009830071694), (-0.722751626490168, 0.631599625949322), (-0.14336683336093667, -0.2799617773414318), (-1.0853537928489194, 0.24343765996018002), (-1.794720097188924, 0.511412573332569)] 
        random_position = np.random.randint(0, len(valid_posittions))
        random_rotation = np.random.randint(60, 120)
        random_rotation = float(random_rotation)
        robotSetPos(setModelStateClient,valid_posittions[random_position][0],valid_posittions[random_position][1],random_rotation, node)
    elif WORLD_TYPE == 'eval2':
        valid_posittions = [(-0.5934334993362427, 0.3856262266635895), (-0.05387035384774208, 0.0286132507026247), (-0.3991207778453827, -0.11908558756113052), (-0.22920307517051697, 1.2897175550460815), (-0.4920153319835663, 1.9013009071350098)]
        random_position = np.random.randint(0, len(valid_posittions))   

        robotSetPos(setModelStateClient,valid_posittions[random_position][0],valid_posittions[random_position][1],0.0, node)
    elif WORLD_TYPE == 'final_v5':
        valid_posittions = [(-1.6269804301731562e-13, 1.5456203422175541e-13), (-0.20935094356536865, 0.20480971038341522), (0.04647219181060791, 0.16526779532432556), (-0.1246488094329834, -0.04933095723390579)]
        random_position = np.random.randint(0, len(valid_posittions))
        # random int rangee 80 to 100 with step 5
        random_rotation = np.random.randint(80, 100)
        random_rotation = float(random_rotation)
        robotSetPos(setModelStateClient,valid_posittions[random_position][0],valid_posittions[random_position][1],random_rotation, node)
    
    elif WORLD_TYPE == 'real':
        # wait for space key to be pressed
        input("Press Enter to continue...")



    else:
        raise ValueError('World type not supported')
    

def eval():
    global msgScan_ready

    reward_per_episode = np.zeros(NUM_OF_EPISODES)
    distance_to_goal = math.inf
    prev_distance_to_goal = math.inf

    while rclpy.ok():
        for episode in range(NUM_OF_EPISODES):
            node.get_logger().info('Episode: ' + str(episode))

            reseetRobotPos()

            step_time = node.get_clock().now()


            for step in range(NUM_OF_STEPS_PER_EPISODE):
                while (node.get_clock().now() - step_time).nanoseconds / 1e9 < MIN_TIME_BETWEEN_STEPS:
                    pass
                step_time = node.get_clock().now()
                if(step % 100 == 0):
                    node.get_logger().info('Step: ' + str(step))

                msgScan_ready = False
                while not msgScan_ready:
                    rclpy.spin_once(node)
                
                ( lidar, angles ) = lidarScan(msgScan)
                scan_reduction = lidarReduction(lidar)
                state_ind = getStateIndex(state_space, scan_reduction)
                crash = checkCrash(lidar)
                

                action, status = getBestAction(Q_table, state_ind, action_space)
                # action, status = epsiloGreedyExploration(Q_table, state_ind, action_space, 0.001)
                robotDoAction(velPub, action)

                prev_lidar = lidar
                prev_action = action
                prev_state_ind = state_ind
                prev_distance_to_goal = distance_to_goal
                distance_to_goal = getDistanceToGoal(robot_current_x, robot_current_y, X_GOAL, Y_GOAL)
                ( reward, terminal_state ) = getReward(action, prev_action, lidar, prev_lidar, crash, distance_to_goal, prev_distance_to_goal)
                reward_per_episode[episode] = reward

                if terminal_state or crash:
                    node.get_logger().info('Terminal state')
                    break

        node.get_logger().info('Reward per episode: ' + str(reward_per_episode))
        node.get_logger().info('Average reward: ' + str(np.mean(reward_per_episode)))
        break



def robotDescCallback(msg: ModelStates):
    # print("robotDescCallback")
    global robot_current_x, robot_current_y, robot_current_theta
    # find robot index in msg
    robot_index = msg.name.index('omnibot')
    # get robot pose
    robot_current_x = msg.pose[robot_index].position.x
    robot_current_y = msg.pose[robot_index].position.y
    robot_current_theta = msg.pose[robot_index].orientation.z



def laserCallback(msg: LaserScan):
    global msgScan, msgScan_ready
    msgScan = msg
    msgScan_ready = True

def main():
    global node, setModelStateClient, velPub

    rclpy.init()
    node = Node('eval_node')
    node.get_logger().info('eval_node started')
    node.create_rate(10)
    setModelStateClient = node.create_client(SetEntityState, '/set_entity_state')
    robotDescSub = node.create_subscription(ModelStates, '/model_states', robotDescCallback, 10)
    laserSub = node.create_subscription(LaserScan,'/scan', laserCallback, 10)
    velPub = node.create_publisher(Twist, '/cmd_vel', 10)

    eval()
    robotStop(velPub)
    node.get_logger().info('eval_node finished')
    
    rclpy.shutdown()



if __name__ == '__main__':
    main()
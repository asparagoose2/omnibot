#! /usr/bin/env python3

import rclpy
from rclpy.node import Node, Client, Publisher
from datetime import datetime
import numpy as np
from sensor_msgs.msg import LaserScan
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Twist




from Qlearning import getBestAction, readQTable, createStateSpace, createActions, getStateIndex, getReward, epsiloGreedyExploration
from Control import robotSetRandomPos, robotDoAction, robotSetPos
from Lidar import lidarScan, lidarReduction, checkCrash

node: Node = None
setModelStateClient: Client = None
msgScan_ready: bool = False
msgScan: LaserScan = None
velPub: Publisher = None

NUM_OF_EPISODES = 10
NUM_OF_STEPS_PER_EPISODE = 1000

ROOT_PATH = '/home/user/dev_ws/src/omnibot'
Q_TABLE_PATH = '/home/user/dev_ws/src/omnibot/Data/Log_learning/Qtable-longeest-27.csv.npy'

WORLD_TYPE = 'eval-basic'

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
    else:
        raise ValueError('World type not supported')
    

def eval():
    global msgScan_ready

    reward_per_episode = np.zeros(NUM_OF_EPISODES)

    while rclpy.ok():
        for episode in range(NUM_OF_EPISODES):
            node.get_logger().info('Episode: ' + str(episode))

            reseetRobotPos()


            for step in range(NUM_OF_STEPS_PER_EPISODE):
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

                ( reward, terminal_state ) = getReward(action, prev_action, lidar, prev_lidar, crash)
                reward_per_episode[episode] = reward

                if terminal_state or crash:
                    node.get_logger().info('Terminal state')
                    break

        node.get_logger().info('Reward per episode: ' + str(reward_per_episode))
        node.get_logger().info('Average reward: ' + str(np.mean(reward_per_episode)))
        break





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
    laserSub = node.create_subscription(LaserScan,'/scan', laserCallback, 10)
    velPub = node.create_publisher(Twist, '/cmd_vel', 10)

    eval()
    node.get_logger().info('eval_node finished')
    rclpy.shutdown()



if __name__ == '__main__':
    main()
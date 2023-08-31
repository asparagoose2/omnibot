#! /usr/bin/env python3

import numpy as np
from math import *
from std_msgs.msg import String
from itertools import product
from sensor_msgs.msg import LaserScan
from Lidar import CONST_SECTOR_ANGLE

STATE_SPACE_IND_MAX = 19683 - 1 # 729 - 1 # 3 states ^ 6 sectors
STATE_SPACE_IND_MIN = 1 - 1
ACTIONS_IND_MAX = 40 # 14 directions * 3 rotation speeds - 1 stop action (-1,0)
ACTIONS_IND_MIN = 0

ANGLE_MAX = 90
ANGLE_MIN = -90
HORIZON_WIDTH = 180

T_MIN = 0.001

# Create actions
def createActions():
    # create array from -90 to 90 with step 6
    directions = np.arange(-90, 90 + 1, 15)
    directions = np.append(directions, -1)

    rotation_speeds = np.arange(-1, 1 + 1, 1)
    actions_ = np.array(list(set(product(directions, rotation_speeds))))

    #  find index of (-1,0) and delete it
    index = np.where((actions_ == [-1,0]).all(axis=1))[0][0]
    actions_ = np.delete(actions_, index, 0)

    # actions = np.array([0,1,2])

    return actions_

# Create state space for Q table
def createStateSpace():

    s1 = set((0,1,2))
    s2 = set((0,1,2))
    s3 = set((0,1,2))
    s4 = set((0,1,2))
    s5 = set((0,1,2))
    s6 = set((0,1,2))

    s7 = set((0,1,2))
    s8 = set((0,1,2))
    s9 = set((0,1,2))
    # s10 = set((0,1,2))
    # s11 = set((0,1,2))
    # s12 = set((0,1,2))


    state_space = set(product(s1,s2,s3,s4,s5,s6,s7,s8,s9))
    print('len(state_space) = ', len(state_space))
    # state_space = set(product(x1,x2,x3,x4))
    # state_space = set(product(sectors,states_for_sector))
    # return np.arange(0, 180//CONST_SECTOR_ANGLE*4, 1)
    return np.array(list(state_space))

def getStateIndex(state, state_space):
    state_index = np.where((state_space == state).all(axis=1))[0][0]
    return state_index

def getActionIndex(action, actions):
    action_index = np.where((actions == action).all(axis=1))[0][0]
    return action_index

# Create Q table, dim: n_states x n_actions
def createQTable(n_states, n_actions):
    #Q_table = np.random.uniform(low = -0.05, high = 0, size = (n_states,n_actions) )
    Q_table = np.zeros((n_states, n_actions))
    return Q_table

# Read Q table from path
def readQTable(path):
    Q_table = np.load(path)
    return Q_table

# Write Q table to path
def saveQTable(path, Q_table):
    np.save(path, Q_table)

# Select the best action a in state
def getBestAction(Q_table, state_ind, actions):
    if STATE_SPACE_IND_MIN <= state_ind <= STATE_SPACE_IND_MAX:
        status = 'getBestAction => OK'
        a_ind = np.argmax(Q_table[state_ind,:])
        a = actions[a_ind]
    else:
        status = 'getBestAction => INVALID STATE INDEX'
        a = getRandomAction(actions)

    return ( a, status )

# Select random action from actions
def getRandomAction(actions):
    n_actions = len(actions)
    a_ind = np.random.randint(n_actions)
    return actions[a_ind]

# Epsilog Greedy Exploration action chose
def epsiloGreedyExploration(Q_table, state_ind, actions, epsilon):
    if np.random.uniform() > epsilon and STATE_SPACE_IND_MIN <= state_ind <= STATE_SPACE_IND_MAX:
        status = 'epsiloGreedyExploration => OK'
        ( a, status_gba ) = getBestAction(Q_table, state_ind, actions)
        if status_gba == 'getBestAction => INVALID STATE INDEX':
            status = 'epsiloGreedyExploration => INVALID STATE INDEX'
    else:
        status = 'epsiloGreedyExploration => OK'
        a = getRandomAction(actions)

    return ( a, status )

# SoftMax Selection
def softMaxSelection(Q_table, state_ind, actions, T):
    if STATE_SPACE_IND_MIN <= state_ind <= STATE_SPACE_IND_MAX:
        status = 'softMaxSelection => OK'
        n_actions = len(actions)
        P = np.zeros(n_actions)

        # Boltzman distribution
        P = np.exp(Q_table[state_ind,:] / T) / np.sum(np.exp(Q_table[state_ind,:] / T))

        ( a, status_gba ) = getBestAction(Q_table, state_ind, actions)
        if status_gba == 'getBestAction => INVALID STATE INDEX':
            status = 'softMaxSelection => INVALID STATE INDEX'

    else:
        status = 'softMaxSelection => INVALID STATE INDEX'
        a = getRandomAction(actions)

    return ( a, status )

# Reward function for Q-learning - table
def getReward(action, prev_action, lidar, prev_lidar, crash, distance_to_goal, prev_distance_to_goal):
    if crash:
        terminal_state = True
        reward = -100
    elif distance_to_goal < 0.5:
        terminal_state = True
        reward = 10
    else:
        lidar_horizon = lidar[ANGLE_MAX:ANGLE_MIN]
        prev_lidar_horizon = prev_lidar[ANGLE_MAX:ANGLE_MIN]
        terminal_state = False
        # Reward from action taken = fowrad -> +0.2 , turn -> -0.1
        if action[0] == 0:
            r_action = +1.0
        elif action[0] == -1:
            r_action = -0.8
        elif action[0] > -30 and action[0] < 30:
            r_action = +0.4
        else:
            r_action = -0.1
        # Reward from crash distance to obstacle change
        W = np.linspace(0.9, 1.1, len(lidar_horizon) // 2)
        W = np.append(W, np.linspace(1.1, 0.9, len(lidar_horizon) // 2))
        distance_diff = np.sum( W * ( lidar_horizon - prev_lidar_horizon) )
        if  distance_diff > 0:
            r_obstacle = +0.4
        elif distance_diff == 0:
            r_obstacle = -0.1    
        else:
            r_obstacle = -0.4
        # Reward from turn left/right change
        if ( prev_action[0] < 0 and action[0] > 0 ) or ( prev_action[0] > 0 and action[0] < 0 ) or (prev_action[1] < 0 and action[1] > 0) or (prev_action[1] > 0 and action[1] < 0):
            r_change = -0.8
        else:
            r_change = 0.0

        if(prev_action[0] == action[0] and prev_action[1] == action[1]):
            r_consistency = 0.4
        else:
            r_consistency = 0.0

        if distance_to_goal < prev_distance_to_goal:
            r_distance = 0.4
        else:
            r_distance = -0.4

        # Cumulative reward
        reward = r_action + r_obstacle + r_change + r_consistency + r_distance - 0.1 # -0.1 is the cost of time

    return ( reward, terminal_state )


# Update Q-table values
def updateQTable(Q_table, state_ind, action_ind, reward, next_state_ind, alpha, gamma):
    if STATE_SPACE_IND_MIN <= state_ind <= STATE_SPACE_IND_MAX and STATE_SPACE_IND_MIN <= next_state_ind <= STATE_SPACE_IND_MAX:
        status = 'updateQTable => OK'
        Q_table[state_ind,action_ind] = ( 1 - alpha ) * Q_table[state_ind,action_ind] + alpha * ( reward + gamma * max(Q_table[next_state_ind,:]) )
    else:
        status = 'updateQTable => INVALID STATE INDEX'
    return ( Q_table, status )

# if __name__ == "__main__":
#     print('Q-learning - table')
#     print('------------------')
#     actions = createActions()
#     # print all np arrays
    
#     print('Actions', actions)
#     print("Action length", len(actions))
#     states = createStateSpace()
#     print('State space', states)
#     print("State space length", len(states))

    
#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from time import time
from time import sleep
from datetime import datetime
import matplotlib.pyplot as plt
import numpy as np

import sys
DATA_PATH =     '/home/user/dev_ws/src/omnibot/Data'
MODULES_PATH =  '/home/user/dev_ws/src/omnibot/scripts'
sys.path.insert(0, MODULES_PATH)

from Qlearning import *
from Lidar import *
from Control import *

node: Node = None
msgScan: LaserScan = None
msgScan_ready: bool = False

msgOdom: Odometry = None
msgOdom_ready: bool = False


# Episode parameters
MAX_EPISODES = 12000
MAX_STEPS_PER_EPISODE = 600
MIN_TIME_BETWEEN_ACTIONS = 0.05

# Learning parameters
ALPHA = 0.5
GAMMA = 0.9

T_INIT = 25
T_GRAD = 0.95
T_MIN = 0.001

EPSILON_INIT = 0.95 # 0.9
EPSILON_GRAD = 0.98 # 0.96
EPSILON_MIN = 0.05

# 1 - Softmax , 2 - Epsilon greedy
EXPLORATION_FUNCTION = 2

# Initial position
X_INIT = 0.015688741579619265
Y_INIT = 0.05600228905671296
THETA_INIT = 90.0


# Goal position
X_GOAL = 0.016976527428010722
Y_GOAL = 3.2001529447737216

GOAL_ARR =  [(0.12038379907608032, 1.5257762670516968), (1.4389824867248535, 1.753847360610962), (2.747143268585205, 3.2472877502441406), (2.219297409057617, 4.684449195861816)]
GOAL_ARR_IND = 0

RANDOM_INIT_POS = True

# Log file directory
LOG_FILE_DIR = DATA_PATH + '/Log_learning'

# Q table source file
Q_SOURCE_DIR =  LOG_FILE_DIR # ''

robot_current_x = 0.0
robot_current_y = 0.0
robot_current_theta = 0.0

def initRclpy():
    global node
    rclpy.init()
    node = rclpy.create_node('learning_node')
    return node.create_rate(10)


def initLearning():
    global actions, state_space, Q_table
    actions = createActions()
    state_space = createStateSpace()
    if Q_SOURCE_DIR != '':
        Q_table = readQTable(Q_SOURCE_DIR+'/Qtable.csv.npy')
    else:
        Q_table = createQTable(len(state_space),len(actions))
    print('Initial Q-table:')
    print(Q_table)
    print('Initial Q-table shape:')
    print(Q_table.shape)

def initParams():
    global T, EPSILON, alpha
    global ep_steps, ep_reward, episode, steps_per_episode, reward_per_episode, T_per_episode, EPSILON_per_episode
    global ep_reward_arr, reward_max_per_episode, reward_min_per_episode, reward_avg_per_episode
    global crash, t_ep, t_per_episode, t_sim_start, t_step
    global log_sim_info, log_sim_params
    global now_start, now_stop
    global robot_in_pos, first_action_taken

    # Init log files
    log_sim_info = open(LOG_FILE_DIR+'/LogInfo.txt','w+')
    log_sim_params = open(LOG_FILE_DIR+'/LogParams.txt','w+')

    # Learning parameters
    T = T_INIT
    EPSILON = EPSILON_INIT
    alpha = ALPHA

    # Episodes, steps, rewards
    ep_steps = 0
    ep_reward = 0
    episode = 1
    crash = False
    reward_max_per_episode = np.array([])
    reward_min_per_episode = np.array([])
    reward_avg_per_episode = np.array([])
    ep_reward_arr = np.array([])
    steps_per_episode = np.array([])
    reward_per_episode = np.array([])

    # initial position
    robot_in_pos = False
    first_action_taken = False

    # init time
    t_0 = node.get_clock().now()
    t_start = node.get_clock().now()

    # init timer
    while not (t_start > t_0):
        t_start = node.get_clock().now()

    t_ep = t_start
    t_sim_start = t_start
    t_step = t_start

    T_per_episode = np.array([])
    EPSILON_per_episode = np.array([])
    t_per_episode = np.array([])

    # Date
    now_start = datetime.now()
    dt_string_start = now_start.strftime("%d/%m/%Y %H:%M:%S")

    # Log date to files
    text = '\r\n' + 'SIMULATION START ==> ' + dt_string_start + '\r\n\r\n'
    print(text)
    log_sim_info.write(text)
    log_sim_params.write(text)

    # Log simulation parameters
    text = '\r\nSimulation parameters: \r\n'
    text = text + '--------------------------------------- \r\n'
    if RANDOM_INIT_POS:
        text = text + 'INITIAL POSITION = RANDOM \r\n'
    else:
        text = text + 'INITIAL POSITION = ( %.2f , %.2f , %.2f ) \r\n' % (X_INIT,Y_INIT,THETA_INIT)
    text = text + '--------------------------------------- \r\n'
    text = text + 'MAX_EPISODES = %d \r\n' % MAX_EPISODES
    text = text + 'MAX_STEPS_PER_EPISODE = %d \r\n' % MAX_STEPS_PER_EPISODE
    text = text + 'MIN_TIME_BETWEEN_ACTIONS = %.2f s \r\n' % MIN_TIME_BETWEEN_ACTIONS
    text = text + '--------------------------------------- \r\n'
    text = text + 'ALPHA = %.2f \r\n' % ALPHA
    text = text + 'GAMMA = %.2f \r\n' % GAMMA
    if EXPLORATION_FUNCTION == 1:
        text = text + 'T_INIT = %.3f \r\n' % T_INIT
        text = text + 'T_GRAD = %.3f \r\n' % T_GRAD
        text = text + 'T_MIN = %.3f \r\n' % T_MIN
    else:
        text = text + 'EPSILON_INIT = %.3f \r\n' % EPSILON_INIT
        text = text + 'EPSILON_GRAD = %.3f \r\n' % EPSILON_GRAD
        text = text + 'EPSILON_MIN = %.3f \r\n' % EPSILON_MIN
    text = text + '--------------------------------------- \r\n'
    text = text + 'MAX_LIDAR_DISTANCE = %.2f \r\n' % MAX_LIDAR_DISTANCE
    text = text + 'COLLISION_DISTANCE = %.2f \r\n' % COLLISION_DISTANCE
    text = text + 'ZONE_0_LENGTH = %.2f \r\n' % ZONE_0_LENGTH
    text = text + 'ZONE_1_LENGTH = %.2f \r\n' % ZONE_1_LENGTH
    text = text + '--------------------------------------- \r\n'
    text = text + 'CONST_LINEAR_SPEED_FORWARD = %.3f \r\n' % CONST_LINEAR_SPEED_FORWARD
    text = text + 'CONST_ANGULAR_SPEED_FORWARD = %.3f \r\n' % CONST_ANGULAR_SPEED_FORWARD
    text = text + 'CONST_LINEAR_SPEED_TURN = %.3f \r\n' % CONST_LINEAR_SPEED_TURN
    text = text + 'CONST_ANGULAR_SPEED_TURN = %.3f \r\n' % CONST_ANGULAR_SPEED_TURN
    log_sim_params.write(text)

def laserCallback(msg: LaserScan):
    # print("laserCallback")
    global msgScan, msgScan_ready
    msgScan = msg
    msgScan_ready = True

def odomCallback(msg: Odometry):
    global msgOdom, msgOdom_ready
    print("odomCallback")
    msgOdom = msg
    msgOdom_ready = True

def robotDescCallback(msg: ModelStates):
    # print("robotDescCallback")
    global robot_current_x, robot_current_y, robot_current_theta
    # find robot index in msg
    robot_index = msg.name.index('omnibot')
    # get robot pose
    robot_current_x = msg.pose[robot_index].position.x
    robot_current_y = msg.pose[robot_index].position.y
    robot_current_theta = msg.pose[robot_index].orientation.z


    



def main():
    try:
        global msgScan_ready
        global actions, state_space, Q_table
        global T, EPSILON, alpha
        global ep_steps, ep_reward, episode, steps_per_episode, reward_per_episode, T_per_episode, EPSILON_per_episode
        global ep_reward_arr, reward_max_per_episode, reward_min_per_episode, reward_avg_per_episode
        global crash, t_ep, t_per_episode, t_sim_start, t_step
        global log_sim_info, log_sim_params
        global now_start, now_stop
        global robot_in_pos, first_action_taken
        global robot_current_x, robot_current_y, robot_current_theta
        global GOAL_ARR_IND, GOAL_ARR


        # rospy.init_node('learning_node', anonymous = False)
        # rate = rospy.Rate(10)
        
        rate = initRclpy()

        odomNode = rclpy.create_node('odom_node')

        setPosPub = node.create_publisher(ModelState, '/gazebo/set_entity_state', 10)
        velPub = node.create_publisher(Twist, '/cmd_vel', 10)
        laserSub = node.create_subscription(LaserScan,'/scan', laserCallback, 10)
        robotDescSub = node.create_subscription(ModelStates, '/model_states', robotDescCallback, 10)
        setModelStateClient = node.create_client(SetEntityState, '/set_entity_state')
        getGazeboModelStateClient = node.create_client(GetEntityState, '/get_entity_state')

        initLearning()
        initParams()

        # main loop
        while rclpy.ok():
            msgScan_ready = False
            # time how long it takes to get the laser data
            now_start_laser = node.get_clock().now()
            while not msgScan_ready:
                rclpy.spin_once(node)
                pass
            now_stop_laser = node.get_clock().now()
            if (now_stop_laser - now_start_laser).nanoseconds / 1e9 > 0.1:
                node.get_logger().warn("time to get laser data: %.2f s" % ((now_stop_laser - now_start_laser).nanoseconds / 1e9))


            # Secure the minimum time interval between 2 actions
            step_time = (node.get_clock().now() - t_step).nanoseconds / 1e9
            if step_time > MIN_TIME_BETWEEN_ACTIONS:

                t_step = node.get_clock().now()
                if step_time > 2:
                    text = '\r\nTOO BIG STEP TIME: %.2f s' % step_time
                    node.get_logger().warn(text)
                    log_sim_info.write(text+'\r\n')

                # End of Learning
                if episode > MAX_EPISODES:
                    # simulation time
                    sim_time = (node.get_clock().now() - t_sim_start).nanoseconds / 1e9
                    sim_time_h = sim_time // 3600
                    sim_time_m = ( sim_time - sim_time_h * 3600 ) // 60
                    sim_time_s = sim_time - sim_time_h * 3600 - sim_time_m * 60

                    # real time
                    now_stop = datetime.now()
                    dt_string_stop = now_stop.strftime("%d/%m/%Y %H:%M:%S")
                    real_time_delta = (now_stop - now_start).total_seconds()
                    real_time_h = real_time_delta // 3600
                    real_time_m = ( real_time_delta - real_time_h * 3600 ) // 60
                    real_time_s = real_time_delta - real_time_h * 3600 - real_time_m * 60

                    # Log learning session info to file
                    text = '--------------------------------------- \r\n\r\n'
                    text = text + 'MAX EPISODES REACHED(%d), LEARNING FINISHED ==> ' % MAX_EPISODES + dt_string_stop + '\r\n'
                    text = text + 'Simulation time: %d:%d:%d  h/m/s \r\n' % (sim_time_h, sim_time_m, sim_time_s)
                    text = text + 'Real time: %d:%d:%d  h/m/s \r\n' % (real_time_h, real_time_m, real_time_s)
                    print(text)
                    log_sim_info.write('\r\n'+text+'\r\n')
                    log_sim_params.write(text+'\r\n')

                    # Log data to file
                    saveQTable(LOG_FILE_DIR+'/Qtable.csv', Q_table)
                    np.savetxt(LOG_FILE_DIR+'/StateSpace.csv', state_space, '%d')
                    np.savetxt(LOG_FILE_DIR+'/steps_per_episode.csv', steps_per_episode, delimiter = ' , ')
                    np.savetxt(LOG_FILE_DIR+'/reward_per_episode.csv', reward_per_episode, delimiter = ' , ')
                    np.savetxt(LOG_FILE_DIR+'/T_per_episode.csv', T_per_episode, delimiter = ' , ')
                    np.savetxt(LOG_FILE_DIR+'/EPSILON_per_episode.csv', EPSILON_per_episode, delimiter = ' , ')
                    np.savetxt(LOG_FILE_DIR+'/reward_min_per_episode.csv', reward_min_per_episode, delimiter = ' , ')
                    np.savetxt(LOG_FILE_DIR+'/reward_max_per_episode.csv', reward_max_per_episode, delimiter = ' , ')
                    np.savetxt(LOG_FILE_DIR+'/reward_avg_per_episode.csv', reward_avg_per_episode, delimiter = ' , ')
                    np.savetxt(LOG_FILE_DIR+'/t_per_episode.csv', t_per_episode, delimiter = ' , ')

                    # Close files and shut down node
                    log_sim_info.close()
                    log_sim_params.close()
                    node.destroy_node()
                    rclpy.shutdown()
                else:
                    ep_time = (node.get_clock().now() - t_ep).nanoseconds / 1e9
                    # End of en Episode
                    if crash or ep_steps >= MAX_STEPS_PER_EPISODE or GOAL_ARR_IND == len(GOAL_ARR):
                        robotStop(velPub)

                        if crash:
                            # get crash position
                            ( x_crash , y_crash , theta_crash) = getGazeboModelPossitoin(node, MODEL_NAME)

                        t_ep = node.get_clock().now()
                        if(ep_reward_arr.size > 0):
                            reward_min = np.min(ep_reward_arr)
                            reward_max = np.max(ep_reward_arr)
                            reward_avg = np.mean(ep_reward_arr)
                        else:
                            node.get_logger().warn('ep_reward_arr is empty')
                            reward_min = 0
                            reward_max = 0
                            reward_avg = 0
                        now = datetime.now()
                        dt_string = now.strftime("%d/%m/%Y %H:%M:%S")

                        text = '---------------------------------------\r\n'
                        if crash:
                            text = text + '\r\nEpisode %d ==> CRASH {%.2f,%.2f,%.2f}    ' % (episode, x_crash, y_crash, theta_crash) + dt_string
                        elif ep_steps >= MAX_STEPS_PER_EPISODE:
                            text = text + '\r\nEpisode %d ==> MAX STEPS PER EPISODE REACHED {%d}    ' % (episode, MAX_STEPS_PER_EPISODE) + dt_string
                        elif GOAL_ARR_IND == len(GOAL_ARR):
                            text = text + '\r\nEpisode %d ==> GOAL REACHED    ' % (episode) + dt_string
                        else:
                            text = text + '\r\nEpisode %d ==> UNKNOWN TERMINAL CASE    ' % episode + dt_string
                        text = text + '\r\nepisode time: %.2f s (avg step: %.2f s) \r\n' % (ep_time, ep_time / ep_steps)
                        text = text + 'episode steps: %d \r\n' % ep_steps
                        text = text + 'episode reward: %.2f \r\n' % ep_reward
                        text = text + 'episode max | avg | min reward: %.2f | %.2f | %.2f \r\n' % (reward_max, reward_avg, reward_min)
                        if EXPLORATION_FUNCTION == 1:
                            text = text + 'T = %f \r\n' % T
                        else:
                            text = text + 'EPSILON = %f \r\n' % EPSILON
                        print(text)
                        log_sim_info.write('\r\n'+text)

                        steps_per_episode = np.append(steps_per_episode, ep_steps)
                        reward_per_episode = np.append(reward_per_episode, ep_reward)
                        T_per_episode = np.append(T_per_episode, T)
                        EPSILON_per_episode = np.append(EPSILON_per_episode, EPSILON)
                        t_per_episode = np.append(t_per_episode, ep_time)
                        reward_min_per_episode = np.append(reward_min_per_episode, reward_min)
                        reward_max_per_episode = np.append(reward_max_per_episode, reward_max)
                        reward_avg_per_episode = np.append(reward_avg_per_episode, reward_avg)
                        ep_reward_arr = np.array([])
                        ep_steps = 0
                        ep_reward = 0
                        crash = False
                        robot_in_pos = False
                        first_action_taken = False
                        GOAL_ARR_IND = 0
                        if T > T_MIN:
                            T = T_GRAD * T
                        if EPSILON > EPSILON_MIN:
                            EPSILON = EPSILON_GRAD * EPSILON
                        episode = episode + 1
                    else:
                        
                        ep_steps = ep_steps + 1
                        # Initial position
                        if not robot_in_pos:
                            robotStop(velPub)
                            ep_steps = ep_steps - 1
                            first_action_taken = False
                            # init pos
                            if RANDOM_INIT_POS:
                                ( x_init , y_init , theta_init ) = robotSetRandomPos(setModelStateClient, node)
                            else:
                                ( x_init , y_init , theta_init ) = robotSetPos(setModelStateClient, X_INIT, Y_INIT, THETA_INIT,node)

                           
                            
                            x , y , theta =  x_init , y_init , theta_init 
                            node.get_logger().info('Robot position: {0} {1} {2}'.format(x, y, theta))
                            robot_in_pos = True

                        # First acion
                        elif not first_action_taken:
                            ( lidar, angles ) = lidarScan(msgScan)
                            scan_reduction = lidarReduction(lidar)
                            state_ind = getStateIndex(state_space, scan_reduction)
                            crash = checkCrash(lidar)

                            if EXPLORATION_FUNCTION == 1 :
                                ( action, status_strat ) = softMaxSelection(Q_table, state_ind, actions, T)
                            else:
                                ( action, status_strat ) = epsiloGreedyExploration(Q_table, state_ind, actions, T)

                            node.get_logger().info('Action: {0}'.format(action))

                            status_rda = robotDoAction(velPub, action)


                            prev_lidar = lidar
                            prev_action = action
                            prev_state_ind = state_ind
                            distance_to_goal = getDistanceToGoal(robot_current_x, robot_current_y, GOAL_ARR[GOAL_ARR_IND][0], GOAL_ARR[GOAL_ARR_IND][1])
                            

                            first_action_taken = True

                            if not (status_strat == 'softMaxSelection => OK' or status_strat == 'epsiloGreedyExploration => OK'):
                                print('\r\n', status_strat, '\r\n')
                                log_sim_info.write('\r\n'+status_strat+'\r\n')

                            if not status_rda == 'robotDoAction => OK':
                                print('\r\n', status_rda, '\r\n')
                                log_sim_info.write('\r\n'+status_rda+'\r\n')


                        # Rest of the algorithm
                        else:

                            temp_time = node.get_clock().now()

                            ( lidar, angles ) = lidarScan(msgScan)
                            scan_reduction = lidarReduction(lidar)
                            state_ind = getStateIndex(state_space, scan_reduction)
                            prev_distance_to_goal = distance_to_goal
                            distance_to_goal = getDistanceToGoal(robot_current_x, robot_current_y, GOAL_ARR[GOAL_ARR_IND][0], GOAL_ARR[GOAL_ARR_IND][1])
                            # time to reset variables
                            time_took = (node.get_clock().now() - temp_time).nanoseconds / 1e9
                            if time_took > 0.1:
                                node.get_logger().warn('time to get position: {0}'.format(time_took))


                            temp_time = node.get_clock().now()
                            crash = checkCrash(lidar)
                            time_took = (node.get_clock().now() - temp_time).nanoseconds / 1e9
                            if time_took > 0.1:
                                node.get_logger().warn('time to check crash: {0}'.format(time_took))


                            temp_time = node.get_clock().now()
                            ( reward, terminal_state ) = getReward(action, prev_action, lidar, prev_lidar, crash, distance_to_goal, prev_distance_to_goal)
                            time_took = (node.get_clock().now() - temp_time).nanoseconds / 1e9
                            if time_took > 0.1:
                                node.get_logger().warn('time to get reward: {0}'.format(time_took))
                                

                            temp_time = node.get_clock().now()
                            action_indx = getActionIndex(actions, action)
                            time_took = (node.get_clock().now() - temp_time).nanoseconds / 1e9
                            if time_took > 0.1:
                                node.get_logger().warn('time to get action index: {0}'.format(time_took))



                            temp_time = node.get_clock().now()
                            ( Q_table, status_uqt ) = updateQTable(Q_table, prev_state_ind, action_indx, reward, state_ind, alpha, GAMMA)
                            time_took = (node.get_clock().now() - temp_time).nanoseconds / 1e9
                            if time_took > 0.1:
                                node.get_logger().warn('time to update Q table: {0}'.format(time_took))

                            temp_time = node.get_clock().now()
                            if EXPLORATION_FUNCTION == 1:
                                ( action, status_strat ) = softMaxSelection(Q_table, state_ind, actions, T)
                            else:
                                ( action, status_strat ) = epsiloGreedyExploration(Q_table, state_ind, actions, T)
                            time_took = (node.get_clock().now() - temp_time).nanoseconds / 1e9
                            if time_took > 0.1:
                                node.get_logger().warn('time to get action: {0}'.format(time_took))
                            
                            if terminal_state and distance_to_goal < 0.5:
                                node.get_logger().info('Reached goal #{0}'.format(GOAL_ARR_IND))
                                log_sim_info.write('Reached goal')
                                GOAL_ARR_IND += 1



                            temp_time = node.get_clock().now()
                            status_rda = robotDoAction(velPub, action)
                            time_took = (node.get_clock().now() - temp_time).nanoseconds / 1e9
                            if time_took > 0.1:
                                node.get_logger().warn('time to do action: {0}'.format(time_took))

                            if action[0] == -1:
                                node.get_logger().info('Action: {0}'.format(action))

                            if not status_uqt == 'updateQTable => OK':
                                print('\r\n', status_uqt, '\r\n')
                                log_sim_info.write('\r\n'+status_uqt+'\r\n')
                            if not (status_strat == 'softMaxSelection => OK' or status_strat == 'epsiloGreedyExploration => OK'):
                                print('\r\n', status_strat, '\r\n')
                                log_sim_info.write('\r\n'+status_strat+'\r\n')
                            if not status_rda == 'robotDoAction => OK':
                                print('\r\n', status_rda, '\r\n')
                                log_sim_info.write('\r\n'+status_rda+'\r\n')

                            ep_reward = ep_reward + reward
                            ep_reward_arr = np.append(ep_reward_arr, reward)
                            prev_lidar = lidar
                            prev_action = action
                            prev_state_ind = state_ind

    except Exception as e:
        print(e)
        robotStop(velPub)
        print('Simulation terminated!')
        pass


if __name__ == '__main__':
    main()
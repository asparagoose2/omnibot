#! /usr/bin/env python3

import numpy as np
from math import *
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import rclpy

# Configuration
CONST_SECTOR_ANGLE = 20 # [degrees]
 
COLLISION_DISTANCE = 0.20 # LaserScan.range_min = 0.1199999
NEARBY_DISTANCE = 0.25
FAR_DISTANCE = 0.45
MAX_LIDAR_DISTANCE = 0.65

# COLLISION_DISTANCE_ENUM = 0
NEARBY_DISTANCE_ENUM = 0
FAR_DISTANCE_ENUM = 1
MAX_LIDAR_DISTANCE_ENUM = 2


ZONE_0_LENGTH = 0.4
ZONE_1_LENGTH = 0.7

ANGLE_MAX = 360 - 1
ANGLE_MIN = 1 - 1
HORIZON_WIDTH = 75

# Convert LasecScan msg to array
def lidarScan(msgScan: LaserScan):
    distances = np.array([])
    angles = np.array([])
    # print("*"*50)
    # print("Lidar scan:")
    # print("*"*50)
    # print("Time increment: ", msgScan.time_increment)
    # print("Number of ranges: ", len(msgScan.ranges))
    # print("Angle increment: ", degrees(msgScan.angle_increment))
    # print("Angle min: ", degrees(msgScan.angle_min))
    # print("Angle max: ", degrees(msgScan.angle_max))
    # print("Range min: ", msgScan.range_min)
    # print("Range max: ", msgScan.range_max)
    # print("*"*50)


    for i in range(len(msgScan.ranges)):
        angle = degrees(i * msgScan.angle_increment)
        if ( msgScan.ranges[i] > MAX_LIDAR_DISTANCE ):
            distance = MAX_LIDAR_DISTANCE
        elif ( msgScan.ranges[i] < msgScan.range_min ):
            distance = msgScan.range_min
            # For real robot - protection
            if msgScan.ranges[i] < 0.01:
                distance = MAX_LIDAR_DISTANCE
        else:
            distance = msgScan.ranges[i]

        distances = np.append(distances, distance)
        angles = np.append(angles, angle)

    # distances in [m], angles in [degrees]
    return ( distances, angles )

def lidarReduction(distances: list, angles: list):

    # trim the last and first fourth of the scan
    [distances, angles] = zip(*[(d, angles[i]) for i, d in enumerate(distances) if angles[i] > 270 or angles[i] < 90])
    # print(list(zip(distances, angles)))
    # find min range in each sector
    lidar = np.array([])
    # print("Lidar reduction:")
    # print("*"*50)
    # print("Number of ranges: ", len(scan))
    # print("Scan: ", scan)
    # from -90 to 90
    relevant_distances = [
        [distance for distance, angle in zip(distances, angles) if angle > 270 and angle <= 290],
        [distance for distance, angle in zip(distances, angles) if angle > 290 and angle <= 310],
        [distance for distance, angle in zip(distances, angles) if angle > 310 and angle <= 330],
        [distance for distance, angle in zip(distances, angles) if angle > 330 and angle <= 350],
        [distance for distance, angle in zip(distances, angles) if angle > 350 or angle <= 10],
        [distance for distance, angle in zip(distances, angles) if angle > 10 and angle <= 30],
        [distance for distance, angle in zip(distances, angles) if angle > 30 and angle <= 50],
        [distance for distance, angle in zip(distances, angles) if angle > 50 and angle <= 70],
        [distance for distance, angle in zip(distances, angles) if angle > 70 and angle <= 90],
    ]
    # print(relevant_distances)
    relevant_distances = [min(asd) for asd in relevant_distances]
    for min_range in relevant_distances[::-1]:
        # start_angle = (i*CONST_SECTOR_ANGLE + 90) % 360
        # end_angle = ((i+1)*CONST_SECTOR_ANGLE + 90) % 360
        # start_angle = i*CONST_SECTOR_ANGLE - 90
        # start_angle = 360 + start_angle if start_angle < 0 else start_angle
        # start_angle = 0 if start_angle == 350 else start_angle
        # end_angle = (i+1)*CONST_SECTOR_ANGLE - 90
        # end_angle = 360 + end_angle if end_angle < 0 else end_angle
        # end_angle = 360 if end_angle == 0 else end_angle
        # relevant_distances = [distance for distance, angle in zip(distances, angles) if angle > start_angle and angle < end_angle]
        # print(list(zip(distances, angles)))
        # print(f'{start_angle}-{end_angle}: {len(relevant_distances)}')

        # min_range = min(distances[i*CONST_SECTOR_ANGLE:(i+1)*CONST_SECTOR_ANGLE])
        # if (len(relevant_distances) == 0):
        #     lidar = np.append(lidar, MAX_LIDAR_DISTANCE)
        #     continue
        # min_range = min(relevant_distances)
        if min_range == MAX_LIDAR_DISTANCE:
            min_range = MAX_LIDAR_DISTANCE_ENUM
        elif min_range <= MAX_LIDAR_DISTANCE and min_range > FAR_DISTANCE:
            min_range = FAR_DISTANCE_ENUM
        elif min_range <= FAR_DISTANCE and min_range > NEARBY_DISTANCE:
            min_range = NEARBY_DISTANCE_ENUM
        elif min_range <= NEARBY_DISTANCE :
            min_range = NEARBY_DISTANCE_ENUM
        
        # print(f'{start_angle}-{end_angle}: {min_range}')

        lidar = np.append(lidar, min_range)

    return lidar


# Check - crash
def checkCrash(scan: list):
    if np.min(scan) <= COLLISION_DISTANCE:
        # print("<----  CRASH  ---->")
        # print("min = ", np.min(scan))
        # print("scan = ", scan)
        # print("<----------------->")

        return True
    else:
        return False
    lidar_horizon = np.concatenate((lidar[(ANGLE_MIN + HORIZON_WIDTH):(ANGLE_MIN):-1],lidar[(ANGLE_MAX):(ANGLE_MAX - HORIZON_WIDTH):-1]))
    W = np.linspace(1.2, 1, len(lidar_horizon) // 2)
    W = np.append(W, np.linspace(1, 1.2, len(lidar_horizon) // 2))
    if np.min( W * lidar_horizon ) < COLLISION_DISTANCE:
        return True
    else:
        return False

# Check - object nearby
def checkObjectNearby(lidar):
    lidar_horizon = np.concatenate((lidar[(ANGLE_MIN + HORIZON_WIDTH):(ANGLE_MIN):-1],lidar[(ANGLE_MAX):(ANGLE_MAX - HORIZON_WIDTH):-1]))
    W = np.linspace(1.4, 1, len(lidar_horizon) // 2)
    W = np.append(W, np.linspace(1, 1.4, len(lidar_horizon) // 2))
    if np.min( W * lidar_horizon ) < NEARBY_DISTANCE:
        return True
    else:
        return False

# Check - goal near
def checkGoalNear(x, y, x_goal, y_goal):
    ro = sqrt( pow( ( x_goal - x ) , 2 ) + pow( ( y_goal - y ) , 2) )
    if ro < 0.3:
        return True
    else:
        return False


# # Callback function
# def callback(msgScan: LaserScan):
#     distances, angles = lidarScan(msgScan)
#     # print type of distances
#     print("type of distances: ", type(distances))
#     crash = checkCrash(distances)
#     object_nearby = checkObjectNearby(distances)
#     print("range min:{}, max:{}".format(msgScan.range_min, msgScan.range_max))
#     red_distances = lidarReduction(distances)
#     print("red_distances = ", red_distances)
#     print("crash = ", crash)


# if __name__ == '__main__':
#     rclpy.init()
#     LaserNode = rclpy.create_node('LaserNode')
#     laserSub = LaserNode.create_subscription(LaserScan,'/scan', callback, 10)
#     rclpy.spin_once(LaserNode)


#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, PoseArray, PoseWithCovariance
import numpy as np
from tf.transformations import euler_from_quaternion
from utils import *

class Occupancy_Slam(object):
    def __init__(self):
        self.pose_in_occupancy = Pose()
        #self.landmarks = dict()
        #self.X_hat = np.zeros((3, 1))  # Robot state [x, y, theta]
        #self.P_hat = np.eye(3)  # Covariance matrix
        #self.Q = np.eye(3)  # Process noise covariance
        #self.R = np.eye(2)  # Measurement noise covariance
        
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid,callback=self.map_call ,queue_size=10)
        self.pose_sub = rospy.Subscriber('/robot_pose', Pose, callback=self.pose_call, queue_size=10)
        
    def map_call(self,map):
        size = len(map.data)
        free_idx = [i for i in range(size) if map.data[i] == 0]
        rospy.loginfo(free_idx)
        rospy.loginfo(size)
        rospy.loginfo(set(map.data))

    def pose_call(self, pose):
        rospy.loginfo(pose)

    


if __name__ == "__main__":
    rospy.init_node('hear_map')
    slam = Occupancy_Slam()
    rospy.spin()

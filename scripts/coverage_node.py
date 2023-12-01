#!/usr/bin/env python3

#from typing import Any
import rospy
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData, Path
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, PoseArray, PoseWithCovariance,Twist, PoseStamped
import numpy as np
from tf.transformations import euler_from_quaternion
from utils import *
import time
from copy import deepcopy

class Coverage_node(object):

    MOVE_FORWARD = 0
    TURN_RIGHT = 1
    TURN_LEFT = 2
    MOVE_BACKWARD = 3
    STOP = 4

    def __init__(self) -> None:

        self.Twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=100)
        self.path_pub = rospy.Publisher('/path', Path, queue_size=100)
        self.visited_pub = rospy.Publisher('/visted', OccupancyGrid, queue_size=100)
        #self.position_sub = rospy.Subscriber('/odom', Odometry, queue_size=10, callback=self.OdomCallBack)
        self.laser_sub = rospy.Subscriber('/base_scan', LaserScan, queue_size=100, callback = self.LaserCallBack)
        self.Map_pose_sub = rospy.Subscriber('/robot_pose', PoseStamped, queue_size=100, callback=self.PoseCallBack)
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, queue_size=100, callback=self.mapCallBacK)
        self.Path = Path()
        self.Path.header.frame_id = 'map'
        self.Visited_map = OccupancyGrid()
        self.Visited_init = False

        self.wall_following = False
        self.wall_distance_threshold = 0.5  # Adjust as needed
        self.linear_speed = 0.2  # Adjust linear speed as needed
        self.angular_speed = 0.5  # Adjust angular speed as needed

        self.prev_move_type = None
        self.wall_distance_min = 0.3  # min
        self.wall_distance_max = 0.6  # max_distance with the wall


        # def OdomCallBack(self, odom):
        #     pass
    

    def _1to2D(self, indx, map_col):
        i = indx % map_col
        j = int(indx / map_col)
        return i, j

    def mapCallBacK(self, map):
        if self.Visited_init == False:
            
            self.Visited_map.info.width = map.info.width
            self.Visited_map.info.height = map.info.height
            self.Visited_map.info.resolution = map.info.resolution
            self.Visited_map.info.origin.position.x = map.info.origin.position.x
            self.Visited_map.info.origin.position.y = map.info.origin.position.y
            self.visited_pub.data = [-1] * (self.Visited_map.info.width * self.Visited_map.info.height) 
            #self.map_pub.publish(OccupancyGrid)


    def front_free(self, map, robot_x, robot_y, theta):
        
        grid_size = self.Visited_map.info.resolution
        num_cells_to_check = 8  # the threhold distance in front

        
        dx = round(np.cos(theta) * grid_size)
        dy = round(np.sin(theta) * grid_size)

        for i in range(1, num_cells_to_check + 1):
            # 计算前方网格的位置
            x = int(robot_x + i * dx)
            y = int(robot_y + i * dy)

            # 检查位置是否在地图范围内
            if x < 0 or y < 0 or x >= map.shape[0] or y >= map.shape[1]:
                return False
            
    
            if map[x][y] != 0:  # 假设0为free，非0为障碍或访问过的网格
                return False

        return True

    def left_free(self, map, robot_x, robot_y, theta):
        grid_size = self.Visited_map.info.resolution
        num_cells_to_check = 8  # Adjust as needed

        dx = round(np.cos(theta - np.pi / 2) * grid_size)
        dy = round(np.sin(theta - np.pi / 2) * grid_size)

        for i in range(1, num_cells_to_check + 1):
            # Calculate the position of the left-side grid
            x = int(robot_x + i * dx)
            y = int(robot_y + i * dy)

            # Check if the position is within the map's range
            if x < 0 or y < 0 or x >= map.shape[0] or y >= map.shape[1]:
                return False
            
            # Check if the grid is occupied or visited
            if map[x][y] != 0:  # Assuming 0 is free, non-zero values represent obstacles or visited grids
                return False

        return True

    def right_free(self, map, robot_x, robot_y, theta):
        grid_size = self.Visited_map.info.resolution
        num_cells_to_check = 8  # Adjust as needed

        dx = round(np.cos(theta + np.pi / 2) * grid_size)
        dy = round(np.sin(theta + np.pi / 2) * grid_size)

        for i in range(1, num_cells_to_check + 1):
            # Calculate the position of the right-side grid
            x = int(robot_x + i * dx)
            y = int(robot_y + i * dy)

            # Check if the position is within the map's range
            if x < 0 or y < 0 or x >= map.shape[0] or y >= map.shape[1]:
                return False
            
            # Check if the grid is occupied or visited
            if map[x][y] != 0:  # Assuming 0 is free, non-zero values represent obstacles or visited grids
                return False

        return True

        

    def PoseCallBack(self, posestampe):
        self.Path.header.stamp = rospy.Time.now()
        self.Path.header.frame_id = 'map'
        self.Path.poses.append((posestampe))
        self.path_pub.publish(self.Path)

    def LaserCallBack(self, laser):
        self.publish_twist(1)

    def publish_twist(self, move_type):
        twist_msg = Twist()

        if move_type == self.MOVE_FORWARD:
            twist_msg.linear.x = self.linear_speed
        elif move_type == self.TURN_RIGHT:
            twist_msg.angular.z = -self.angular_speed
        elif move_type == self.TURN_LEFT:
            twist_msg.angular.z = self.angular_speed
        else:
            twist_msg.linear.x = 0
            twist_msg.angular.z = 0
        #     
        # # Add STOP and MOVE_BACKWARD logic if needed
        #     pass

        self.Twist_pub.publish(twist_msg)


if __name__ == '__main__':
    rospy.init_node('coverage')
    node = Coverage_node()
    rospy.spin()
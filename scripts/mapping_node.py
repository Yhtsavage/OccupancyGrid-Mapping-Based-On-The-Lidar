#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, PoseArray, PoseWithCovariance
import numpy as np
from tf.transformations import euler_from_quaternion
from utils import *
from copy import deepcopy

class Occupancy_Slam(object):
    def __init__(self):
        self.pose_in_world = PoseStamped()
        self.pose_in_occupancy = PoseStamped()
        #self.pose_in_occupancy.header.frame_id = 'map'
        self.pose_in_occupancy.header.frame_id = 'map'
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback,queue_size=100)
        self.scan_sub = rospy.Subscriber("/base_scan", LaserScan, self.scan_callback, queue_size=100)
        self.map_pub = rospy.Publisher("/map", OccupancyGrid, queue_size=100)
        self.pose_pub = rospy.Publisher("/robot_pose", PoseStamped, queue_size=100)
        self.map_init()
        self.min_angle = -1.5707963705062866
        self.max_angle = 1.5707963705062866
        self.process_laser = False
        self.waiting_laser = False
        self.previous_pose = PoseStamped()
        self.pose_update = True
        self.init_previous = True
    
    def map_init(self): # the original is 30 30 zero central
        self.map_width = 800 # 
        self.map_height = 800 # 
        self.map_resolution = 0.05  # 
        self.occupancy_grid = OccupancyGrid()
        self.occupancy_grid.header.frame_id = 'map'
        self.occupancy_grid.info.width = self.map_width
        self.occupancy_grid.info.height = self.map_height
        self.occupancy_grid.info.resolution = self.map_resolution
        self.occupancy_grid.info.origin.position.x = 0
        self.occupancy_grid.info.origin.position.y = 0
        self.occupancy_grid.data = [-1] * (self.map_width * self.map_height) 
        self.map_pub.publish(OccupancyGrid)
        #rospy.loginfo(OccupancyGrid.data)
        rospy.loginfo('map has initialized')

    

    def process_sensor_data(self,laser_data):
        #self.pose_in_occupancy = self.pose_convert(self.pose_in_world)

        ranges = laser_data.ranges
        reading_step = (len(ranges) - 1) / (500 - 1) # 20 of reading data
        laser_remain = [ranges[i]/self.map_resolution for i in range(0, len(ranges), int(reading_step))]

        angles = np.linspace(self.min_angle, self.max_angle, len(laser_remain)) #from -90 - 90 and len 500
        
        laser_data_max = laser_data.range_max/self.map_resolution
        
        dx = laser_remain * np.cos(angles)
        
        #dx_int = [int(i+1) for i in dx]# det x of each point detected by laser, relative respect to the robot
        dy = laser_remain * np.sin(angles)
        robot_orientation = self.pose_in_occupancy.pose.orientation
        _, _, robot_theta = euler_from_quaternion([robot_orientation.x, robot_orientation.y, robot_orientation.z, robot_orientation.w])
        robot_x = self.pose_in_occupancy.pose.position.x
        robot_y = self.pose_in_occupancy.pose.position.y
        # update the map
        for n, (x, y, rng) in enumerate(zip(dx, dy, laser_remain)):
            steps = int(abs(x))+1
            direction = np.sign(x)
            angle = angles[n]
            # if steps < 1:  # Occupying the cell directly if dx is smaller than 1
            #     map_x = int(robot_x + x * np.cos(robot_theta) - x *  np.tan(angle) * np.sin(robot_theta))
            #     map_y = int(robot_x + x * np.cos(robot_theta) + x *  np.tan(angle) * np.sin(robot_theta))
            #     self.occupancy_grid.data[map_x * self.map_width + map_y] = 100 #if dis < laser_data_max else 0
            # else:
            for i in range(steps):
                
                j = i * np.tan(angle)
                dis = np.sqrt(i**2 + j**2)
                map_x = int(robot_x + i * direction * np.cos(robot_theta) - j * np.sin(robot_theta))
                map_y = int(robot_y + i * direction * np.sin(robot_theta) + j * np.cos(robot_theta))
                #dis = distance(pose1=[map_x, map_y], pose2=[robot_x, robot_y])
                
                if i != steps - 1:  # Not the last iteration
                    if dis < rng: #distance is big
                        self.occupancy_grid.data[map_y * self.map_width + map_x] = 0
                else:
                    if i == steps - 1:  # Check if it's the last iteration
                        if rng < laser_data_max:
                            self.occupancy_grid.data[map_y * self.map_width + map_x] = 100
                        else:
                            self.occupancy_grid.data[map_y * self.map_width + map_x] = 0
                # if 0 <= map_x < self.map_width and 0 <= map_y < self.map_height:
                #     if rng == laser_data_max:
                #         self.occupancy_grid.data[map_y * self.map_width + map_x] = 0
                #     else:
                #         self.occupancy_grid.data[map_y * self.map_width + map_x] = 1

    def odom_callback(self, odom):
        # Publish robot pose
        #if self.init_previous == 
        if self.waiting_laser == False:
            self.pose_convert(odom) #update the pose_in_occ
            # if self.init_previous == True:
            #     self.previous_pose.pose = self.pose_in_occupancy.pose
            #     self.init_previous = False
            self.process_laser = True
            if self.init_previous == True:
                self.previous_pose.pose = deepcopy(self.pose_in_occupancy.pose)
                self.pose_update = True
                self.init_previous = False
            else:
                if self.previous_pose.pose == self.pose_in_occupancy.pose: # the first laser should able to mapping
                    self.pose_update = False
                else:
                    self.pose_update = True
                    self.previous_pose.pose = deepcopy(self.pose_in_occupancy.pose)

            pub_pose = PoseStamped()
            pub_pose.header.frame_id = 'map'
            pub_pose.header.stamp = rospy.Time.now()
            #self.pose_in_occupancy.header.stamp = rospy.Time.now()
            pub_pose.pose = deepcopy(self.pose_in_occupancy.pose)
            self.pose_pub.publish(pub_pose)
            self.waiting_laser = True

        #self.process_laser = True
        # od = Odometry()
        # od.pose.pose.position.x


    # def odom_callback(self, odom):
    #     # Publish robot pose
    #     self.pose_in_world.pose.position  = odom.pose.pose.position
    #     self.pose_in_world.pose.orientation = odom.pose.pose.orientation

    def scan_callback(self, laser):
        if self.process_laser == True and self.pose_update == True:
            self.process_sensor_data(laser)
            # Publish map data
            # Fill in map_msg data based on your map representation
            self.map_pub.publish(self.occupancy_grid)
            rospy.loginfo('map_published')
            self.process_laser = False
        self.waiting_laser = False
    
    def pose_convert(self, odom_pose):
        #spose_in_map = PoseStamped()
        dx = int((odom_pose.pose.pose.position.x) / self.map_resolution)
        dy = int((odom_pose.pose.pose.position.y) / self.map_resolution)
        self.pose_in_occupancy.pose.position.x = dx + self.map_width / 2
        self.pose_in_occupancy.pose.position.y = dy + self.map_height / 2
        self.pose_in_occupancy.pose.orientation = deepcopy(odom_pose.pose.pose.orientation) 
        #return pose_in_map

    # def pose_convert(self, world_pose):
    #     pose_in_map = PoseStamped()
    #     dx = int((world_pose.pose.position.x) / self.map_resolution)
    #     dy = int((world_pose.pose.position.y) / self.map_resolution)
    #     pose_in_map.pose.position.x = dx + self.map_width / 2
    #     pose_in_map.pose.position.y = dy + self.map_height / 2
    #     pose_in_map.pose.orientation = world_pose.pose.orientation 
    #     return pose_in_map
    

if __name__ == "__main__":
    rospy.init_node('slam_node')
    slam = Occupancy_Slam()
    rospy.spin()
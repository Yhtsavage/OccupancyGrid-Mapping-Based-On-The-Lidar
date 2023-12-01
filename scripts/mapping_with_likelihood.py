import rospy
from std_msgs.msg import Int8MultiArray
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, PoseArray, PoseWithCovariance
import numpy as np
from tf.transformations import euler_from_quaternion
from utils import *

class Occupancy_Slam(object):
    def __init__(self):
        self.pose_in_occupancy = PoseStamped()
        self.pose_in_occupancy.header.frame_id = 'map'
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback,queue_size=10)
        self.scan_sub = rospy.Subscriber("/base_scan", LaserScan, self.scan_callback, queue_size=5)
        self.map_pub = rospy.Publisher("/map", OccupancyGrid, queue_size=10)
        self.pose_pub = rospy.Publisher("/robot_pose", PoseStamped, queue_size=10)
        self._2D_map = Int8MultiArray()
        self.map_init()
        self.min_angle = -1.5707963705062866
        self.max_angle = 1.5707963705062866

    def map_init(self): # the original is 30 30 zero central
        self.map_width = 500  # 
        self.map_height = 500 # 
        self.map_center = self.map_height / 2
        self.map_resolution = 0.1  # 
        self.occupancy_grid = OccupancyGrid()
        self.occupancy_grid.info.width = self.map_width
        self.occupancy_grid.info.height = self.map_height
        self.occupancy_grid.info.resolution = self.map_resolution
        self.occupancy_grid.info.origin.position.x = -10
        self.occupancy_grid.info.origin.position.y = -10
        self.occupancy_grid.data = [-1] * (self.map_width * self.map_height)
        self._2D_map = [[-1] * self.map_width] * self.map_height 
        self._2D_likelihood = [[1] * self.map_width] * self.map_height
        self.occ_likeli = self.p2l(0.75)
        self.free_likeli = self.p2l(0.35)
        self.prior = self.p2l(0.5) 
        self.map_pub.publish(OccupancyGrid)
        rospy.loginfo(OccupancyGrid.data)
        #rospy.loginfo('map has innitialed')

    def occu2world(self, i, j):
        y, x = (i - self.map_center) * self.map_resolution, (j - self.map_center) * self.map_resolution
        return y, x
    def world2occu(self, x, y):
        j, i = (x / self.map_resolution) + self.map_center, (y / self.map_resolution) + self.map_center
        return j, i

    # p(x) = 1 - \frac{1}{1 + e^l(x)}
    def l2p(self, l):
        return 1 - (1/(1+np.exp(l)))

    # l(x) = log(\frac{p(x)}{1 - p(x)})
    def p2l(self, p):
        return np.log(p/(1-p))

    def process_sensor_data(self,laser_data):
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
                        self._2D_likelihood[map_y][map_x] += self.free_likeli - self.prior
                        current_value  = self.l2p(self._2D_likelihood[map_y][map_x])
                        #rospy.loginfo(f'free{current_value}')
                        self.occupancy_grid.data[map_y * self.map_width + map_x] = 0 if (current_value * 100) < 1 else self.occupancy_grid.data[map_y * self.map_width + map_x]
                else:
                    if i == steps - 1:  # Check if it's the last iteration
                        if rng < laser_data_max:
                            self._2D_likelihood[map_y][map_x] += self.occ_likeli - self.prior
                            current_value = self.l2p(self._2D_likelihood[map_y][map_x])
                            if current_value > 0.0:    
                                rospy.loginfo(f'occ{current_value}')
                            self.occupancy_grid.data[map_y * self.map_width + map_x] = 100 if (current_value * 100) > 5 else self.occupancy_grid.data[map_y * self.map_width + map_x]
                        else:
                            self._2D_likelihood[map_y][map_x] += self.free_likeli - self.prior
                            current_value  = self.l2p(self._2D_likelihood[map_y][map_x])
                            self.occupancy_grid.data[map_y * self.map_width + map_x] = 0 if (current_value * 100) < 1 else self.occupancy_grid.data[map_y * self.map_width + map_x]
                # if 0 <= map_x < self.map_width and 0 <= map_y < self.map_height:
                #     if rng == laser_data_max:
                #         self.occupancy_grid.data[map_y * self.map_width + map_x] = 0
                #     else:
                #         self.occupancy_grid.data[map_y * self.map_width + map_x] = 1



    def odom_callback(self, odom):
        # Publish robot pose
        self.pose_in_occupancy = self.pose_convert(odom)
        self.pose_pub.publish(self.pose_in_occupancy)
        # od = Odometry()
        # od.pose.pose.position.x

    def scan_callback(self, laser):

        self.process_sensor_data(laser)
        # Publish map data
        # Fill in map_msg data based on your map representation
        self.map_pub.publish(self.occupancy_grid)
        #rospy.loginfo('map_published')
    
    def pose_convert(self, odom_pose):
        pose_in_map = PoseStamped()
        dx = int((odom_pose.pose.pose.position.x) / self.map_resolution)
        dy = int((odom_pose.pose.pose.position.y) / self.map_resolution)
        pose_in_map.pose.position.x = dx + self.map_width / 2
        pose_in_map.pose.position.y = dy + self.map_height / 2
        pose_in_map.pose.orientation = odom_pose.pose.pose.orientation 
        return pose_in_map
    



if __name__ == "__main__":
    rospy.init_node('like_mapping')
    slam = Occupancy_Slam()
    rospy.spin()
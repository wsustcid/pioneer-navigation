#!/usr/bin/env python
'''
@Author: Shuai Wang
@Github: https://github.com/wsustcid
@Version: 1.0.0
@Date: 2020-11-26 14:58:57
@LastEditTime: 2020-12-20 16:33:39
@Description:  
'''

"""
https://zhuanlan.zhihu.com/p/21738718
"""

import os
import rospy
from sensor_msgs.msg import PointCloud
from nav_msgs.msg import Odometry
import tf
import numpy as np

import message_filters
import cv2
import pickle
root_dir = os.path.dirname(os.path.abspath(__file__))



class Mapping():
    def __init__(self):
        rospy.init_node('robo_mapping')

        # sonar params
        self.sonar_num = 8
        self.sonar_range = 5.0
        self.sonar_headings = [1.57079, 0.872664, 0.523598, 0.1745329, 
                               -0.1745329, -0.523598, -0.872664, -1.57079]
        
        # map params
        self.save_map = rospy.get_param('~save_map', default=False)
        map_name = rospy.get_param('~map_name', default='test.txt')
        self.map_file = os.path.join(root_dir, map_name) # the default root dir is .ros/
        self.map_size = 400
        
        self.grid_size = 0.1 # m
        self.grid_scale = int(1/self.grid_size) # from world unit m to map unit pixel
        self.grid_add = 100 # set map (100,100) as the world origin

        self.update = np.log(0.7/0.3)
        self.l0 = 0

        self.count = 0
        
    
    def build_map(self):
        """ Building map using sonar pointcloud and Odometry 
          1. Subscribe sonar and pose message and synchronize them
          2. Register a callback function to get the robot robot and objects position
        """
        # Initialize the world map
        self.world_map = np.zeros((self.map_size, self.map_size))
        
        # Subscribe data and process them in the callback func
        sonar_sub = message_filters.Subscriber('/RosAria/sonar', PointCloud)
        pose_sub  = message_filters.Subscriber('/RosAria/pose', Odometry)

        time_sync = message_filters.TimeSynchronizer([sonar_sub, pose_sub], queue_size=10)
        time_sync.registerCallback(self.callback_map)
        
        # show map interactively
        rospy.sleep(1)
        while not rospy.is_shutdown():
            cv2.imshow('world_map', self.world_prob)
            cv2.waitKey(100)

            if self.save_map and self.count%1000==0:
                with open(self.map_file, 'w') as f:
                    pickle.dump(self.world_prob, f)
                print("=== Save map to {} ===".format(self.map_file))
                
        
    def callback_map(self, sonar_data, pose_data):
        """ Data callback func to build map
          1. Get robot pose wrt. world coordinate
          2. Get sonar readings and rotate them to align with the world coordinate
          3. Tranlate object local pose to the world coord 
          4. Transform the object pose to the grid map index
          5. Build the grid map
          6. Show and save map
        Notes:
          1. The origin of the world coordinate is the robot initial pose, i.e. (0,0,0)
          2. Thus the robot Odometry data is wrt. world coord. 
          3. The sonar coord and robot coord have the same origin but have different orientation
        """
        self.count += 1

        ## update robot pose
        robo_x = pose_data.pose.pose.position.x
        robo_y = pose_data.pose.pose.position.y
        robo_theta = tf.transformations.euler_from_quaternion(
                                        [0, 0,
                                        pose_data.pose.pose.orientation.z,
                                        pose_data.pose.pose.orientation.w])[2] # yaw
        print("{}: x,y: ({}, {}); theta: {}".format(self.count, robo_x, robo_y, robo_theta))
        
        ## update map
        for i in range(self.sonar_num):
            # update each detected object pose
            obj_x = sonar_data.points[i].x
            obj_y = sonar_data.points[i].y
            obj_alpha = robo_theta + self.sonar_headings[i]

            obj_r = (obj_x**2+obj_y**2)**0.5
            obj_x_proj = obj_r* np.cos(obj_alpha)
            obj_y_proj = obj_r* np.sin(obj_alpha)

            obj_x_world = obj_x_proj + robo_x
            obj_y_world = obj_y_proj + robo_y

            obj_x_map = int(obj_x_world*self.grid_scale) + self.grid_add
            obj_y_map = int(obj_y_world*self.grid_scale) + self.grid_add

            # plot obstacle on the map
            if obj_r < self.sonar_range:
                self.world_map[obj_x_map, obj_y_map] += (self.update - self.l0)
            
            # plot free space on the map
            line_m = np.tan(obj_alpha) # build the line from robot to the object
            line_c = robo_y - line_m*robo_x

            x1 = robo_x
            y1 = line_m*x1 + line_c
            r1 = ((x1-robo_x)**2+(y1-robo_y)**2)**0.5    
            while r1 < min(self.sonar_range, obj_r):
                x1_map = int(x1*self.grid_scale) + self.grid_add
                y1_map = int(y1*self.grid_scale) + self.grid_add
                self.world_map[x1_map, y1_map] -= (self.update+self.l0)
                
                if robo_x < obj_x_world:
                    x1 += self.grid_size # grow x for x positive plane
                else:
                    x1 -= self.grid_size #  grow x for x negative plane
                y1 = line_m*x1 + line_c
                r1 = ((x1-robo_x)**2+(y1-robo_y)**2)**0.5 
    
        # normalize the world map
        self.world_prob = np.exp(self.world_map)/(1+np.exp(self.world_map)) # (0,1)

        



if __name__ == '__main__':
    mapping = Mapping()
    mapping.build_map()


#!/usr/bin/env python3

import numpy as np
import pdb
 
import rospy
from nav_msgs.msg import Odometry
import nav_msgs.msg
from scipy.ndimage import rotate

from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
import math
from geometry_msgs.msg import PoseStamped
import time

# from dynamic_reconfigure.server import Server
# from sphero_stage_2.cfg import obstacle_avoidance_2Config

from utils_2.vector2 import Vector2
from utils_2.vector2 import angle_diff, pose_dist, pose_vector_dist, get_agent_position, get_agent_velocity, get_agent_position_list

from utils_2.boid_2 import Boid

class Robot_move:

    
    def get_goal(self, goal):
            
        print("New goal received: ({}, {})".format(goal.pose.position.x, goal.pose.position.y))
        self.target_received = True
        self.target = [goal.pose.position.x, goal.pose.position.y]

    def send_velocity(self, frame_id, send_velocity_v):
            
            Total_velocity_cmd = Twist()
            Total_velocity_cmd.linear.x = send_velocity_v.x
            Total_velocity_cmd.linear.y = send_velocity_v.y 

            self.cmd_vel_pub[frame_id].publish(Total_velocity_cmd)
    
    def calculate_distance(self, point1, point2):
        x1, y1 = point1
        x2, y2 = point2
        # Calculate the Euclidean distance
        distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        return distance




    def odom_callback(self, msg):
            
        frame_id = msg.header.frame_id 
        id = frame_id[7]

        if frame_id not in self.cmd_vel_pub:
            self.cmd_vel_pub[frame_id] = rospy.Publisher("{}cmd_vel".format(frame_id[:9]), Twist, queue_size=10)
            
        self.agent= self.agents[int(id)]

        if self.target_received:
            _, nav_velocity_v, self.target_reached_list[int(id)]  = self.agent.get_cmd_vel(msg, self.target, self.target_reached_list[int(id)])
            if all(self.target_reached_list):
                self.target_received = False
                self.target_reached_list = [False for i in range(self.num_of_robots)]

                print("Target position reached!")
        else:
            nav_velocity_v = Vector2()

        # """ ===== Total Velocity ======"""

        self.Total_velocity = self.nav_weight * nav_velocity_v 
        self.send_velocity(frame_id, self.Total_velocity)
    










    def __init__(self):

        # initialize
        self.count = 1
        self.target_received = False

        
        # fixed parameters
        self.num_of_robots = rospy.get_param("/num_of_robots")
        self.target = [0.0, 0.0]
        self.velocity = [0.0, 0.0]  # Initial velocity
        self.initial_time = rospy.get_time()
        self.cmd_vel_pub = {}
        self.target_reached_list = [False for i in range(self.num_of_robots)]

        # Tunable parameters
        self.max_speed = 0.6
        self.slowing_radius = 1.0
        self.search_radius = 2.0

        # Weights
        self.nav_weight = 1
        self.Total_velocity = Vector2()


        # self.map_subscriber = rospy.Subscriber("/map", nav_msgs.msg.OccupancyGrid, self.read_map_callback_2)
        self.move_goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.get_goal)#None # TODO: subscriber to /move_base_simple/goal published by rviz    
        
        rate = rospy.Rate(10)
        rate.sleep()
        
        self.agents = [Boid(initial_velocity_x=0.0, initial_velocity_y=0.0, max_speed_mag=self.max_speed, 
                            slowing_radius=self.slowing_radius) for _ in range(self.num_of_robots)]

        [rospy.Subscriber("robot_{}/odom".format(i), Odometry, self.odom_callback) for i in range(self.num_of_robots)]




if __name__ == '__main__':
    try:
        rospy.init_node('consensus_mrs_2')
        robot = Robot_move()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass


















# def param_callback(self, config, level):

    #     # other params
    #     self.max_speed = config.max_speed
    #     self.slowing_radius = config.slowing_radius
    #     # self.search_radius = config.search_radius

    #     # weights param
    #     self.nav_weight = config.nav_weight
    #     self.rep_weight = config.rep_weight

    #     return config

# def read_map_callback_2(self, msg):
    #     """Callback function that is called when a message is received on the `/map` topic."""

    #     map_data = np.array([msg.data])
    #     # map_data = map_data.reshape(200, 200)
    #     map_data = np.reshape(map_data, (200, 200), 'F')
    #     self.new_map_data = rotate(map_data, 90)
    #     self.map_subscriber.unregister()

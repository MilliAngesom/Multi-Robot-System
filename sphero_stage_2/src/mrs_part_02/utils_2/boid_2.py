#!/usr/bin/env python3

import numpy as np
import pdb
 
import math
import rospy
from geometry_msgs.msg import Twist
import rospy
from geometry_msgs.msg import Twist
import rospy
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import math
from tf.transformations import euler_from_quaternion
import random

from utils_2.vector2 import Vector2
from utils_2.vector2 import angle_diff, pose_dist, pose_vector_dist, get_agent_position, get_agent_velocity, get_agent_position_list

# boid class represents a specific assigned boid and its methods.
# add any methods related to each boid like arrive behavior is added here. properties that all boids present

class Boid(object):

    # use twist type for velocity and pose type message for position

    def __init__(self, adj_mat, edge_values, id, position_error_list, pos_diff_list, target_lists,
                 max_speed_mag, slowing_radius, search_radius): # , wait_count, start_count, frequency
        """Create an boid with empty properties and update parameters."""
        self.position_v         = Vector2()
        self.cur_vel_v          = Vector2()
        self.adj_mat            = adj_mat
        self.id                 = id 
        self.edges              = edge_values # nodes to which current agent is connected like [2, 3]
        

        self.target_list           = target_lists
        self.position_error_list   = position_error_list
        self.pos_diff_list         = pos_diff_list     # distance between agent and target
 

        self.max_speed_mag  = max_speed_mag  #
        self.slowing_radius = slowing_radius #
        self.search_radius  = search_radius

        self.target_reached = False           # So, they dont move until they get a target  
    

        # self.des_vel_v          = Vector2()

    def arrive(self): # update the current velocity of the agent

        if all(pos_diff < 0.13 for pos_diff in self.pos_diff_list):
            self.target_reached = True
            self.cur_vel_v.set_mag(0)
        else:
            for i in range(len(self.edges)): # can skip adding adj_mat as well
                ramped_speed        = (self.pos_diff_list[i] / self.slowing_radius)
                self.cur_vel_v.x    += self.adj_mat[self.id, self.edges[i]] * (ramped_speed / self.pos_diff_list[i]) * self.position_error_list[i].x
                self.cur_vel_v.y    += self.adj_mat[self.id, self.edges[i]] * (ramped_speed / self.pos_diff_list[i]) * self.position_error_list[i].y


            if self.cur_vel_v.norm() > self.max_speed_mag:
                self.cur_vel_v.set_mag(self.max_speed_mag)





































            # self.cur_vel_v.x = (ramped_speed / self.pos_diff) * self.position_error_v.x
            # self.cur_vel_v.y = (ramped_speed / self.pos_diff) * self.position_error_v.y
            

            # return desired_velocity_v, self.target_reached
        
    # def get_cmd_vel(self, agent_msg, target, target_reached):

    #     self.steering_force_v, new_target_reached = self.arrive(agent_msg, target, target_reached)

    #     cmd = Twist()
    #     cmd.linear.x = self.steering_force_v.x
    #     cmd.linear.y = self.steering_force_v.y  # Adjust the angular velocity as needed
    #     return cmd, self.steering_force_v, new_target_reached

        
















   # # position and velocity vector components
    # def set_pose(self, name, variable):
    #     rospy.set_param(name, variable)


    # def meters_to_pixels(self, meter_coordinates, map_size=200, map_range=10.0): # meter_coordinates = [x, y]
        
    #     # Calculate the middle pixel (center of the map)
    #     middle_pixel = map_size // 2
        
    #     # Calculate the pixel scale per meter
    #     pixels_per_meter = map_size / map_range
        
    #     # Convert meters to pixels
    #     x_meters, y_meters = meter_coordinates
    #     x_pixels = int((x_meters  * pixels_per_meter)+ middle_pixel)
    #     y_pixels = int(-(y_meters  * pixels_per_meter) + middle_pixel)

    #     return [y_pixels, x_pixels]
    
    # def pixels_to_meters(self, pixel_coordinates, map_size=200, map_range=10.0):
    #     # Calculate the middle pixel (center of the map)
    #     middle_pixel = map_size // 2
        
    #     # Calculate the meters per pixel
    #     meters_per_pixel = map_range / map_size
        
    #     # Convert pixels to meters
    #     y_pixels, x_pixels = pixel_coordinates
    #     y_meters = (middle_pixel - y_pixels) * meters_per_pixel
    #     x_meters = (x_pixels - middle_pixel) * meters_per_pixel
        
    #     return [x_meters, y_meters]
    
    
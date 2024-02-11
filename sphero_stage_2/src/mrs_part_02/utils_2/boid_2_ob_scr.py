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

from bresenham import bresenham
# boid class represents a specific assigned boid and its methods.
# add any methods related to each boid like arrive behavior is added here. properties that all boids present

class Boid(object):

    # use twist type for velocity and pose type message for position

    def __init__(self, map_data, adj_mat, edge_values, id, position_error_list, pos_diff_list, target_lists,
                 max_speed_mag, max_speed_leader, slowing_radius, slowing_radius_other, search_radius, sep_dist): # , wait_count, start_count, frequency
        """Create an boid with empty properties and update parameters."""
        self.position_v         = Vector2()
        self.cur_vel_v          = Vector2()
        self.adj_mat            = adj_mat
        self.id                 = id 
        self.edges              = edge_values # nodes to which current agent is connected like [2, 3]
        
        self.separation_dist    = sep_dist    # separation distance between the robots
        
        self.target_list           = target_lists
        self.position_error_list   = position_error_list
        self.pos_diff_list         = pos_diff_list     # distance between agent and target
 

        self.max_speed_mag  = max_speed_mag  #
        self.max_speed_leader = max_speed_leader
        self.slowing_radius = slowing_radius #
        self.search_radius  = search_radius
        self.slowing_radius_other = slowing_radius_other

        self.target_reached = False           # So, they dont move until they get a target  
        self.all_clusters = rospy.get_param('/clusters')
        self.all_centroids = rospy.get_param('/centroids')
        self.map_data = map_data

        # self.des_vel_v          = Vector2()

    def arrive(self): # update the current velocity of the agent

        if all(abs(pos_diff) < 0.05 for pos_diff in self.pos_diff_list):
            self.target_reached = True
            self.cur_vel_v.set_mag(0)
            
        else:
            self.target_reached = False
            for i in range(len(self.edges)): # can skip adding adj_mat as well
                if self.edges[i]==None:
                    if abs(self.pos_diff_list[self.id]) < 0.05:
                        self.cur_vel_v.set_mag(0)
                    else:
                        ramped_speed        = (self.pos_diff_list[i] / self.slowing_radius)
                        self.cur_vel_v.x    =  (ramped_speed / self.pos_diff_list[i]) * self.position_error_list[i].x
                        self.cur_vel_v.y    =  (ramped_speed / self.pos_diff_list[i]) * self.position_error_list[i].y
                        if self.cur_vel_v.norm() > self.max_speed_leader:
                            self.cur_vel_v.set_mag(self.max_speed_leader)
                else:
                    ramped_speed        = (self.pos_diff_list[i] / self.slowing_radius_other)
                    self.cur_vel_v.x    += self.adj_mat[self.id, self.edges[i]] * (ramped_speed / self.pos_diff_list[i]) * self.position_error_list[i].x
                    self.cur_vel_v.y    += self.adj_mat[self.id, self.edges[i]] * (ramped_speed / self.pos_diff_list[i]) * self.position_error_list[i].y
                    if self.cur_vel_v.norm() > self.max_speed_mag:
                        self.cur_vel_v.set_mag(self.max_speed_mag)
            self.pos_diff_list = []
                    

    def arrive_ob(self, agent_msg, target):
        target_v = Vector2(target[0], target[1])
        desired_velocity_v = Vector2()
        self.position_v = get_agent_position(agent_msg) # agent position
        # print('agent_position', get_agent_position(agent_msg))


        target_offset_v = target_v - self.position_v
        distance = target_offset_v.norm() 
        
        ramped_speed = (distance / self.slowing_radius)
        
        if distance < 0.02:
            # print(f'position reached')
            return Vector2()
        else:
            desired_velocity_v.x = (ramped_speed / distance) * target_offset_v.x
            desired_velocity_v.y = (ramped_speed / distance) * target_offset_v.y
                    # ]
            if target_offset_v.norm() > self.max_speed_leader:
                desired_velocity_v.set_mag(self.max_speed_leader)
            # print(' desired_velocity_v', desired_velocity_v.x, desired_velocity_v.y)
            return desired_velocity_v
        

    def get_cmd_vel(self, agent_msg, target):

        self.steering_force_v = self.arrive_ob(agent_msg, target)

        # steering_force_history.append(self.steering_force)
        # velocity_history.append(self.velocity)

        cmd = Twist()
        cmd.linear.x = self.steering_force_v.x
        cmd.linear.y = self.steering_force_v.y  # Adjust the angular velocity as needed
        return cmd, self.steering_force_v



    def meters_to_pixels(self, meter_coordinates, map_size=200, map_range=10.0): # meter_coordinates = [x, y]
        
        if math.isnan(meter_coordinates[0]) or math.isnan(meter_coordinates[1]):
        # Handle the NaN case, maybe log an error, return a default value, or raise an exception
            return None 
        # Calculate the middle pixel (center of the map)
        middle_pixel = 100 #map_size // 2
        
        # Calculate the pixel scale per meter
        pixels_per_meter = 20 #map_size / map_range
        
        # Convert meters to pixels
        x_meters, y_meters = meter_coordinates
        x_pixels = int((x_meters  * pixels_per_meter)+ middle_pixel)
        y_pixels = int(-(y_meters  * pixels_per_meter) + middle_pixel)

        return [y_pixels, x_pixels]
    
    def pixels_to_meters(self, pixel_coordinates, map_size=200, map_range=10.0):
        # Calculate the middle pixel (center of the map)
        middle_pixel = 100 #map_size // 2
        
        # Calculate the meters per pixel
        meters_per_pixel = map_range / map_size
        
        # Convert pixels to meters
        y_pixels, x_pixels = pixel_coordinates
        y_meters = (middle_pixel - y_pixels) * meters_per_pixel
        x_meters = (x_pixels - middle_pixel) * meters_per_pixel
        
        return [x_meters, y_meters]



# take end point from robot and map and return all the cells with obstacles
    def check_front_cylinder_obstacle(self, position_list, end_array, cylinder_len=1.0):

        start_m = [position_list[0], position_list[1]]
        if math.isnan(start_m[0]) or math.isnan(start_m[1]):
        # Handle the NaN case, maybe log an error, return a default value, or raise an exception
            return None 
        start_pi = self.meters_to_pixels(start_m)
        
        angle_rad = math.atan2(end_array[1], end_array[0]) 
        
        end_m = [start_m[0] + cylinder_len*np.cos(angle_rad), start_m[1] + cylinder_len*np.sin(angle_rad)] # equal lenth 
        
        # Calculate end_pi without clamping 
        end_pi_unclamped = self.meters_to_pixels(end_m)
        if end_pi_unclamped == None:
            return None
        # Clamp end_pi values to stay within boundary
        end_pi = [np.clip(coord, 0, 200) for coord in end_pi_unclamped]

        # Define the line endpoints
        line_endpoints = [start_pi, end_pi]
        # print('line', line_endpoints)
        # print('line', start_pi, end_pi)
        # pdb.set_trace()

        # Get the cells from Bresenham's algorithm
        cells = list(bresenham(line_endpoints[0][0], line_endpoints[0][1], line_endpoints[1][0], line_endpoints[1][1]))
        # print(cells)

        
        # Expand the list to include adjacent cells
        expanded_cells = cells.copy()
        for cell in cells:
            for i in range(-2, 3):
                for j in range(-2, 3):
                    if (i, j) != (0, 0):  # Exclude the original cells
                        adjacent_cell = (cell[0] + i, cell[1] + j)
                        if 0 <= adjacent_cell[0] < self.map_data.shape[0] and 0 <= adjacent_cell[1] < self.map_data.shape[1]:
                            expanded_cells.append(adjacent_cell)

        # Remove duplicates
        expanded_cells = list(set(expanded_cells))

        # Find cells with a value of 100
        # cells_with_100 = [[x, y] for x, y in expanded_cells if self.map_data[x][y] == 100]
        cells_with_100 = [[x, y] for x, y in expanded_cells if 0 <= x < 200 and 0 <= y < 200 and self.map_data[x][y] == 100]
        # print('cells with hundread', cells_with_100)
        if not cells_with_100:
            # print('100 null')
            return None  # no obstacle cells
        # pdb.set_trace()
        return cells_with_100 




# take all the cells with obstacles and return clusters and their centroids

    def find_cluster_centroid_infront(self, agent_msg, start_list, end_array, cylinder_length): 
        """end is with velocity vector and map_array is a 2D array"""
        # self.position_v = get_agent_position(agent_msg)

        
        indices = self.check_front_cylinder_obstacle(start_list, end_array, cylinder_length) 
        # pdb.set_trace()
        if not indices:
            # print('returning empty indices')
            return [] # no obstacles found
        
        # all_clusters = rospy.get_param('/clusters')
        # all_centroids = rospy.get_param('/centroids')
        
        # all_clusters = # [[[2, 3], [2, 2]], [[4, 3]], [[4, 4]]]
        # all_centroids = [[4.0], [2.0]]

        centroids_of_interest = []
        for point in indices:
            for idx, cluster in enumerate(self.all_clusters):
                if point in cluster:
                    centroid = self.all_centroids[idx]
                    if centroid not in centroids_of_interest:
                        centroids_of_interest.append(centroid)

        # Convert centroids from pixels to meters
        centroids_meters = [self.pixels_to_meters(centroid_pixel) for centroid_pixel in centroids_of_interest]
        # print('centroids', centroids_meters)
        # pdb.set_trace()
        
        return centroids_meters   # [[1.0, 1.5], [2.0, 2.5]] cemtroids of the obstacles
        




    def rotate_vector(self, vector, direction, angle_deg):

        angle_rad = np.radians(angle_deg)

        if direction == "clockwise":
            rotation_matrix = np.array([[np.cos(angle_rad), np.sin(angle_rad)],
                                        [-np.sin(angle_rad), np.cos(angle_rad)]])
        else:  # Counterclockwise
            rotation_matrix = np.array([[np.cos(angle_rad), -np.sin(angle_rad)],
                                        [np.sin(angle_rad), np.cos(angle_rad)]])
        
        # Perform matrix multiplication to rotate the vector
        # rotated_vector = np.dot(rotation_matrix, vector)/np.linalg.norm(vector)
        rotated_vector = np.dot(rotation_matrix, vector)/np.linalg.norm(vector)
        return rotated_vector

                
    def find_repulsive_vector(self, total_vel_array, adjacent_differences_pos, angle_list_positive_sorted, adjacent_differences_neg, angle_list_negative_sorted):
            print('in final repul')
            max_diff_pos = max(adjacent_differences_pos) 
            max_diff_neg = max(adjacent_differences_neg) 

            if max_diff_pos > abs(max_diff_neg):

                direction = "counter-clockwise"
                max_index_pos = adjacent_differences_pos.index(max_diff_pos)
                big_angle_pos = angle_list_positive_sorted[max_index_pos + 1]
                small_angle_pos = angle_list_positive_sorted[max_index_pos]
                desired_orient_pos = small_angle_pos + (big_angle_pos - small_angle_pos)/2
                repulsive_v_array = self.rotate_vector(total_vel_array, direction, desired_orient_pos)

            else:
                direction = "clockwise"
                max_index_neg = adjacent_differences_neg.index(max_diff_neg)
                big_angle_neg = angle_list_negative_sorted[max_index_neg + 1]
                small_angle_neg = angle_list_negative_sorted[max_index_neg]
                desired_orient_neg = small_angle_neg + (big_angle_neg - small_angle_neg)/2
                repulsive_v_array = self.rotate_vector(total_vel_array, direction, desired_orient_neg)

            return repulsive_v_array




    def get_desired_repulsive_vel(self, agent_msg, total_vel_array, CW, obstacles, max_repulsive_force=1.0):
        
        # total_vel = rospy.get_param('total_vel')
        # total_vel_array = np.array([total_vel[0], total_vel[1]])

        self.position_v = get_agent_position(agent_msg)
        # nav_velocity_v_array = np.array([nav_velocity_v.x, nav_velocity_v.y])

        start_m = [self.position_v.x, self.position_v.y]
        start_m_array = np.array([start_m[0], start_m[1]])
        start_pi = self.meters_to_pixels(start_m)

        adjacent_cells = []

        # Check 10 pixels in all directions around the robot 
        for i in range(-15, 16):
            for j in range(-15, 16):
                cell = [start_pi[0] + i, start_pi[1] + j]

                # Ensure the cell is within the map bounds
                if 0 <= cell[0] < 200 and 0 <= cell[1] < 200:
                    # Check if the cell contains an obstacle
                    if self.map_data[cell[0]][cell[1]] == 100:
                        adjacent_cells.append(cell)
        # pdb.set_trace()
        # Get clusters and centroids from ROS parameters
        # all_clusters = rospy.get_param('/clusters')
        # all_centroids = rospy.get_param('/centroids')

        centroids_of_interest = []
        for point in adjacent_cells:
            for idx, cluster in enumerate(self.all_clusters):
                if point in cluster:
                    centroid = self.all_centroids[idx]
                    if centroid not in centroids_of_interest:
                        centroids_of_interest.append(centroid)
        
        centroids_meters = [self.pixels_to_meters(centroid_pixel) for centroid_pixel in centroids_of_interest]
        
        # pdb.set_trace()

        angle_list_positive = []
        angle_list_negative = []


        for centroid_m in centroids_meters:
            centroid_m_array = np.array([centroid_m[0], centroid_m[1]])

            along_vector = [centroid_m_array[0]-start_m_array[0], centroid_m_array[1]-start_m_array[1]]

            # cross_product = np.cross(nav_velocity_v_array, along_vector)

            dot_product = np.dot(total_vel_array, along_vector)
            cross_product = np.cross(total_vel_array, along_vector)

            # Calculate angle in radians
            angle_rad = np.arctan2(np.linalg.norm(cross_product), dot_product)
            angle_deg = np.degrees(angle_rad)

            # Convert angle to degrees
            if cross_product > 0:
                angle_list_positive.append(angle_deg)
            else:
                angle_list_negative.append(-angle_deg)

        
        # print('positive angle:', angle_list_positive)
        # print('negative angle:', angle_list_negative)
        # Check for angles between 45 to 135 degrees 
        angles_45_to_135 = [angle for angle in angle_list_positive if 45 <= angle <= 135]
    
        # Check for angles between -45 to -135 degrees 
        angles_minus_45_to_135 = [angle for angle in angle_list_negative if -135 <= angle <= -45]

        angles_100_to_170 = [angle for angle in angle_list_positive if 100 <= angle <= 170]
    
        # Check for angles between -45 to -135 degrees
        angles_minus_100_to_minus_170 = [angle for angle in angle_list_negative if -170 <= angle <= -100]

        # if not angles_45_to_135 or not angles_minus_45_to_135:
        #     print(90)
        #     rotate_angle_45_to_135 = 100
        #     repulsive_v_array, CW = self.angle_between_100_and_170(CW, rotate_angle_45_to_135, angles_45_to_135, angles_minus_45_to_135, total_vel_array)

        if not angles_100_to_170 or not angles_minus_100_to_minus_170:
            # print(115)

            # rotate_angle_100_to_170 = 90
            rotate_angle_100_to_170 = 115
            repulsive_v_array, CW = self.angle_between_100_and_170(CW, rotate_angle_100_to_170, angles_100_to_170, angles_minus_100_to_minus_170, total_vel_array)

        else:
        
            angle_list_positive_sorted = sorted(angle_list_positive)
            adjacent_differences_pos = [angle_list_positive_sorted[i + 1] - angle_list_positive_sorted[i] for i in range(len(angle_list_positive_sorted) - 1)]
            

            angle_list_negative_sorted = sorted(angle_list_negative)
            adjacent_differences_neg = [angle_list_negative_sorted[i + 1] - angle_list_negative_sorted[i] for i in range(len(angle_list_negative_sorted) - 1)] # angle difference

            
            if not adjacent_differences_pos: # if it is empty 
                
                direction = "counter-clockwise"
                repulsive_v_array = self.rotate_vector(total_vel_array, direction, 90)
                # repulsive_v_array = self.angle_to_vector(-90) # project a vector in along 

            elif not adjacent_differences_neg: # if it is empty

                direction = "clockwise"
                repulsive_v_array = self.rotate_vector(total_vel_array, direction, 90)

            else:
                repulsive_v_array = self.find_repulsive_vector(total_vel_array, adjacent_differences_pos, angle_list_positive_sorted, adjacent_differences_neg, angle_list_negative_sorted)




        desired_repulsive_velocity_v = Vector2()
        desired_repulsive_velocity_v.x = repulsive_v_array[0]
        desired_repulsive_velocity_v.y = repulsive_v_array[1]
        # pdb.set_trace()

        return desired_repulsive_velocity_v, CW
                


    def angle_between_100_and_170(self, CW, rotate_angle, angles_positive, angles_negative, total_vel_array):

        if not angles_positive and not angles_negative:
            # x = random.choice([0, 1]) # 50% prob
            if CW < 15:
                print('in CW less')

                direction = "counter-clockwise"
                repulsive_v_array = self.rotate_vector(total_vel_array, direction, rotate_angle)
                CW += 1
            elif CW >= 15:
                print('in CW more')

                direction = "clockwise"
                repulsive_v_array = self.rotate_vector(total_vel_array, direction, rotate_angle)
                CW += 1

        elif not angles_positive:
            print('in +')
                           
            direction = "counter-clockwise"
            repulsive_v_array = self.rotate_vector(total_vel_array, direction, rotate_angle)

            # perpendicular_v = np.array([-total_vel_array[1], total_vel_array[0]])
            # repulsive_v_array = perpendicular_v / np.linalg.norm(perpendicular_v)

        elif not angles_negative:
            print('in -')
            direction = "clockwise"
            repulsive_v_array = self.rotate_vector(total_vel_array, direction, rotate_angle)
            # perpendicular_v = np.array([-total_vel_array[1], total_vel_array[0]])
            # repulsive_v_array = perpendicular_v / np.linalg.norm(perpendicular_v) 
        return repulsive_v_array, CW
    
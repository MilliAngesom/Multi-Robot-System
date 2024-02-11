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

from dynamic_reconfigure.server import Server
from sphero_stage_2.cfg import fixedConfig

from utils_2.vector2 import Vector2
from utils_2.vector2 import angle_diff, pose_dist, pose_vector_dist, get_agent_position, get_agent_velocity, get_agent_position_list

from utils_2.boid_2_ob_scr import Boid

class Robot_move:
    
    def get_goal(self, goal):
            
        print("New goal received: ({}, {})".format(goal.pose.position.x, goal.pose.position.y))
        self.target_received = True
        self.nav_flag = True
        self.stub_target.x = goal.pose.position.x
        self.stub_target.y = goal.pose.position.y
        self.target_m = Vector2(goal.pose.position.x, goal.pose.position.y)
        self.targetss = [self.stub_target.x, self.stub_target.y]

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

    # update all parameters of the boid
    def update_param(self, agent_msg, agent, boids):

        agent.position_v                    = get_agent_position(agent_msg) # current position of the agent
        agent.target_list                   = [self.target_m if value is None else boids[value].position_v for value in agent.edges]     
        agent.position_error_list           = [agent.target_list[i]-agent.position_v - agent.separation_dist[i] for i in range(len(agent.edges))]
        agent.pos_diff_list                 = [agent.position_error_list[i].norm() for i in range(len(agent.edges))]
        agent.arrive()                      # update the current velocity of the agent
        self.target_reached_list[agent.id]  = agent.target_reached
        return agent.cur_vel_v


    def repulsive_vector_fn(self, msg, agent, Total_velocity_list, cylinder_length, Leader=False):

        repulsive_velocity_v_inside_func = Vector2()

        if Leader:
            start_list = [agent.position_v.x, agent.position_v.y]
            total_vel_array = np.array([Total_velocity_list[0], Total_velocity_list[1]])
            obstacle_centroids_points_list = agent.find_cluster_centroid_infront(msg, start_list, total_vel_array, cylinder_length)


            if (not obstacle_centroids_points_list):
                repulsive_velocity_v_inside_func = Vector2()  

            elif obstacle_centroids_points_list: 
                self.nav_flag = False
                repulsive_velocity_v_inside_func, self.CW = agent.get_desired_repulsive_vel(msg, total_vel_array, self.CW, obstacle_centroids_points_list)
                if self.CW == 30:
                    self.CW = 0

        if repulsive_velocity_v_inside_func.x != 0.0 or repulsive_velocity_v_inside_func.y != 0.0:
            repulsive_velocity_v_inside_func.set_mag(self.max_repul_speed)
        return repulsive_velocity_v_inside_func

    def odom_callback(self, msg):

        frame_id = msg.header.frame_id 
        id = frame_id[7]

        if frame_id not in self.cmd_vel_pub:
            self.cmd_vel_pub[frame_id] = rospy.Publisher("{}cmd_vel".format(frame_id[:9]), Twist, queue_size=10)
            
        self.agent= self.agents[int(id)] # taking instance of that robot
        self.agent.position_v = get_agent_position(msg)

        if self.agent.edges == [None]:
            if self.target_received:

                    if self.nav_flag:
                        self.Leader_vel_final = self.update_param(msg, self.agent, self.agents)
                        self.Leader_vel_list = [float(self.Leader_vel_final.x), float(self.Leader_vel_final.y)]
                    
                    leader_repel_vel = self.repulsive_vector_fn(msg, self.agent, self.Leader_vel_list, self.cylinder_length, Leader=True) # if no obstacle, then  zeo

                    if leader_repel_vel.x != 0.0 or leader_repel_vel.y != 0.0:
                        print('obstacle infront of the leader')
                        self.repul_prev_v = leader_repel_vel


                    if not self.nav_flag:   
                        self.Leader_vel_final = self.repul_prev_v
                    
                    self.Leader_vel_list = [float(self.Leader_vel_final.x), float(self.Leader_vel_final.y)]
                    self.send_velocity(self.frame_id_leader, self.Leader_vel_final)


        # if self.agent.edges != [None]: # if not a stub born
        if self.agent.id == 1: # if not a stub born
            frame_local_1 = frame_id
            agent_local_1 = self.agent

            agent_vel_1 = self.update_param(msg, agent_local_1, self.agents)
            self.a_final_vel_1 = [float(agent_vel_1.x), float(agent_vel_1.y)]

            if self.a_final_vel_1[0] != 0.0 or self.a_final_vel_1[1] != 0.0:
                if self.count_1_f == 10:
                    a_repel_vel_1 = self.repulsive_vector_fn(msg, agent_local_1, self.a_final_vel_1, self.cylinder_length, Leader=True)
                    if a_repel_vel_1.x != 0.0 or a_repel_vel_1.y != 0.0:
                        print('obstacle infront of First follower')

                        self.repul_prev_v = a_repel_vel_1
                    self.count_1_f = 1
                else: 
                    self.count_1_f += 1
            self.send_velocity(frame_local_1, agent_vel_1)

        if self.agent.id == 2: # if not a stub born
            frame_local_2 = frame_id
            agent_local_2 = self.agent

            agent_vel_2 = self.update_param(msg, agent_local_2, self.agents)
            self.a_final_vel_2 = [float(agent_vel_2.x), float(agent_vel_2.y)]

            if self.a_final_vel_2[0] != 0.0 or self.a_final_vel_2[1] != 0.0:
                if self.count_2_f == 10:
                    self.count_2_f = 1
                    a_repel_vel_2 = self.repulsive_vector_fn(msg, agent_local_2, self.a_final_vel_2, self.cylinder_length, Leader=True)
                    if a_repel_vel_2.x != 0.0 or a_repel_vel_2.y != 0.0:
                        print('obstacle infront of Second follower')

                        self.repul_prev_v = a_repel_vel_2
                else: 
                    self.count_2_f += 1
            self.send_velocity(frame_local_2, agent_vel_2)


    def read_map_callback_2(self, msg):
        """Callback function that is called when a message is received on the `/map` topic."""

        map_data = np.array([msg.data])
        map_data = np.reshape(map_data, (200, 200), 'F')
        self.new_map_data = rotate(map_data, 90)
        self.map_subscriber.unregister()

    def __init__(self):

        # initialize
        self.target_received = False
        self.num_of_robots          = rospy.get_param("/num_of_robots")
        
        """ Edges and Adjacency matrix """
        # Adjacency matrix
        self.adj_mat = np.zeros((self.num_of_robots, self.num_of_robots))
        self.edges   = {}
        # self.edges[0] = [1]  # self.edges[1] = [2, 3]: 1 listens to 2 and 3
        # self.edges[1] = [None]     # self.edges[2] = [0]    : 2 listens to no one
        # self.edges[2] = [None]  # self.edges[3] = [1, 3]: 3 listens to 1 and 3

        """ Formation """
        self.d = 0.5
        self.separation_dist = {}

        ## Square
        # self.edges[0] = [None]
        # self.edges[1] = [0]
        # self.edges[2] = [0,1,3]
        # self.edges[3] = [0,1]

        # self.separation_dist[0] = [Vector2(0,0)]
        # self.separation_dist[1] = [Vector2(d,0)]
        # self.separation_dist[2] = [Vector2(d,-d), Vector2(0,-d), Vector2(d,0)]
        # self.separation_dist[3] = [Vector2(0,-d), Vector2(-d,-d)]


        ## Line
        self.edges[0] = [None]
        self.edges[1] = [0]
        # self.edges[2] = [0, 1]
        self.edges[2] = [0]

        self.separation_dist[0] = [Vector2(0,0)]
        self.separation_dist[1] = [Vector2(0,-self.d)]
        # self.separation_dist[2] = [Vector2(0,d), Vector2(0,2*d)]
        self.separation_dist[2] = [Vector2(0,self.d)]

        ## Triangle
        self.edges[0] = [None]
        self.edges[1] = [0]
        self.edges[2] = [0,1]

        self.separation_dist[0] = [Vector2(0,0)]
        self.separation_dist[1] = [Vector2(0.5*self.d,-0.866*self.d)]
        self.separation_dist[2] = [Vector2(-0.5*self.d,-0.866*self.d), Vector2(-self.d,0)]

        """ Build adjacency matrix """
        for key, values in self.edges.items():
            for value in values:
                if value == None: # stub born
                    continue
                self.adj_mat[key, value] = 1

        """ List Parameters to be updated by all robots """
        self.cmd_vel_pub            = {}
        self.target_reached_list    = [False for i in range(self.num_of_robots)]
        self.target_lists           = [list() for i in range(self.num_of_robots)] 
        self.position_error_list    = [list() for i in range(self.num_of_robots)]
        self.pos_diff_list          = [list() for i in range(self.num_of_robots)]

        
        # Tunable parameters
        self.max_speed          = 0.8 # 0.6
        # Leader
        self.max_speed_leader   = 0.6 # 0.4
        self.max_repul_speed    = 0.6 # 0.4
        self.slowing_radius     = 1.0
        self.search_radius      = 2.0
        self.slowing_radius_other = 0.5
        self.cylinder_length   = 0.2

        # Weights
        self.nav_weight = 1
        self.nav_velocity_v = Vector2()

        self.map_subscriber = rospy.Subscriber("/map", nav_msgs.msg.OccupancyGrid, self.read_map_callback_2)
        self.move_goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.get_goal)#None # TODO: subscriber to /move_base_simple/goal published by rviz    
        
        rate = rospy.Rate(10)
        rate.sleep()

        self.agents = [Boid(self.new_map_data, self.adj_mat, self.edges[id], id, self.position_error_list[id], self.pos_diff_list[id], self.target_lists[id],
                            self.max_speed, self.max_speed_leader, self.slowing_radius, self.slowing_radius_other, self.search_radius, self.separation_dist[id]) for id in range(self.num_of_robots)]

        self.stub_target = Vector2(1, 0)

        self.CW = 0
        self.count = 1
        self.count_1_f = 1
        self.count_2_f = 1
        self.nav_flag = True
        self.repul_weight = 1 

        # repulsive vectors
        self.repul_vel = Vector2()
        self.repul_prev_v = Vector2()

        # Leader 
        self.Leader_vel_final = Vector2()
        self.Leader_vel_list = [0.0, 0.0]
        self.frame_id_leader = '/robot_0/odom'
        self.send_repel_vel = False
        self.target_m = Vector2()

        # Agent
        self.agent_vel = Vector2()
        self.a_final_vel = [0.0, 0.0]
        self.Total_velocity_other_list = [0.0, 0.0]


        [rospy.Subscriber("robot_{}/odom".format(i), Odometry, self.odom_callback) for i in range(self.num_of_robots)]



    def param_callback(self, config, level):

            # other params
            self.d = config.separation_dist

            ## Line
            self.edges[0] = [None]
            self.edges[1] = [0]
            # self.edges[2] = [0, 1]
            self.edges[2] = [0]

            self.separation_dist[0] = [Vector2(0,0)]
            self.separation_dist[1] = [Vector2(0,-self.d)]
            # self.separation_dist[2] = [Vector2(0,d), Vector2(0,2*d)]
            self.separation_dist[2] = [Vector2(0,self.d)]
        
            ## Triangle
            self.edges[0] = [None]
            self.edges[1] = [0]
            self.edges[2] = [0,1]

            self.separation_dist[0] = [Vector2(0,0)]
            self.separation_dist[1] = [Vector2(0.5*self.d,-0.866*self.d)]
            self.separation_dist[2] = [Vector2(-0.5*self.d,-0.866*self.d), Vector2(-self.d,0)]
                
            self.agents = [Boid(self.new_map_data, self.adj_mat, self.edges[id], id, self.position_error_list[id], self.pos_diff_list[id], self.target_lists[id],
                        self.max_speed, self.max_speed_leader, self.slowing_radius, self.slowing_radius_other, self.search_radius, self.separation_dist[id]) for id in range(self.num_of_robots)]

            return config


if __name__ == '__main__':
    try:
        rospy.init_node('consensus_mrs_2')
        robot = Robot_move()
        srv = Server(fixedConfig, robot.param_callback)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass



#!/usr/bin/env python3

import numpy as np
import pdb
import math

import rospy
import logging
from geometry_msgs.msg import Twist


class Vector2(object): # agent
    def __init__(self, x=0, y=0):
        """
        Initialize vector components.

        Args:
            x (float): x component of the vector
            y (float): y component of the vector
        """
        self.x = x
        self.y = y
    
    def __add__(self, other):
        if isinstance(other, self.__class__):
            return Vector2(self.x + other.x, self.y + other.y)
        elif isinstance(other, int) or isinstance(other, float):
            return Vector2(self.x + other, self.y + other)

    def __sub__(self, other):
        if isinstance(other, self.__class__):
            return Vector2(self.x - other.x, self.y - other.y)
        elif isinstance(other, int) or isinstance(other, float):
            return Vector2(self.x - other, self.y - other)

    def __mul__(self, other):
        if isinstance(other, self.__class__):
            raise NotImplementedError("Multiplying vectors is not implemented!")
        elif isinstance(other, int) or isinstance(other, float):
            return Vector2(self.x * other, self.y * other)

    def __rmul__(self, other):
        return self.__mul__(other)

    # def __str__(self):
    #     return "({: .5f}, {: 6.1f})".format(self.norm(), self.arg())
        # return "({: .3f}, {: .3f})".format(self.x, self.y)

    def __repr__(self):
        return "Vector2({0}, {1})\t norm = {2}\t arg = {3}".format(self.x, self.y, self.norm(), self.angle())

    
    # Robot velocity vector length in robot frame
    # Robot pose in world frame
    def norm(self):
        """Return the norm of the vector."""
        # print('x and y', self.x, self.y)
        return math.sqrt(pow(self.x, 2) + pow(self.y, 2))

    # Robot heading with velocity vector length in robot frameOR 
    # angle between robot position and origin with position vector in workd frame
    def angle(self):
        # print('x and y for angle ', self.x, self.y)
        """Return the angle of the vector."""
        return math.degrees(math.atan2(self.y, self.x))

    # Velocity vectors in robot frame
    def set_mag(self, value):
        """Set vector's magnitude without changing direction."""
        if self.norm() == 0:
            logging.warning('Trying to set magnitude for a null-vector! Angle will be set to 0!')
            self.x = 1
            self.y = 0
        else:
            self.normalize()
        self.x *= value
        self.y *= value

        # velocity vectors.
    def normalize(self, ret=False):
        """Normalize the vector."""
        d = self.norm()
        if d:
            if not ret:
                self.x /= d
                self.y /= d
            else:
                return Vector2(self.x / d, self.y / d)

    # angle setting in robot frame with velocity vector
    def set_angle(self, value):
        """Set vector's direction without changing magnitude."""
        if self.norm() == 0:
            logging.warning('Trying to set angle for a null-vector! Magnitude will be set to 1!')
            self.x = 1
            self.y = 0
        delta = angle_diff(self.angle(), value)
        self.rotate(delta)

    # veclocity vectors rotation.
    def rotate(self, angle_rad):
        """Rotate vector by degrees specified in value."""
        angle_rad = math.radians(angle_rad)
        # print(angle_rad)
        self.x, self.y = math.cos(angle_rad) * self.x - math.sin(angle_rad) * self.y, \
                math.sin(angle_rad) * self.x + math.cos(angle_rad) * self.y


    # veclocity vectors rotation.
    def rotate_with_dir(self, direction, angle_rad):
        """Rotate vector by degrees specified in value."""
        angle_rad = math.radians(angle_rad)
        # print(angle_rad)
        if direction == "clockwise":

            self.x, self.y = math.cos(angle_rad) * self.x + math.sin(angle_rad) * self.y, \
                            -math.sin(angle_rad) * self.x + math.cos(angle_rad) * self.y
        else: # Counterclockwise
            self.x, self.y = math.cos(angle_rad) * self.x - math.sin(angle_rad) * self.y, \
                    math.sin(angle_rad) * self.x + math.cos(angle_rad) * self.y

        # print(self.x, self.y)

    # velocity vectors.

    def upper_limit(self, value):
        """Limit vector's maximum magnitude to given value."""
        if self.norm() > value:
            self.set_mag(value)

    # velocity vectors.
    def limit_lower(self, value):
        """Limit vector's minimum magnitude to given value."""
        if self.norm() < value:
            self.set_mag(value)

    # def constrain(self, old_value, max_value): # old value is angle of the agent
    #     """Limit vector's change of direction to max_value from old_value."""
    #     desired_value = self.angle()
    #     delta = angle_diff(old_value, desired_value)
    #     if abs(delta) > max_value:
    #         value = angle_diff(desired_value, old_value + math.copysign(max_value, delta))
    #         self.rotate(value)


def angle_diff(from_angle, to_angle):
    diff = (to_angle - from_angle) % 360
    if diff >= 180:
        diff -= 360
    return diff

# position vectors
def pose_vector_dist(pose1, pose2):
    """Return Euclidean distance between two ROS poses."""
    x1 = pose1.x
    y1 = pose1.y
    x2 = pose2.x
    y2 = pose2.y

    return math.sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2))


def pose_dist(pose1, pose2):
    """Return Euclidean distance between two ROS poses."""
    x1 = pose1.position.x
    y1 = pose1.position.y
    x2 = pose2.position.x
    y2 = pose2.position.y

    return math.sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2))

### ROS related functions and take the raw message

# take input the whole message from ros and return a vector of class Vector2
def get_agent_velocity(agent):
    """Return agent velocity as Vector2 instance."""
    vel = Vector2()
    vel.x = agent.twist.twist.linear.x
    vel.y = agent.twist.twist.linear.y
    return vel

# take input the whole message from ros and return a vector of class Vector2
def get_agent_position(agent):
    """Return agent position as Vector2 instance."""
    pos = Vector2()
    pos.x = agent.pose.pose.position.x
    pos.y = agent.pose.pose.position.y
    return pos

def get_agent_position_list(agent):
    """Return agent position as Vector2 instance."""
    pos = Vector2()
    pos.x = agent.pose.pose.position.x
    pos.y = agent.pose.pose.position.y
    return [pos.x, pos.y]


""" Test the Vector2 class and its methods """


# if __name__ == "__main__":
#     # Create two vector instances
    # v1 = Vector2(-0.6, 0)
#     v2 = Vector2(1, 2)

#     # Demonstrate addition
#     v3 = v1 + v2
#     print(f"Addition: v1+ v2 = {v3.x, v3.y}")

#     # Demonstrate subtraction
#     v4 = v1 - v2
#     print(f"Subtraction: v1 - v2 = {v4.x, v3.y}")

#     # Demonstrate multiplication by scalar
#     v6 = v1 * 3
#     print(f"Multiplication by scalar: v1 * 3 = {v6.x, v3.y}")

#     # Demonstrate norm
    # print(f"Norm of v1: {v1.norm()}")

#     # Demonstrate angle
#     print(f"Angle of v1: {v1.angle()} in degrees")

#     # Demonstrate setting magnitude
#     v1.set_mag(5)
#     print(f"Setting magnitude of v1 to 5: {v1.x, v1.y}")

#     # Demonstrate normalization
#     v1.normalize()
#     print(f"Normalization of v1: {v1.__repr__()}")

#     # Demonstrate setting angle
#     v1.set_angle(45)
#     print(f"Setting angle of v1 to 45 degrees: {v1.angle()}")

#     # Demonstrate rotating vector
#     v1.rotate_with_dir("clockwise", 90)
#     print(f"Rotating v1 clockwise by 90 degrees: {v1.angle()}")

#     # Demonstrate upper limit
#     v1.x, v1.y = 3, 4
#     v1.upper_limit(2)
#     print(f"Applying upper limit of 2 to v1: {v1.__repr__()}")

#     # Demonstrate lower limit
#     v1.limit_lower(1)
#     print(f"Applying lower limit of 1 to v1: {v1.__repr__()}")

#     # angle diff 
#     print(f"Angle diff: {angle_diff(10, 350)}")

#     # pose vector dist 
#     pose1 = Vector2(1, 1)
#     pose2 = Vector2(2, 2)
#     print(f"Pose vector euclidean distance: {pose_vector_dist(pose1, pose2)}")



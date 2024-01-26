#!/usr/bin/env python3

#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped , PoseArray
from geometry_msgs.msg import Point, Twist
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry 


import rospy
from geometry_msgs.msg import PoseArray, Pose
from geometry_msgs.msg import Twist
import math 
import rospy
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker


import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist


class VelocityVisualizer:
    def __init__(self):
        rospy.init_node('velocity_visualizer', anonymous=True)

        rospy.Subscriber('/robot_0/cmd_vel', Twist, self.velocity_callback_0)
        rospy.Subscriber('/robot_1/cmd_vel', Twist, self.velocity_callback_1)
        rospy.Subscriber('/robot_2/cmd_vel', Twist, self.velocity_callback_2)
        rospy.Subscriber('/robot_3/cmd_vel', Twist, self.velocity_callback_3)
        rospy.Subscriber('/robot_4/cmd_vel', Twist, self.velocity_callback_4)
        rospy.Subscriber('/robot_5/cmd_vel', Twist, self.velocity_callback_5)
        rospy.Subscriber('/robot_6/cmd_vel', Twist, self.velocity_callback_6)
        rospy.Subscriber('/robot_7/cmd_vel', Twist, self.velocity_callback_7)
        rospy.Subscriber('/robot_8/cmd_vel', Twist, self.velocity_callback_8)
        rospy.Subscriber('/robot_9/cmd_vel', Twist, self.velocity_callback_9)

        self.marker_pub_0 = rospy.Publisher('/0velocity_arrow_marker', Marker, queue_size=10)
        self.marker_pub_1 = rospy.Publisher('/1velocity_arrow_marker', Marker, queue_size=10)
        self.marker_pub_2 = rospy.Publisher('/2velocity_arrow_marker', Marker, queue_size=10)
        self.marker_pub_3 = rospy.Publisher('/3velocity_arrow_marker', Marker, queue_size=10)
        self.marker_pub_4 = rospy.Publisher('/4velocity_arrow_marker', Marker, queue_size=10)
        self.marker_pub_5 = rospy.Publisher('/5velocity_arrow_marker', Marker, queue_size=10)
        self.marker_pub_6 = rospy.Publisher('/6velocity_arrow_marker', Marker, queue_size=10)
        self.marker_pub_7 = rospy.Publisher('/7velocity_arrow_marker', Marker, queue_size=10)
        self.marker_pub_8 = rospy.Publisher('/8velocity_arrow_marker', Marker, queue_size=10)
        self.marker_pub_9 = rospy.Publisher('/9velocity_arrow_marker', Marker, queue_size=10)

        self.rate = rospy.Rate(10)  # Set the publishing rate

    def velocity_sender(self, msg, id, base_footprint, pub):


        vel_x = msg.linear.x
        vel_y = msg.linear.y
        
        # print(msg)
        # vel_angle = 

        marker = Marker()
        marker.header.frame_id = base_footprint 
        marker.header.stamp = rospy.Time.now()
        marker.ns = "velocity_arrow"
        marker.id = id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0

        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        points = [
            Point(x=0.0, y=0.0, z=0.0),
            # Point(x=1.0, y=1.0, z=0.0)
            Point(x=vel_x, y=vel_y, z=0.0)
        ]

        # point = Point()
        # point.x = vel_x
        # point.y = vel_y
        marker.points = points

        marker.scale.x = 0.05  # Set arrow length based on linear velocity x component
        marker.scale.y = 0.1  # Set arrow width
        marker.scale.z = 1.0  # Set arrow height
        
        marker.color.a = 1.0
        marker.color.r = 1.0  # Arrow color (red)
        marker.color.g = 0.0
        marker.color.b = 0.0

        # Publish the Marker message
        pub.publish(marker)


    def velocity_callback_0(self, msg):
        id = 0
        base_footprint = "robot_0/base_footprint"
        pub = self.marker_pub_0
        self.velocity_sender(msg, id, base_footprint, pub)


    def velocity_callback_1(self, msg):
        id = 1
        base_footprint = "robot_1/base_footprint"
        pub = self.marker_pub_1
        self.velocity_sender(msg, id, base_footprint, pub)

    def velocity_callback_2(self, msg):
        id = 2
        base_footprint = "robot_2/base_footprint"
        pub = self.marker_pub_2
        self.velocity_sender(msg, id, base_footprint, pub)

    def velocity_callback_3(self, msg):
        id = 3
        base_footprint = "robot_3/base_footprint"
        pub = self.marker_pub_3
        self.velocity_sender(msg, id, base_footprint, pub)

    def velocity_callback_4(self, msg):
        id = 4
        base_footprint = "robot_4/base_footprint"
        pub = self.marker_pub_4
        self.velocity_sender(msg, id, base_footprint, pub)


    def velocity_callback_5(self, msg):
        id = 5
        base_footprint = "robot_5/base_footprint"
        pub = self.marker_pub_5
        self.velocity_sender(msg, id, base_footprint, pub)


    def velocity_callback_6(self, msg):
        id = 6
        base_footprint = "robot_6/base_footprint"
        pub = self.marker_pub_6
        self.velocity_sender(msg, id, base_footprint, pub)


    def velocity_callback_7(self, msg):
        id = 7
        base_footprint = "robot_7/base_footprint"
        pub = self.marker_pub_7
        self.velocity_sender(msg, id, base_footprint, pub)


    def velocity_callback_8(self, msg):
        id = 8
        base_footprint = "robot_8/base_footprint"
        pub = self.marker_pub_8
        self.velocity_sender(msg, id, base_footprint, pub)


    def velocity_callback_9(self, msg):
        id = 9
        base_footprint = "robot_9/base_footprint"
        pub = self.marker_pub_9
        self.velocity_sender(msg, id, base_footprint, pub)

if __name__ == '__main__':
    try:
        VelocityVisualizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

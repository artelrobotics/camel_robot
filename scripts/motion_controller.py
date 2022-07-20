#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from rospy.client import get_param, has_param
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import time
from math import radians, sqrt, pi , degrees 
import tf
from tf.transformations import euler_from_quaternion
from math import radians, degrees, pi


class MotionController:
    def __init__(self) -> None:
        rospy.init_node('motion_control_node', anonymous=True)
        self.vel_publisher = rospy.Publisher('/camel_amr_1000_001/cmd_vel', Twist, queue_size=1)
        self.rate = rospy.Rate(10)
        self.odometry_val = Odometry()
        self.cmd_val = Twist()
        rospy.Subscriber('/camel_amr_1000_001/odometry', Odometry, self.odometry_callback)

    def odometry_callback(self, msg: Odometry):
        self.odometry_val = msg

    def move(self):
        pass

    def turn(self):
        pass

    def stop(self):
        self.cmd_val = Twist()
        self.vel_publisher.publish(self.cmd_val)

    pass
#!/usr/bin/env python
import imp
import rospy
from geometry_msgs.msg import Twist
from rospy.client import get_param, has_param
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import time
from camel_robot.srv import light, sound 
from roboteq_motor_controller_driver.srv import emergency_stop_srv
from math import radians, sqrt, pi , degrees 
from tf.transformations import euler_from_quaternion
from math import radians, degrees, pi
from motion_control import CamelControl
from std_srvs.srv import SetBool, SetBoolResponse
from ar_track_alvar_msgs.msg import AlvarMarkers


class Docking:

    def __init__(self):
        self.robot_controller = CamelControl()
        rospy.Subscriber('/camel_amr_1000_001/ar_pose_marker', AlvarMarkers, self.aruco_callback)
        self.ar_markers = []

    def aruco_callback(self, ar_msg):
        for ar_marker in ar_msg.markers:
            if not ar_marker.id in [marker.id for marker in self.ar_markers]:
                self.ar_markers.append(ar_marker)

if __name__ == "__main__":
    rospy.init_node('robot_docking_node', anonymous=True)
    docking = Docking()
    rospy.spin()
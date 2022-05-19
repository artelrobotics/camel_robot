#!/usr/bin/env python
import rospy
from motion_control import CamelControl
import time

cc = CamelControl()
while rospy.is_shutdown:

    aruco_pose = cc.aruco_pose()
    #print(a)
    odom = cc.get_odom()
    print(odom)
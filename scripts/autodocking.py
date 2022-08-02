#!/usr/bin/env python
from this import d
from turtle import circle
from scipy import empty
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
import PyKDL
from scipy import empty
import math
from obstacle_detector.msg import Obstacles
from roboteq_motor_controller_driver.srv import emergency_stop_srv
import rosparam
from ds4_driver.msg import Status
from std_msgs.msg import String
import dynamic_reconfigure.client
from dynamic_reconfigure import DynamicReconfigureCallbackException
from camel_robot.srv import light

import actionlib

from camel_robot.msg import AutoDockingAction, AutoDockingFeedback, AutoDockingResult

class Docking():

    def __init__(self):
        self.state = False
        self.status = ""
        self.scan_obstacle = True
        self.stop = False
        self.charging = False
        self.yaw = 30
        self.station_yaw = 30
        self.cmd_vel = Twist()
        self.rc = CamelControl()
        self.result = light()
        self.gc_dr_client = dynamic_reconfigure.client.Client("/camel_amr_1000_001/move_base/global_costmap")
        self.lc_dr_client = dynamic_reconfigure.client.Client("/camel_amr_1000_001/move_base/local_costmap")
        self.sound_service = rospy.ServiceProxy("/camel_amr_1000_001/sound_server", sound)
        self.hook_service = rospy.ServiceProxy("common/hooks_ctrl", SetBool)
        self.emergency_service = rospy.ServiceProxy("/camel_amr_1000_001/driver/emergency_stop_service", emergency_stop_srv)
        #rospy.Subscriber("/camel_amr_1000_001/status", Status, self.cb_status, queue_size=1)
        self.rate = rospy.Rate(30)
        self.vel_publisher = rospy.Publisher('/camel_amr_1000_001/cmd_vel', Twist, queue_size=1)
    
        self._as = actionlib.SimpleActionServer('Docking', AutoDockingAction, execute_cb=self.on_goal, auto_start=False)
        self._as.start()
        # Subscribing to the topics Odometry and Scan
        
        rospy.Subscriber('/camel_amr_1000_001/scan', LaserScan, self.laser_callback)
        
        rospy.Subscriber('/camel_amr_1000_001/front/ar_pose_marker', AlvarMarkers, self.front_aruco_callback)
        rospy.Subscriber('/camel_amr_1000_001/back/ar_pose_marker', AlvarMarkers, self.back_aruco_callback)
        rospy.Subscriber('/camel_amr_1000_001/raw_obstacles', Obstacles, self.obstacles_callback)
        rospy.Subscriber('/camel_amr_1000_001/sound_state', String, self.sound_callback)
        rospy.Subscriber('/camel_amr_1000_001/cmd_vel', Twist, self.cmd_callback)
       
    def sound_callback(self, msg):
        self.last_sound = msg.data    
        
    def cmd_callback(self, msg):
        
        
        if self.stop == True:
            self.cmd_vel.linear.x = 0
            self.cmd_vel.angular.z = 0
            self.vel_publisher.publish(self.cmd_vel)

    def laser_callback(self, msg):
        """ Laser scan callback: recives Laser Scan
            ranges[0,2879] 
            return : None
        """         
        self.laser_msg = msg

 
                        
    def obstacles_callback(self, msg):
        count = 0
        length = 0
        # if(len(msg.circles) and self.scan_obstacle == True):

        #     for i in range(len(msg.circles) - 1):
        #         for j in range(i+1, len(msg.circles)):
        #             if msg.circles[i].center.x > 0.3 and msg.circles[j].center.x > 0.3 and (abs(msg.circles[i].center.x - msg.circles[j].center.x) < 0.25) and (abs(msg.circles[i].center.y + msg.circles[j].center.y) < 0.4):     
        
        #                 yaw = abs(msg.circles[i].center.y) - abs(msg.circles[j].center.y) - 0.04
        #                 self.rc.cmd.angular.z = - 0.2 * yaw
                    
        #             else:
        #                 self.rc.cmd.angular.z = 0

        if self.scan_obstacle == True and len(msg.circles):
            if (len(msg.circles) == 2 and self.scan_obstacle == True):
                yaw = abs(msg.circles[0].center.y) - abs(msg.circles[1].center.y) + 0.05
                self.rc.cmd.angular.z = - 0.2 * yaw
                        
            else:
                self.rc.cmd.angular.z = 0


        elif self.state == True:
            count_stop = 0
            count_run = 0
            for object in msg.circles:   
                length = len(msg.circles)
                x = object.center.x
                y = object.center.y
            

                if x < 1.25 and y < 0.65 and y > -0.65 and x > -1.25 and self.stop == False:
                    count_stop = count_stop + 1
                
                if x < 1.35 and y < 0.75 and y > -0.75 and x > -1.35 and self.stop == False:
                    count_run = count_run + 1
            
            if count_stop > 0 and self.stop == False:    
                self.stop = True
                
            elif count_run == length - 1 and self.stop == True:
                self.stop = False
                
    def scan_orientation(self):
        min_l =  min(self.laser_msg.ranges[720:1440])
        min_r =  min(self.laser_msg.ranges[0:720])

    def turn(self, angular_speed):
        self.cmd_vel.linear.x = 0
        self.cmd_vel.angular.z = angular_speed

        self.vel_publisher.publish(self.cmd_vel)
    
    def quat_to_degree(self, quat):
        rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
        return degrees(rot.GetRPY()[1])
    
    def front_aruco_callback(self, data):
        if self.state == True:
            if len(data.markers) and data.markers[0].id == 1:
                
                self.pose_y = data.markers[0].pose.pose.position.x 
                #self.pose_z = data.markers[0].pose.pose.position.y
                self.pose_x = data.markers[0].pose.pose.position.z
                
                orintation = data.markers[0].pose.pose.orientation
                self.yaw = self.quat_to_degree(orintation)

    
    
    
    def back_aruco_callback(self, data):
        
            if len(data.markers) and data.markers[0].id == 2:
                
                self.station_pose_y = data.markers[0].pose.pose.position.x
                #self.pose_z = data.markers[0].pose.pose.position.y
                self.station_pose_x = data.markers[0].pose.pose.position.z
                
                orintation = data.markers[0].pose.pose.orientation
                self.station_yaw = self.quat_to_degree(orintation)
                
                #print("y_pose : " , self.station_pose_y,"x_pose: ", self.station_pose_x, "yaw: ", self.station_yaw)

    def charging_yaw_correction(self, tolerance):
        
        while(abs(self.station_yaw) > tolerance and self.preempted == False):
            if self._as.is_preempt_requested():
                self.preempted = True
                break
            angular_speed =  0.01 * self.station_yaw 
            self.turn(angular_speed)
        self.rc.stop_robot()
    
    def charging_y_pose_correction(self):
        yaw_correction_task = False
        
        while(yaw_correction_task == False and self.preempted == False):
            if self._as.is_preempt_requested():
                self.preempted = True
                break
            summ_pose_y = 0
            for x in range(20):
                summ_pose_y  = self.station_pose_y + summ_pose_y
                time.sleep(0.2)
            avg_pose_y = summ_pose_y / 20
            if abs(avg_pose_y ) <= 0.1 :
                yaw_correction_task = True
                print("average distance_y  1:" , avg_pose_y)
                
            elif abs(avg_pose_y) > 0.1 :
                distance = avg_pose_y
                print("average distance_y 2:" , distance)
                if(distance < 0):
                    self.rc.rotate(-90)
                    self.rc.move(-abs(distance))
                    self.rc.rotate(90)
                    self.charging_yaw_correction(0.3)
                else:
                    self.rc.rotate(90)
                    self.rc.move(-abs(distance))
                    self.rc.rotate(-90)
                    self.charging_yaw_correction(0.3)
                avg_pose_y = 0
    
    def move_charger(self):
        print("move charger")
        while(abs(self.station_pose_x) > 1.3 and self.preempted == False):
            if self._as.is_preempt_requested():
                self.preempted = True
                break
            self.cmd_vel.linear.x = -0.1
           
            self.cmd_vel.angular.z = - 0.5 * math.atan2(self.station_pose_y, abs(self.station_pose_x))
            self.vel_publisher.publish(self.cmd_vel)
        
        self.rc.stop_robot()

    def yaw_correction(self, tolerance):
        self.status = "Yaw_Correction"
        self.send_feedback()
        while(abs(self.yaw) > tolerance and self.preempted == False):
            if self._as.is_preempt_requested():
                self.preempted = True
                break
            
            angular_speed = - 0.01 * self.yaw 
            self.turn(angular_speed)
        self.rc.stop_robot()
        
    def y_pose_correction(self):
        self.status = "Y_Pose_Correction"
        self.send_feedback()
        
        yaw_correction_task = False
        while(yaw_correction_task == False and self.preempted == False):
            if self._as.is_preempt_requested():
                self.preempted = True
                break
            
            summ_pose_y = 0
            for x in range(20):
                summ_pose_y  = self.pose_y + summ_pose_y
                time.sleep(0.2)
            avg_pose_y = summ_pose_y / 20
            #rospy.logerr(avg_pose_y)
            if abs(avg_pose_y ) <= 0.1 :
                yaw_correction_task = True
                #rospy.logerr(avg_pose_y)
            elif abs(avg_pose_y) > 0.1 :
                distance = avg_pose_y
                # print("average distance_y :" , distance)
                if(distance > 0):
                    self.rc.rotate(-90)
                    self.rc.move(abs(distance))
                    self.rc.rotate(90)
                    self.yaw_correction(0.3)
                else:
                    self.rc.rotate(90)
                    self.rc.move(abs(distance))
                    self.rc.rotate(-90)
                    self.yaw_correction(0.3)
                avg_pose_y = 0
    
    def move_forward(self):
        self.status = "Move_Forward"
        self.send_feedback()
        while(abs(self.pose_x) > 2.13 and self.preempted == False):
            if self._as.is_preempt_requested():
                self.preempted = True
                break
            self.cmd_vel.linear.x = 0.1
            #self.cmd_vel.angular.z = - 0.01 * self.yaw 
            self.cmd_vel.angular.z = 0.5 * math.atan2(self.pose_y, self.pose_x)
            self.vel_publisher.publish(self.cmd_vel)
        
        self.rc.stop_robot()

    #def scan_correction(self):
    def send_feedback(self):
        feedback = AutoDockingFeedback()
        feedback.feedback = self.status
        self._as.publish_feedback(feedback)

    def on_goal(self, command):
        self.success = False
        self.preempted = False
        invalid_parameters = False
        self.message = ""
        self.result.result = AutoDockingResult()
        
        
        self.yaw = 30
        self.station_yaw = 30
        self.state = True
        self.scan_obstacle = False
        
        if(command.goal == "loading"):
            self.yaw_correction(1)
            self.y_pose_correction()
            self.move_forward()
            self.yaw_correction(0.3)
            self.scan_obstacle = True
            self.rc.move(2.45)
            self.hook_service(True)
            self.scan_obstacle = False
            self.rc.move(0.3)
            try:
                self.gc_dr_client.update_configuration({'footprint': '[[-1.19,-0.9],[-1.19,0.9],[1.19,0.9],[1.19,-0.9]]'})
                time.sleep(1)
                self.lc_dr_client.update_configuration({'footprint': '[[-1.19,-0.9],[-1.19,0.9],[1.19,0.9],[1.19,-0.9]]'})
                time.sleep(1)
            except DynamicReconfigureCallbackException as e:
                rospy.logerr(e)
                rospy.logerr("footprint not changed")
            finally:
                self.gc_dr_client.update_configuration({'footprint': '[[-1.19,-0.9],[-1.19,0.9],[1.19,0.9],[1.19,-0.9]]'})
                time.sleep(1)
                self.lc_dr_client.update_configuration({'footprint': '[[-1.19,-0.9],[-1.19,0.9],[1.19,0.9],[1.19,-0.9]]'})
                time.sleep(1)
            self.state = False
         
            
            if self.preempted:
                rospy.loginfo("Preemted")
                self.result.result = "Preemted" 
                self._as.set_preempted(self.result)
            elif self.success:
                rospy.loginfo("Success")
                self.result.result = "Success"
                self._as.set_succeeded(self.result)
            else:
                rospy.loginfo("Aborted")
                self.result.result = "Aborted"
                self._as.set_aborted(self.result)
                
        
        
        elif(command.goal == "unloading"):
            self.hook_service(False)
            self.rc.move(-2.5)
            self.rc.rotate(180)
            try:
                self.gc_dr_client.update_configuration({'footprint': '[[-1.19,-0.65],[-1.19,0.65],[1.19,0.65],[1.19,-0.65]]'})
                time.sleep(1)
                self.lc_dr_client.update_configuration({'footprint': '[[-1.19,-0.65],[-1.19,0.65],[1.19,0.65],[1.19,-0.65]]'})
                time.sleep(1)
            except DynamicReconfigureCallbackException as e:
                rospy.logerr(e)
                rospy.logerr("footprint not changed")
            finally:
                self.gc_dr_client.update_configuration({'footprint': '[[-1.19,-0.65],[-1.19,0.65],[1.19,0.65],[1.19,-0.65]]'})
                time.sleep(1)
                self.lc_dr_client.update_configuration({'footprint': '[[-1.19,-0.65],[-1.19,0.65],[1.19,0.65],[1.19,-0.65]]'})
                time.sleep(1)
            self.state = False
            

        elif(command.goal == "charging"):
            self.charging = True
            self.charging_yaw_correction(1)
            self.charging_y_pose_correction()
            self.charging_yaw_correction(0.3)
            self.move_charger()
            self.charging_yaw_correction(0.3)
            self.charging = False
            self.state = False
            
            
        elif(command.goal == "discharging"):
            self.rc.move(1)
            self.state = False
            
        else:
            invalid_parameters = True
            message = "Invalid command !!!"

        


if __name__ == '__main__':
    # Initialize Node 
    rospy.init_node('robot_docking_node', anonymous=True)
    robotdocking = Docking()
    rospy.spin()
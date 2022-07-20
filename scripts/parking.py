#!/usr/bin/env python
from this import d

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


class Docking():

    def __init__(self):
        self.state = False
        self.scan_obstacle = True
        self.stop_robot = False
        self.sound_service = rospy.ServiceProxy("/camel_amr_1000_001/sound_server", sound)
        self.hook_service = rospy.ServiceProxy("common/hooks_ctrl", SetBool)
        self.emergency_service = rospy.ServiceProxy("/camel_amr_1000_001/driver/emergency_stop_service", emergency_stop_srv)
        rospy.Subscriber("/camel_amr_1000_001/status", Status, self.cb_status, queue_size=1)
        self.rate = rospy.Rate(30)
        self._as = rospy.Service('Docking_server',SetBool, handler=self.docking_command)
        # Publishing Twist msgs to /cmd_vel topic
        self.rc = CamelControl()
        self.yaw = 30
        self.cmd_vel = Twist()
        time.sleep(3)
        self.vel_publisher = rospy.Publisher('/camel_amr_1000_001/cmd_vel', Twist, queue_size=1)
        
        # Subscribing to the topics Odometry and Scan
        # rospy.Subscriber('/camel_amr_1000_001/odometry', Odometry, self.odom_callback)
        rospy.Subscriber('/camel_amr_1000_001/scan', LaserScan, self.laser_callback)
        # rospy.Subscriber('/camel_amr_1000_001/cmd_vel', Twist, self.cmd_callback)

        rospy.Subscriber('/camel_amr_1000_001/ar_pose_marker', AlvarMarkers, self.aruco_callback)

        rospy.Subscriber('/camel_amr_1000_001/raw_obstacles', Obstacles, self.obstacles_callback)
  
       
        
        
    
    def laser_callback(self, msg):
        """ Laser scan callback: recives Laser Scan
            ranges[0,2879] 
            return : None
        """         
        self.laser_msg = msg

    def cb_status(self, msg):
        if self.stop_robot == True:
            self.cmd_vel.linear.x = 0
            self.cmd_vel.angular.z = 0
                        
    def obstacles_callback(self, msg):
        if self.state == True :

            for object in msg.circles:
               
                x = object.center.x
                y = object.center.y

                if x < 1.25 and y < 0.7 and y > -0.7 and x > -1.25 and self.stop_robot == False:
                    self.sound_service("warning")
                    self.stop_robot = True
                    
                    
                else:
                    self.sound_service("sound_stop")
                    self.stop_robot = False
                    time.sleep(2)

            if(len(msg.circles) == 2 and self.scan_obstacle == True):
                if (abs((msg.circles[0].center.x) - msg.circles[1].center.x) < 0.2):
                    yaw = abs(msg.circles[0].center.y) - abs(msg.circles[1].center.y) + 0.05
                    self.rc.cmd.angular.z = - 0.2 * yaw
                    print(yaw)
                else:
                    self.rc.cmd.angular.z = 0
           
            


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
    
    def aruco_callback(self, data):
        if self.state == True:
            if len(data.markers) :
                self.pose_y = data.markers[0].pose.pose.position.x
                #self.pose_z = data.markers[0].pose.pose.position.y
                self.pose_x = data.markers[0].pose.pose.position.z
                
                orintation = data.markers[0].pose.pose.orientation
                self.yaw = self.quat_to_degree(orintation)
                
                print("y_pose : " , self.pose_y,"x_pose: ", self.pose_x)

                angle = math.atan2(self.pose_y , self.pose_x)
                # print("Angle: ", angle)
            #print("yaw : " , self.yaw)
            #print(getattr(orintation, "x"))
            # for marker in data.markers:
            #     if marker.id == 1:
            #         print(marker)

    def yaw_correction(self, tolerance):
        print(tolerance)
        while(abs(self.yaw) > tolerance):
                angular_speed = - 0.01 * self.yaw 
                self.turn(angular_speed)
        self.rc.stop_robot()
        # print("Yaw Correction is Succesfull")        

    def y_pose_correction(self):
        yaw_correction_task = False
        
        while(yaw_correction_task == False):
            summ_pose_y = 0
            for x in range(10):
                summ_pose_y  = self.pose_y + summ_pose_y
                time.sleep(0.1)
            avg_pose_y = summ_pose_y / 10
            if abs(avg_pose_y ) <= 0.07 :
                yaw_correction_task = True
            
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
        while(abs(self.pose_x) > 2.05):
            self.cmd_vel.linear.x = 0.1
            #self.cmd_vel.angular.z = - 0.01 * self.yaw 
            self.cmd_vel.angular.z = 0.5 * math.atan2(self.pose_y, self.pose_x)
            self.vel_publisher.publish(self.cmd_vel)
        self.rc.stop_robot()

    #def scan_correction(self):



    def docking_command(self, command):
        self.state = True
        self.scan_obstacle = False
        if(command.data == True):
            self.yaw_correction(1)
            self.y_pose_correction()
            self.move_forward()
            self.yaw_correction(0.3)
            print("self.scan ", self.scan_obstacle)
            self.scan_obstacle = True
            self.rc.move(2.3)
            self.hook_service(True)
            #rospy.set_param('/camel_amr_1000_001/move_base/footprint', [[-1.19,-0.83],[-1.19,0.83],[1.19,0.83],[1.19,-0.83]])
            #rospy.set_param('/camel_amr_1000_001/move_base/local_costmap/footprint', "[[-1.19,-0.83],[-1.19,0.83],[1.19,0.83],[1.19,-0.83]]")
            #rospy.set_param('/camel_amr_1000_001/move_base/global_costmap/footprint', "[[-1.19,-0.83],[-1.19,0.83],[1.19,0.83],[1.19,-0.83]]")
            rospy.set_param('/camel_amr_1000_001/move_base/global_costmap/footprint_padding', 0.27)
            rospy.set_param('/camel_amr_1000_001/move_base/local_costmap/footprint_padding', 0.27)
            rospy.set_param('/camel_amr_1000_001/move_base/footprint_padding', 0.27)
            self.state = False
            return SetBoolResponse(True, "Docking is succesfull")
        
        elif(command.data == False):
            self.scan_obstacle = True
            self.hook_service(False)
            self.rc.move(-2.5)
            self.rc.rotate(180)
            #rospy.set_param('/camel_amr_1000_001/move_base/footprint', [[-1.19,-0.65],[-1.19,0.65],[1.19,0.65],[1.19,-0.65]])
            #rospy.set_param('/camel_amr_1000_001/move_base/local_costmap/footprint', "[[-1.19,-0.65],[-1.19,0.65],[1.19,0.65],[1.19,-0.65]]")
            #rospy.set_param('/camel_amr_1000_001/move_base/global_costmap/footprint', "[[-1.19,-0.65],[-1.19,0.65],[1.19,0.65],[1.19,-0.65]]")
            rospy.set_param('/camel_amr_1000_001/move_base/global_costmap/footprint_padding', 0.01)
            rospy.set_param('/camel_amr_1000_001/move_base/local_costmap/footprint_padding', 0.01)
            rospy.set_param('/camel_amr_1000_001/move_base/footprint_padding', 0.01)
            self.state = False
            return SetBoolResponse(True, "Undocking is succesfull")

        
        



        




if __name__ == '__main__':
    # Initialize Node 
    rospy.init_node('robot_docking_node', anonymous=True)
    
    robotdocking = Docking()
   
    

    
    rospy.spin()
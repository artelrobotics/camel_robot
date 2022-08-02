#!/usr/bin/env python
from logging import shutdown
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
from ds4_driver.msg import Status, Feedback
import PyKDL
from obstacle_detector.msg import Obstacles 
from std_msgs.msg import String

class CamelControl():

    def __init__(self):
        # Publishing Twist msgs to /cmd_vel topic
        self.vel_publisher = rospy.Publisher('/camel_amr_1000_001/cmd_vel', Twist, queue_size=1)
        
        # Subscribing to the topics Odometry and Scan
        #rospy.Subscriber('/camel_amr_1000_001/odometry', Odometry, self.odom_callback)
        rospy.Subscriber('/camel_amr_1000_001/encoder/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/camel_amr_1000_001/scan', LaserScan, self.laser_callback)
        
        #rospy.Subscriber('/camel_amr_1000_001/set_feedback', Feedback, self.obstacle_avoidance_callback)
        rospy.Subscriber('/camel_amr_1000_001/sound_state', String, self.sound_callback)
       
        #Initialize Service
        self.light_service = rospy.ServiceProxy("/camel_amr_1000_001/common/Light_server", light)
        self.sound_service = rospy.ServiceProxy("/camel_amr_1000_001/sound_server", sound)
        self.emergency_service = rospy.ServiceProxy("/camel_amr_1000_001/driver/emergency_stop_service", emergency_stop_srv)
        frequency = rospy.get_param('frequency', 20) # Get frequency in Hz default '10'
        self.rate = rospy.Rate(20)  # Rate in Hz
              
  
       
      
      
        # Get angular tolarance for rotate function default " 1 degree"
      
        self.laser_msg = LaserScan()
        rospy.on_shutdown(self.shutdownhook)
        self.ctrl_c = False
        self.cmd = Twist()
        self.move_motion_threshold = 0.4
        self.rotate_motion_threshold = 0.2
        self.warning = False
        #-----------------------
        self.cmd_forward = False
        self.cmd_backward = False
        self.cmd_vel = Twist()
        time.sleep(1)
    
    def sound_callback(self, msg):
        self.last_sound = msg.data    
    


    def publish_once_in_cmd_vel(self):
        """
        This is because publishing in topics sometimes fails the first time you publish.
        In continuos publishing systems there is no big deal but in systems that publish only
        once it IS very important.
        """
        while not self.ctrl_c:
            connections = self.vel_publisher.get_num_connections()
        
            if connections > 0 :
                self.vel_publisher.publish(self.cmd)
                break
            else:
                self.rate.sleep()
    
    


    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        self.ctrl_c = True

    def laser_callback(self, msg):
        """ Laser scan callback: recives Laser Scan
            ranges[0,2879] 
            return : None
        """       
        self.laser_msg = msg

    def odom_callback(self, msg):
        """ Odometry callback : recives Odometry data
            Position : x, y, z
            Orientation : roll, pitch, yaw
            velocity  
        """
        self.position = msg.pose.pose.position
        orientation_list = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w] 
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(orientation_list)
        self.linear_x_velocity = msg.twist.twist.linear.x
        self.angular_z_velocity = msg.twist.twist.angular.z
       
    
    


    # def aruco_pose(self):
    #     try:
    #         self.tf_listener.waitForTransform('/camel_amr_1000_001/base_link', '/ar_marker_1', rospy.Time(), rospy.Duration(1.0))
            
    #     except (tf.Exception, tf.ConnectivityException, tf.LookupException):
    #         pass
        
    #     try:
    #         (self.aruco_trans, self.aruco_rot) = self.tf_listener.lookupTransform('/camel_amr_1000_001/base_link', '/ar_marker_1', rospy.Time(0))
    #         return self.aruco_trans, euler_from_quaternion(self.aruco_rot)
    #     except (tf.Exception, tf.ConnectivityException, tf.LookupException):
    #         rospy.loginfo("TF Exception")
    #         return
        
        
    def get_velocity(self):
        """ Getting velocity data

            Return:
                velocity: linear.x
                velocity: angular.z 
        """
        time.sleep(0.1)
        return self.velocity.linear.x, self.velocity.angular.z

    def get_laser(self, pos):
        """ Get laser function for specific position 
            Recive: position of laser point 
        """
        time.sleep(0.1)
        return self.laser_msg.ranges[pos]

    def get_back_laser(self):
        time.sleep(0.1)
        #print(self.laser_msg.ranges[0])
        return self.laser_msg.ranges[0]
    
    # def obstacle_avoidance_callback(self, data):
    #     """ Obstacle avoidance function scans 360 degree of robot
    #         If obstacle detected in critical distance gives emergance signal
    #         recives : Data from self.laser_msg 
    #         returns : Min distance between obstacle and robot
    #     """
        
    #     if(data.led_g == 1):
    #         #time.slee(0.1)
    #         print("ob_working")
    #         k = 1.3

    #         if self.linear_x_velocity < 0.09:
    #             self.warning = True
    #         else:
    #             self.warning = False

    #         tolerance_lock = 1.3 + k * (self.linear_x_velocity  )
            
    #         # print(any([point < 1.2 for point in self.laser_msg.ranges]) and not self.warning)
    #         #print(tolerance_lock, self.warning, min(self.laser_msg.ranges[1200:1680]))
            
    #         if (any([point < tolerance_lock for point in self.laser_msg.ranges[1280:1600]]) and not self.warning):
    #             self.sound_service("warning")
    #             self.warning = True
    #             self.emergency_service(True)
                

    #         elif all([point > 1.55 for point in self.laser_msg.ranges[1280:1600]]) and self.warning:
    #             self.sound_service("sound_stop")
    #             self.warning = False
    #             #self.emergency_service(False)
                
    #         # regions = {
    #         #     'full':  min(min(self.laser_msg.ranges), 10),
    #         #     }
            
    #         # if regions['full'] < 1.2 and self.warning == False :
    #         #     self.sound_service("warning")
    #         #     self.warning = True    
    #         # elif regions['full'] > 1.4:
    #         #     self.sound_service("sound_stop")
    #         #     self.warning = False
    #         # print(regions['full'])

           
    #     elif(data.led_r == 1):
    #         print("not ")
    #         pass
    #     #self.rate.sleep()
            
    def stop_robot(self):
        # rospy.loginfo("shutdown time! Stop the robot")
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publish_once_in_cmd_vel()
    
   
    def move(self, goal_distance):

        time.sleep(1) # time.sleep for getting data from callback functions
        
        # Set the movement command to a forward or backward
        if goal_distance > 0:
            speed = 0.2
            k = 0.4
        else:
            speed = -0.2
            k = -0.4
        
        # Track the last point measured
        last_point_x = self.position.x
        last_point_y = self.position.y
       
        #Initialize variables for motion tracking
        move_distance = 0
        delta_distance = 0

        # Track how far we have moved
        while abs(move_distance) + 0.005 < abs(goal_distance) and not rospy.is_shutdown():
            
            if(abs(goal_distance) - abs(move_distance) > self.move_motion_threshold):
                self.cmd.linear.x = speed
            else:
                self.cmd.linear.x = k * (abs(goal_distance) - abs(move_distance))
            self.vel_publisher.publish(self.cmd)
            self.rate.sleep()

            # Compute the amount of movement since the last lopp
            delta_distance = sqrt((self.position.x - last_point_x) ** 2 + (self.position.y - last_point_y) ** 2)
            move_distance += delta_distance
            last_point_x = self.position.x
            last_point_y = self.position.y
            
            

        self.stop_robot()

    def rotate(self, degree):
        
        time.sleep(1) # time.sleep for getting data from callback functions
        # Set the movement command to a rotation
        if degree > 0:
            self.cmd.angular.z = -0.2
            k = -0.7
        else:
            self.cmd.angular.z = 0.2
            k = 0.7
        # Track the last angle measured
        last_angle = self.yaw

        # Initialize needed variables for tracking
        turn_angle = 0
        goal_angle  = radians(degree) 
        angular = self.cmd.angular.z

        # Begin the rotation with tracking
        while abs(turn_angle) + 0.07 < abs(goal_angle) and not rospy.is_shutdown():
            #print(degrees(goal_angle) , degrees(turn_angle))
            #print('diff: ', abs(goal_angle) - abs(turn_angle))
            
            # Publish the Twist message and sleep 1 cycle
            if(abs(goal_angle) - abs(turn_angle) > self.rotate_motion_threshold):
                self.cmd.angular.z = angular            
            else:
                self.cmd.angular.z = k * (abs(goal_angle) - abs(turn_angle))
            
            self.vel_publisher.publish(self.cmd)
            self.rate.sleep()

            # Compute the amount of rotation since the last lopp
            rotation = self.normalize_angle(self.yaw)
            delta_angle = self.normalize_angle(rotation - last_angle)

            turn_angle += delta_angle
            last_angle = rotation
          
        self.stop_robot()

    def quat_to_angle(self, quat):
        rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
        return rot.GetRPY()[2]

    def quat_to_degree(self, quat):
        rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
        return degrees(rot.GetRPY()[1])

    def normalize_angle(self, angle):
        res = angle
        while res > pi:
            res -= 2.0 * pi
        while res < -pi:
            res += 2.0 * pi
        return res


if __name__ == '__main__':
    # Initialize Node 
    rospy.init_node('robot_control_node', anonymous=True)
    time.sleep(1)
    robotcontrol = CamelControl()
   

    rospy.spin()
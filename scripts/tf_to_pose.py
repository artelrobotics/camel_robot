#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
import tf
from tf import LookupException, ExtrapolationException

class Tf_to_Pose:
    def __init__(self):
        self.tf_listener = tf.TransformListener()
        self.freq = rospy.get_param("~rate", default=1)
        self.rate = rospy.Rate(self.freq)
        self.frame_id = ""
        self.pose_pub = rospy.Publisher("robot_pose", PoseStamped, queue_size=1)
        self.child_frame = rospy.get_param("~child_frame", default="base_footprint")
        self.parrent_frame = rospy.get_param("~parrent_frame", default="map")
        self.pose_msg = PoseStamped()
    
    def check_frames_exist(self, frame):
        if self.tf_listener.frameExists(frame):
            return True
        else:
            rospy.logerr_once("frame {frame} doesn't exist".format(frame=frame))
            return False

    def wait_for_transform(self, time):
        if self.check_frames_exist(self.parrent_frame) and self.check_frames_exist(self.child_frame):
            self.tf_listener.waitForTransform(self.parrent_frame, self.child_frame, time, rospy.Duration(10.0))
        else:
            pass
    
    def get_pose(self):
        self.wait_for_transform(time=rospy.Time())
        while not rospy.is_shutdown():
            try:
                now = rospy.Time.now()
                self.wait_for_transform(time=now)
                translation, rotation = self.tf_listener.lookupTransform(self.parrent_frame, self.child_frame, now)

                self.pose_msg.header.frame_id = self.parrent_frame
                self.pose_msg.header.stamp = rospy.Time.now()
                self.pose_msg.pose.position.x = translation[0]
                self.pose_msg.pose.position.y = translation[1]
                self.pose_msg.pose.position.z = 0.0
                self.pose_msg.pose.orientation.x = 0.0
                self.pose_msg.pose.orientation.y = 0.0
                self.pose_msg.pose.orientation.z = rotation[2]
                self.pose_msg.pose.orientation.w = rotation[3]

                self.pose_pub.publish(self.pose_msg)
                self.rate.sleep()

            except LookupException as e:
                rospy.logerr_once(e)

            except ExtrapolationException as e:
                rospy.logerr_once(e)

if __name__ == "__main__":
    rospy.init_node("tf_to_pose")
    pose = Tf_to_Pose()
    pose.get_pose()
    rospy.spin()

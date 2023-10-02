#!/usr/bin/env python
import rospy
from geometry_msgs.msg import TransformStamped, PoseStamped
import tf2_ros
import tf2_geometry_msgs
import sys

class TfToPose:
    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.freq = rospy.get_param("~rate", default=1)
        self.rate = rospy.Rate(self.freq)
        self.pose_pub = rospy.Publisher("robot_pose", PoseStamped, queue_size=1)
        self.child_frame = rospy.get_param("~child_frame", default="base_footprint")
        self.parent_frame = rospy.get_param("~parent_frame", default="map")
        self.transform_tolerance = rospy.get_param('~transform_tolerance', default=0.5)

    def get_transform(self, target_frame, source_frame) -> TransformStamped:
        try:
            can_transform = self.tf_buffer.can_transform(target_frame, source_frame, rospy.Time(), rospy.Duration(1.0))
            if can_transform:
                transform: TransformStamped = self.tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time())
                if not (rospy.Time.now() - rospy.Duration(self.transform_tolerance)) > transform.header.stamp:
                    return transform
                else:
                    return None
            else:
                return None
        except tf2_ros.TransformException as ex:
            rospy.logerr(f"Transform exception: {ex}")
            return None

    def get_pose(self) -> PoseStamped:
        transform = self.get_transform(self.child_frame, self.parent_frame)
        if transform is not None:
            pose_stamped = tf2_geometry_msgs.do_transform_pose(PoseStamped(), transform)
            return pose_stamped
        else:
            return None

    def publish_pose(self):
        msg = self.get_pose()
        if msg is not None:
            self.pose_pub.publish(msg)

if __name__ == "__main__":
    rospy.init_node("tf_to_pose")
    robot_pose = TfToPose()
    try:
        while not rospy.is_shutdown():
            robot_pose.publish_pose()
            robot_pose.rate.sleep()
    except rospy.ROSInterruptException:
        pass

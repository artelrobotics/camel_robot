#!/usr/bin/env python
import rospy
from camel_robot.srv import pose_save, pose_saveResponse
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Quaternion
import yaml
import rospkg
import time

class Pose_Saver:
    def __init__(self):
        rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped,callback=self.amcl_pose, queue_size=1)
        self._as = rospy.Service('Pose_saver_service', pose_save, handler=self.pose_saver)
        self.rate = rospy.Rate(10)
        self.result = False
        rospack = rospkg.RosPack()
        self.base_path = rospack.get_path("camel_robot")
        self.frame_id = ""
        self.position = Point()
        self.orientation = Quaternion()
        self.goal = {}


    def amcl_pose(self, msg):
        self.frame_id = msg.header.frame_id
        self.position = msg.pose.pose.position
        self.orientation = msg.pose.pose.orientation


    def pose_saver(self, srv):
        resp = pose_saveResponse()
        time.sleep(0.5)
        self.goal[srv.name] = {
            "frame_id": self.frame_id,
            "position": {"x": self.position.x, "y": self.position.y, "z": self.position.z},
            "orientation": {"x": self.orientation.x, "y": self.orientation.y, "z": self.orientation.z, "w": self.orientation.w}}

        file_path = self.base_path + "/goals.yaml"
        try:
            with open(file_path, 'r') as file:
                names = yaml.safe_load(file)
                if names is not None:
                    names = list(names.keys())
                file.close()

            if names is None:
                with open(file_path, 'a') as file:
                    yaml.dump(self.goal, file, default_flow_style=False)
                    self.result = True
                    file.close()

            else:
                if not srv.name in names:
                    with open(file_path, 'a') as file:
                        yaml.dump(self.goal, file, default_flow_style=False)
                        self.result = True
                        file.close()
                else:
                    self.result = False

        except Exception as e:
            rospy.logwarn(e)
            self.result = False

        self.goal = {}
        resp.frame_id = self.frame_id
        resp.goal_pose.position = self.position
        resp.goal_pose.orientation = self.orientation
        resp.result = self.result
        return resp



if __name__ == '__main__':
    rospy.init_node('Pose_Saver_Service')

    server = Pose_Saver()
    rospy.spin()
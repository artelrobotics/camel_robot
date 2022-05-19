#!/usr/bin/env python
import rospy
import yaml
import rospkg

if __name__ == "__main__":
    rospy.init_node("amcl_pose")
    pose = {}
    rospack = rospkg.RosPack()
    base_path = rospack.get_path("camel_robot")
    file_path = base_path + "/config/navigation/amcl_initial_pose.yaml"
    r = rospy.Rate(5)
    while not rospy.is_shutdown():
        try:
            if rospy.has_param("amcl"):
                rospy.loginfo_once("start saving amcl initial pose")
                if rospy.has_param("amcl/initial_pose_a"):
                    pose['initial_pose_a'] = rospy.get_param("amcl/initial_pose_a")
                if rospy.has_param("amcl/initial_pose_x"):
                    pose['initial_pose_x'] = rospy.get_param("amcl/initial_pose_x")
                if rospy.has_param("amcl/initial_pose_y"):
                    pose['initial_pose_y'] = rospy.get_param("amcl/initial_pose_y")
                
                with open(file_path, 'w') as file:
                    yaml.dump(pose, file, default_flow_style=False)
        except Exception as e:
            rospy.logwarn(e)
        
        
        r.sleep()

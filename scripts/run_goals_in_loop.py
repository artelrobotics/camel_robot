#!/usr/bin/env python
import rospy
import yaml
import rospkg
import time
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from docking_server.msg import DockingFeedback, DockingAction, DockingResult, DockingGoal
def get_goals():
    rospack = rospkg.RosPack()
    base_path = rospack.get_path("camel_robot")
    goal_path = base_path + "/goals.yaml"
    with open(goal_path, 'r') as file:
        goals = yaml.safe_load(file)
        return goals

def feedback_cb(msg):
    rospy.loginfo(msg)

def dock_cb(msg):
    rospy.loginfo(msg)

if __name__ == "__main__":
    rospy.init_node('movebase_client_py')
    goals = get_goals()
    goal_names = list(goals.keys())
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client_dock = actionlib.SimpleActionClient('DockingServer', DockingAction)
    
    client.wait_for_server()
    client_dock.wait_for_server()
    
    while True:
        try:
            ac_goal = MoveBaseGoal()
            ac_goal.target_pose.header.frame_id = goals[goal_names[0]]['frame_id']
            ac_goal.target_pose.pose.position.x = goals[goal_names[0]]['position']['x']
            ac_goal.target_pose.pose.position.y = goals[goal_names[0]]['position']['y']
            ac_goal.target_pose.pose.position.z = goals[goal_names[0]]['position']['z']
            ac_goal.target_pose.pose.orientation.x = goals[goal_names[0]]['orientation']['x']
            ac_goal.target_pose.pose.orientation.y = goals[goal_names[0]]['orientation']['y']
            ac_goal.target_pose.pose.orientation.z = goals[goal_names[0]]['orientation']['z']
            ac_goal.target_pose.pose.orientation.w = goals[goal_names[0]]['orientation']['w']
            client.send_goal(ac_goal, feedback_cb=feedback_cb)
            if client.wait_for_result():
                result = client.get_result()
                if result:
                    rospy.loginfo(f'Goal {goal_names[0]} execution done')
                    if(goal_names[0] == "C"):
                        dock = DockingGoal()
                        dock.aruco_id = 1
                        dock.type = "docking"
                        client_dock.send_goal_and_wait(dock,execute_timeout=rospy.Duration(120))
                        break
                    goal_names.append(goal_names.pop(0))
                    time.sleep(5)
                else:
                    pass
            else:
                rospy.logerr("Action server not available!")
                client.cancel_goal()
                break
        except KeyboardInterrupt:
            client.cancel_goal()
            break
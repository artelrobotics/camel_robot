#!/usr/bin/env python3
import rospy
from arobotics_io_module.msg import ReadInputs
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped

class ButtonFinish:
    def __init__(self):
        self.button_status_pub = rospy.Publisher('/camel_amr_500_001/button/status', Bool, queue_size=1)
        self.new_goal_sub = rospy.Subscriber('/camel_amr_500_001/move_base/current_goal', PoseStamped, self.new_goal_callback)
        self.inputs_sub = rospy.Subscriber('/camel_amr_500_001/io_module/inputs_state', ReadInputs, self.inputs_callback)
        self.button_pin = rospy.get_param('button_pin', default=1)
        self.rate = rospy.Rate(10)
        self.pub_msg = Bool()
        self.pushed = False

    def new_goal_callback(self, msg):
        self.pushed = False

    def inputs_callback(self, msg: ReadInputs):
        button_state = msg.pins[self.button_pin - 1]

        if button_state:
            self.pushed = True

        self.pub_msg.data = button_state if not self.pushed else True
        self.button_status_pub.publish(self.pub_msg)

if __name__ == "__main__":
    rospy.init_node('button_finish_node')
    button_finish = ButtonFinish()

    rospy.spin()

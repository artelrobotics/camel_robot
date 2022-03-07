#! /usr/bin/env python3

import time
from ethernet_remote_io_module.msg import ReadDigitalInputs, WriteCoil, WriteCoils
from std_srvs.srv import SetBool, SetBoolResponse, SetBoolRequest
from std_msgs.msg import Bool
import rospy


def tester():
    global last
    rospy.wait_for_service("hooks_ctrl")
    while not rospy.is_shutdown():
        try:
            front_hook(True)
            time.sleep(0.1)
            front_hook(False)
            time.sleep(0.1)
        except rospy.ServiceException as e:
            rospy.logerr(e)

    

if __name__ == "__main__":
    last = None
    rospy.init_node("test_hook", anonymous=True, disable_signals=True)
    front_hook = rospy.ServiceProxy("hooks_ctrl", SetBool)
    tester()
    rospy.spin()

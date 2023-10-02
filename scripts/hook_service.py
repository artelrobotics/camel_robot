#!/usr/bin/env python3
import rospy
from std_srvs.srv import SetBool, SetBoolResponse
from arobotics_io_module.msg import SetOutput

class HookService:
    def __init__(self):
        self._as = rospy.Service('hooks_ctrl', SetBool , handler=self.hook_command)
        self.pub = rospy.Publisher('io_module/set_output', SetOutput, queue_size=1)
        self.pin = rospy.get_param('hook_pin', default=1)
        self.rate = rospy.Rate(10)
        self.pub_msg = SetOutput()

    def hook_up(self):
        """
            Hook up
        """
        self.pub_msg.pin = self.pin
        self.pub_msg.level = True
        self.pub.publish(self.pub_msg)

    def hook_down(self):
        """
            Hook down
        """
        self.pub_msg.pin = self.pin
        self.pub_msg.level = False
        self.pub.publish(self.pub_msg)

    def hook_command(self, command : SetBool) -> SetBoolResponse:
        """
        Service handler recives hook command in SetBool format

        Anaylze command and send a particular massage to /write_coil for Hook Up or Down

        Response will be in SetBoolResponse
        """

        #Initialize command from service
        request_command = command.data

        # If request command is True Hooks will be UP!
        if (request_command == True):
            self.hook_up()
            return SetBoolResponse(True, "hooks are up")

        # If request command is False Hooks will be Down!
        else:
            self.hook_down()
            return SetBoolResponse(True, "hooks are down")


if __name__ == "__main__":
    rospy.init_node('Hook_Service', anonymous=True)
    service = HookService()
    rospy.spin()

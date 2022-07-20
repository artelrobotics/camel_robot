#!/usr/bin/env python3
from typing import List
import rospy
import time
from ethernet_remote_io_module.msg import ReadDigitalInputs, WriteCoil, WriteCoilsList
#from motion_control import CamelControl
from camel_robot.srv import light, sound
from std_srvs.srv import SetBool, SetBoolResponse
from Light_control import LightControl
from robot_sound_service import SoundService

class Dance_service:

    def __init__(self):
        """ Initialize node and Subscribes topics """

        rospy.Service('/camel_amr_1000_001/Dance_service', SetBool , handler=self.dance_command)

        self.light_service = rospy.ServiceProxy("/camel_amr_1000_001/common/Light_server", light)
        self.sound_service = rospy.ServiceProxy("/camel_amr_1000_001/sound_server", sound)
        self.rate = rospy.Rate(40)
        self.light = LightControl()
        #self.motion = CamelControl()
       
        

    def dance_command(self, command : SetBool) -> SetBoolResponse:
        
        """
            [Callback function of Modbus read inputs topic]
            Arg:
                msg: Bool 
            if button pressed change self.button_pressed variable to True
        """    

        #Initialize command from service
        request_command = command.data
        
        # If request command is True Hooks will be UP!
        if (request_command == True):
            #self.sound_service("andijon")
            self.light.light_type_1()
            return SetBoolResponse(True, "Dance has been succesfully finished!")
        
        # If request command is False Hooks will be Down!
        else:
            self.sound_service("sound_stop")
            
            return SetBoolResponse(True, "Dance turned off")
   
    
    

if __name__ == '__main__':
    rospy.init_node('Dance_service')

    server = Dance_service()
    rospy.spin() 
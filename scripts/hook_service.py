#!/usr/bin/env python3
import rospy
from ethernet_remote_io_module.msg import WriteCoil
from std_srvs.srv import SetBool, SetBoolResponse
from Light_control import LightControl


class HookService:
    def __init__(self):        
        self._as = rospy.Service('/camel_amr_1000_001/common/hooks_ctrl', SetBool , handler=self.hook_command)
        self.pub = rospy.Publisher('/camel_amr_1000_001/common/write_coil', WriteCoil, queue_size=1)
        self.rate = rospy.Rate(10)
        self.pub_msg = WriteCoil()
        self.lc = LightControl()

    
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
            self.lc.hook_up()
            return SetBoolResponse(True, "hooks are up")
        
        # If request command is False Hooks will be Down!
        else:
            self.lc.hook_down()
            return SetBoolResponse(True, "hooks are down")

        

if __name__ == "__main__":
    rospy.init_node('Hook_Service', anonymous=True)   
    service = HookService() 
    rospy.spin()

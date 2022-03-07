#! /usr/bin/env python3

from ethernet_remote_io_module.msg import ReadDigitalInputs, WriteCoil, WriteCoils
from std_srvs.srv import SetBool, SetBoolResponse, SetBoolRequest
import threading
import rospy
import time

class Hook:

    def __init__(self, out_pin: int, input_up: int , input_down: int, name: str) -> None:
        self.out_pin    = out_pin - 1
        self.input_up   = input_up - 1 
        self.input_down = input_down - 1
        self.name = name
        self.up_state   = False
        self.down_state = False
        self.success    = False
        self.coil_value = False

        
        self.rate = rospy.Rate(10)
        rospy.Service(self.name, SetBool, self.hooks_controller)
        rospy.Subscriber('read_inputs', ReadDigitalInputs, self.read)
        self.pub_coil = rospy.Publisher('write_coil', WriteCoil, queue_size=1)


    def write(self) -> None :
        write_coil_msg = WriteCoil()
        write_coil_msg.address = self.out_pin
        write_coil_msg.value = self.coil_value
        self.pub_coil.publish(write_coil_msg)


    def read(self, data: ReadDigitalInputs) -> None:
        self.up_state   = data.all_inputs[self.input_up]
        self.down_state = data.all_inputs[self.input_down]


    def hooks_controller(self, request: SetBoolRequest) -> SetBoolResponse:
        self.success = False
        if request.data: # up the hook
            while not self.up_state:
                self.coil_value = True
                self.write()
            else:
                self.coil_value = False
                self.write()
                
            self.success = True
            return SetBoolResponse(self.success, "HOOK IS UP")

        else: # down the hook
            while not self.down_state:
                self.coil_value = True
                self.write()
            else:
                self.coil_value = False
                self.write()

            self.success = True
            return SetBoolResponse(self.success, "HOOK IS DOWN")


    
if __name__ == "__main__":
    rospy.init_node('hooks_control', anonymous=True, disable_signals=True)
    # front_pins = rospy.get_param("~front_pins", default={"out_pin": 2, "input_up": 2, "input_down": 1})
    # back_pins = rospy.get_param("~back_pins", default={"out_pin": 2, "input_up": 2, "input_down": 1})
    front_hook_ctrl = Hook(out_pin=1, input_up=2 , input_down=1, name="front_hook")
    back_hook_ctrl  = Hook(out_pin=2, input_up=3 , input_down=4, name="back_hook")
    for _ in range(10):
        rospy.logdebug("Initialize publisher")
        rospy.logdebug("Publishing blank msgs")
        front_hook_ctrl.write()
        back_hook_ctrl.write()
        time.sleep(0.1)

    rospy.spin()

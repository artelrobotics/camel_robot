#! /usr/bin/env python

from ethernet_remote_io_module.msg import ReadDigitalInputs, WriteCoil, WriteCoils
from std_srvs.srv import SetBool, SetBoolResponse, SetBoolRequest
from dataclasses import dataclass
import rospy

@dataclass
class Hook:
    out_pin: int
    input_up: int
    input_down:int
    up_state: bool = False
    down_state: bool = False

    def __init__(self, out_pin: int, input_up: int , input_down: int) -> None:
        self.out_pin    = out_pin - 1 
        self.input_up   = input_up - 1 
        self.input_down = input_down - 1


class HooksCtrl:
    def __init__(self) -> None:
        self.front_pins = rospy.get_param("~front_pins", default={"out_pin": 2, "input_up": 2, "input_down": 1})
        self.back_pins  = rospy.get_param("~back_pins", default={"out_pin": 1, "input_up": 3, "input_down": 4})
        self.front_hook = Hook(**self.front_pins)
        self.back_hook  = Hook(**self.back_pins)
        self.__hook_srv = rospy.Service('hooks_control', SetBool, self.hooks_controller)
        self.__inputs   = rospy.Subscriber('read_digital_inputs', ReadDigitalInputs, self.read_inputs)
        self.__pub_coil = rospy.Publisher('write_coil', WriteCoil, queue_size=None)
        self.__pub_coils = rospy.Publisher('write_coils', WriteCoils, queue_size=None)
    
    def hooks_controller(self, request: SetBoolRequest):

        write_coil_msg = WriteCoil()
        write_coils_msg = WriteCoils()
        success = False

        if request.data:
            while not success:

                if not self.front_hook.up_state and not self.back_hook.up_state:
                    write_coils_msg.address = self.back_hook.out_pin
                    write_coils_msg.last = self.front_hook.out_pin
                    write_coils_msg.value = True
                    self.__pub_coils.publish(write_coils_msg)

                elif self.front_hook.up_state and not self.back_hook.up_state:
                    write_coil_msg.address = self.back_hook.out_pin
                    write_coil_msg.value = True
                    self.__pub_coil.publish(write_coil_msg)
                    write_coil_msg.address = self.front_hook.out_pin
                    write_coil_msg.value = False
                    self.__pub_coil.publish(write_coil_msg)
                
                elif not self.front_hook.up_state and self.back_hook.up_state:
                    write_coil_msg.address = self.front_hook.out_pin
                    write_coil_msg.value = True
                    self.__pub_coil.publish(write_coil_msg)
                    write_coil_msg.address = self.back_hook.out_pin
                    write_coil_msg.value = False
                    self.__pub_coil.publish(write_coil_msg)

                elif self.front_hook.up_state and self.back_hook.up_state:
                    write_coils_msg.address = self.back_hook.out_pin
                    write_coils_msg.last = self.front_hook.out_pin
                    write_coils_msg.value = False
                    self.__pub_coils.publish(write_coils_msg)
                    success = True
            return SetBoolResponse(success, "HOOKS IS UP")
                    

        else:
            while not success:

                if not self.front_hook.down_state and not self.back_hook.down_state:
                    write_coils_msg.address = self.back_hook.out_pin
                    write_coils_msg.last = self.front_hook.out_pin
                    write_coils_msg.value = True
                    self.__pub_coils.publish(write_coils_msg)

                elif self.front_hook.down_state and not self.back_hook.down_state:
                    write_coil_msg.address = self.back_hook.out_pin
                    write_coil_msg.value = True
                    self.__pub_coil.publish(write_coil_msg)
                    write_coil_msg.address = self.front_hook.out_pin
                    write_coil_msg.value = False
                    self.__pub_coil.publish(write_coil_msg)
                
                elif not self.front_hook.down_state and self.back_hook.down_state:
                    write_coil_msg.address = self.front_hook.out_pin
                    write_coil_msg.value = True
                    self.__pub_coil.publish(write_coil_msg)
                    write_coil_msg.address = self.back_hook.out_pin
                    write_coil_msg.value = False
                    self.__pub_coil.publish(write_coil_msg)

                elif self.front_hook.down_state and self.back_hook.down_state:
                    write_coils_msg.address = self.back_hook.out_pin
                    write_coils_msg.last = self.front_hook.out_pin
                    write_coils_msg.value = False
                    self.__pub_coils.publish(write_coils_msg)
                    success = True
            return SetBoolResponse(success, "HOOKS IS DOWN")

    def read_inputs(self, data: ReadDigitalInputs) -> None:
        self.front_hook.up_state   = data.all_inputs[self.front_hook.input_up]
        self.front_hook.down_state = data.all_inputs[self.front_hook.input_down]
        self.back_hook.up_state    = data.all_inputs[self.back_hook.input_up]
        self.back_hook.down_state  = data.all_inputs[self.back_hook.input_down]

if __name__ == "__main__":
    rospy.init_node('hooks_control', anonymous=True)
    hook_ctrl = HooksCtrl()
    rospy.spin()
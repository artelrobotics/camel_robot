#! /usr/bin/env python3

from ethernet_remote_io_module.msg import ReadDigitalInputs, WriteCoil
from std_srvs.srv import SetBool, SetBoolResponse, SetBoolRequest
import rospy
import time

class Hook:
    def __init__(self, out_pin: int, input_up: int , input_down: int, publisher: rospy.Publisher) -> None:
        self.out_pin    = out_pin - 1
        self.input_up   = input_up - 1 
        self.input_down = input_down - 1
        self.up_state   = False
        self.down_state = False
        self.is_up      = False
        self.is_down    = False
        self.coil_value = False
        self.publisher  = publisher

    def write(self) -> None :
        rospy.logdebug("Hook write function run")
        write_coil_msg = WriteCoil()
        write_coil_msg.address = self.out_pin
        write_coil_msg.value = self.coil_value
        self.publisher.publish(write_coil_msg)

    def hook_ctrl(self, request: bool) -> bool:
        rospy.logdebug("Hook write function hook_ctrl run")
        if request: # up the hook
            rospy.logdebug("up the hook ")
            while not self.up_state:
                self.coil_value = True
                self.write()
            else:
                self.coil_value = False
                self.write()
            self.is_up = True
            return self.is_up

        else: # down the hook
            rospy.logdebug("down the hook ")
            while not self.down_state:
                self.coil_value = True
                self.write()
            else:
                self.coil_value = False
                self.write()
            self.is_down = True
            return self.is_down


def read_inputs(msg: ReadDigitalInputs):
    rospy.logdebug_once("read inputs")
    global front_hook_ctrl, back_hook_ctrl
    front_hook_ctrl.up_state   = msg.all_inputs[front_hook_ctrl.input_up]
    front_hook_ctrl.down_state = msg.all_inputs[front_hook_ctrl.down_state]
    back_hook_ctrl.up_state    = msg.all_inputs[back_hook_ctrl.input_up]
    back_hook_ctrl.down_state  = msg.all_inputs[back_hook_ctrl.input_down]

def hooks_controller(request: SetBoolRequest) -> SetBoolResponse:
    rospy.logdebug_once("run service")
    if request:
        rospy.logdebug("Up hooks")
        while not front_hook_ctrl.is_up and back_hook_ctrl.is_up:
            front_hook_ctrl.hook_ctrl(request=request)
            back_hook_ctrl.hook_ctrl(request=request)
        if front_hook_ctrl.is_up and back_hook_ctrl.is_up:
            rospy.logdebug("hooks are up")
            return SetBoolResponse(True, "hooks are up")
    
    else:
        rospy.logdebug("Down hooks")
        while not front_hook_ctrl.is_down and back_hook_ctrl.is_down:
            front_hook_ctrl.hook_ctrl(request=request)
            back_hook_ctrl.hook_ctrl(request=request)
        if front_hook_ctrl.is_down and back_hook_ctrl.is_down:
            rospy.logdebug("hooks are down")
            return SetBoolResponse(True, "hooks are down")


if __name__ == "__main__":
    rospy.init_node('hooks_control', anonymous=True, disable_signals=True)
    while not rospy.is_shutdown():
        try:
            rospy.Subscriber('read_inputs', ReadDigitalInputs, read_inputs)
            rospy.Service("hooks_ctrl", SetBool, hooks_controller)
            publisher = rospy.Publisher('write_coil', WriteCoil, queue_size=1)
            front_pins = rospy.get_param("~front_pins", default={"out_pin": 1, "input_up": 2, "input_down": 1})
            back_pins = rospy.get_param("~back_pins", default={"out_pin": 2, "input_up": 3, "input_down": 4})
            rospy.logdebug(front_pins, back_pins)
            front_hook_ctrl = Hook(out_pin=1, input_up=2 , input_down=1, publisher=publisher)
            back_hook_ctrl  = Hook(out_pin=2, input_up=3 , input_down=4, publisher=publisher)
            
            for _ in range(10):
                rospy.logdebug_once("Initialize publisher")
                rospy.logdebug_once("Publishing blank msgs")
                front_hook_ctrl.write()
                back_hook_ctrl.write()
                time.sleep(0.1)
            rospy.spin()

        except KeyboardInterrupt:
            break

        except Exception as e:
            rospy.logerr(e)
            break

#!/usr/bin/env python3
from typing import List
import rospy
import time
from ethernet_remote_io_module.msg import ReadDigitalInputs, WriteCoil, WriteCoilsList

class LightControl():
    def __init__(self) -> None:
        self.rate = rospy.Rate(20)  # Rate in Hz
        self.read_digital_inputs_subscriber = rospy.Subscriber('read_inputs', ReadDigitalInputs, self.read_digital_callback)
        self.write_coils_publisher = rospy.Publisher('write_coils', WriteCoilsList, queue_size=10)
        self.write_coil_publisher = rospy.Publisher('/camel_amr_1000_001/common/write_coil', WriteCoil, queue_size=1)
        self.write_coils_msg = WriteCoilsList()
        self.pub_msg = WriteCoil()
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)
        
        """
        lupa x2 - out 3
        circle left - out 4
        line left - out 5
        solinoid x2 - out 6
        circle right - out 7
        line right - out 8
        """  

    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        self.ctrl_c = True    
    
    def write_once_in_coil(self):
        """
        This is because publishing in topics sometimes fails the first time you publish.
        In continuos publishing systems there is no big deal but in systems that publish only
        once it IS very important.
        """
        while not self.ctrl_c:
            summit_connections = self.write_coils_publisher.get_num_connections()
            if summit_connections > 0:
                self.write_coils_publisher.publish(self.write_coils_msg)     
                break
            else:
                self.rate.sleep()
    
    def read_digital_callback(self, data):
        self.din_all = data.all_inputs 
        return self.din_all
    
    def read_inputs(self):
        return self.din_all
    
    def read_specific_din(self, pin):
        time.sleep(0.1)
        return self.din_states[pin]
        
    def write_coils(self, value):
        self.write_coils_msg.value = value
        self.write_once_in_coil()

    def hook_up(self):
        """
            Hook up
        """
        self.pub_msg.address = 0
        self.pub_msg.value = True
        self.write_coil_publisher.publish(self.pub_msg)
    
    def hook_down(self):
        """
            Hook down
        """
        self.pub_msg.address = 0
        self.pub_msg.value = False
        self.write_coil_publisher.publish(self.pub_msg)
    
    def light_off(self):
        light = [0,0,0,0,0,0]
        self.write_coils(light)
    
    def light_type_1(self):
        """
            Both Lupas are on.
        """
        light = [1,0,0,0,0,0]       
        self.write_coils(light)
    
    def light_type_2(self):
        """
            Both Circles are on.
        """
        light = [0,1,0,0,1,0]
        self.write_coils(light)
    
    def light_type_3(self):
        """
            Both lines are on.
        """
        light = [0,0,1,0,0,1]
        self.write_coils(light)
    
    def light_type_4(self):
        """
            Both lupas with Solinoids(Дальний) are on.
        """
        light = [1,0,0,1,0,0]
        self.write_coils(light)
        
    def light_type_5(self):
        """
            Blinking lines are on.
        """
        i = 0
        while i<2:
            light = [0,0,1,0,0,1]
            self.write_coils(light)
            time.sleep(0.2)
            light = [0,0,0,0,0,0]
            self.write_coils(light)
            time.sleep(0.2)
            i=i+1
    
    


    def light_type_7(self):
        """
            Blinking lines are on.
        """
        light = [0,0,1,0,0,1]
        self.write_coils(light)
        time.sleep(0.2)
        light = [0,0,0,0,0,0]
        self.write_coils(light)
        time.sleep(0.2)
        

    def light_dance(self):
        """
            Dancing script are on!
        """
        light = [1,0,0,0,0,0,0,0]
        self.write_coils(light)
        time.sleep(0.1)
        light = [0,1,0,0,1,0,0,0]
        self.write_coils(light)
        time.sleep(0.2)
        light = [0,1,0,0,0,0,0,0]
        self.write_coils(light)
        time.sleep(0.2)
        light = [0,0,0,0,1,0,0,0]
        self.write_coils(light)
        time.sleep(0.2)
        light = [0,0,0,1,0,0,1,0]
        self.write_coils(light)
        time.sleep(0.2)
        light = [0,0,0,1,0,0,0,0]
        self.write_coils(light)
        time.sleep(0.2)
        light = [1,0,0,0,0,0,1,0]
        self.write_coils(light)
        time.sleep(0.1)
        light = [1,0,0,1,0,0,1,0]
        self.write_coils(light)
        time.sleep(0.2)
        light = [0,0,0,0,0,0,0,0]
        self.write_coils(light)
        time.sleep(2)
        
if __name__ == '__main__':
    rospy.init_node('Light_Control')

    server = LightControl()

    rospy.spin()
  
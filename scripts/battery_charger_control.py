#! /usr/bin/env python3
import rospy
from robotnik_msgs.msg import BatteryStatus
from std_msgs.msg import Float32
from ethernet_remote_io_module.msg import WriteCoil
from camel_robot.srv import light, sound
import time


class Battery_and_Charger_Control:

    def __init__(self):
        """ Initialize node and Subscribes topics """

        rospy.Subscriber('/camel_amr_1000_001/bms/data', BatteryStatus, self.bms_data_callback)
        self.pub = rospy.Publisher('/camel_amr_1000_001/common/write_coil', WriteCoil, queue_size=1)
        self.light_service = rospy.ServiceProxy("/camel_amr_1000_001/common/Light_server", light)
        self.sound_service = rospy.ServiceProxy("/camel_amr_1000_001/sound_server", sound)
        self.rate = rospy.Rate(10)
        self.pub_msg = WriteCoil()    
        self.max_voltage_threshold = 3.47
        self.min_voltage_threshold = 3.42
        self.min_warning_voltage = 2.9
        
    def bms_data_callback(self, msg: BatteryStatus):
        
        """[Callback function of bms/data topic]
            Arg:
                msg: BatteryStatus
            If max_cell reaches max_voltage_threshold Charging will be OFF 
            
            else max_cell downwards untill min_voltage_threshold Charging will be ON!
        """
        
        current = msg.current
        voltage = msg.voltage
        max_cell = msg.max_cell
        min_cell = msg.min_cell
        avr_cell = msg.avg_cell
        level = msg.level
        charging = msg.is_charging

        if (min_cell < self.min_warning_voltage and not charging and min_cell != 0):
            self.sound_service("low_battery")
            self.light_service("light_type_7")
            time.sleep(30)
        elif (max_cell > self.max_voltage_threshold) and (self.pub_msg.value == False):
            self.pub_msg.address = 1
            self.pub_msg.value = True
            self.pub.publish(self.pub_msg)

        elif(max_cell < self.min_voltage_threshold) and (self.pub_msg.value == True):
            self.pub_msg.address = 1
            self.pub_msg.value = False
            self.pub.publish(self.pub_msg)
        
        else:
            pass



if __name__ == '__main__':
    rospy.init_node('Battery_and_Charger_Control')
    time.sleep(30)
    server = Battery_and_Charger_Control()
    rospy.spin() 
#! /usr/bin/env python3
import rospy
from robotnik_msgs.msg import BatteryStatus
from std_msgs.msg import Float32
import time


class Battery_calculation:

    def __init__(self):
        """ Initialize node and Subscribes topics """
        
        rospy.Subscriber('bms/data', BatteryStatus, self.bms_data_callback)
        self.battery_status_pub = rospy.Publisher('/camel_amr_1000_001/battery_status', Float32, queue_size=1)

        self.rate = rospy.Rate(40)    
        self.time_1 = time.time()
        self.time_2 = time.time()
        self.total_current_h = 0
    
    def bms_data_callback(self, msg: BatteryStatus):
        
        """[Callback function of Move_base/result topic]
            Arg:
                msg: MoveBaseActionResult
            if goal reached  signal turns on untill button is pressed
        """
        self.time_1 = time.time()

        current = msg.current
        voltage = msg.voltage

        delta_time = self.time_1 - self.time_2
        amper_h = current * delta_time / 3600
        self.total_current_h = amper_h + self.total_current_h
        percentage = self.total_current_h / 200 * 100
        print("Instantenous current", amper_h)
        print("Total current is : ", self.total_current_h)
        print("Percentage is : ", percentage,"%")

        self.time_2 = time.time() 
    


if __name__ == '__main__':
    rospy.init_node('Battery_calculation')

    server = Battery_calculation()
    rospy.spin() 
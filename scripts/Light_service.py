#!/usr/bin/env python3
from Light_control import LightControl
import rospy
from rospy.core import rospyerr ,loginfo
from camel_robot.srv import light
class LightService:

    def __init__(self):
        self._as = rospy.Service('Light_server', light, handler=self.light_command)
        self.lc = LightControl()
        self.rate = rospy.Rate(10)    
        self.result = light()

    def light_command(self, command):
        self.command = command.request_string
        
        if(self.command == "light_off"):
            self.lc.light_off()
            loginfo("All Lights are turned OFF!")
            self.result = "ALL Lights are turned OFF!"
        
        elif(self.command == "light_type_1"):
            self.lc.light_type_1()
            loginfo("Lighting_type_1 is on!")
            self.result = "Lighting_type_1 is on!"
        
        elif(self.command == "light_type_2"):
            self.lc.light_type_2()
            loginfo("Lighting_type_2 is on!")
            self.result = "Lighting_type_2 is on!"
        
        elif(self.command == "light_type_3"):
            self.lc.light_type_3()
            loginfo("Lighting_type_3 is on!")
            self.result = "Lighting_type_3 is on!"
        
        elif(self.command == "light_type_4"):
            self.lc.light_type_4()
            loginfo("Lighting_type_4 is on!")
            self.result = "Lighting_type_4 is on!"
        
        elif(self.command == "light_type_5"):
            self.lc.light_type_5()
            loginfo("Lighting_type_5 is on!")
            self.result = "Lighting_type_5 is on!"
        
        elif(self.command == "light_dance"):
            self.lc.light_dance()
            loginfo("Light Dance has been done!")
            self.result = "Light Dance has been done!"
        
        else:
            loginfo("Undefined request String")
            self.result = "Undefined request String"
        return self.result
        
if __name__ == '__main__':
    rospy.init_node('Light_Service')

    server = LightService()

    rospy.spin()
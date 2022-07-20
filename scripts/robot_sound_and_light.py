#! /usr/bin/env python3
import rospy
from ethernet_remote_io_module.msg import ReadDigitalInputs
from camel_robot.srv import light, sound
from move_base_msgs.msg import MoveBaseActionResult, MoveBaseActionGoal
from actionlib_msgs.msg import GoalStatusArray
from std_msgs.msg import Bool
import time

class Task_compleation:

    def __init__(self):
        """ Initialize node and Subscribes topics """
        self.prev_data = 0
        #rospy.Subscriber('move_base/result', MoveBaseActionResult, self.task_finish_callback)
        rospy.Subscriber('move_base/status', GoalStatusArray, self.status_recieve_callback)
        rospy.Subscriber('common/read_inputs', ReadDigitalInputs, self.button_callback)
        self.pub_task_comp = rospy.Publisher("task_compelation", Bool , queue_size=1)
        self.light_service = rospy.ServiceProxy("common/Light_server", light)
        self.sound_service = rospy.ServiceProxy("sound_server", sound)
        self.rate = rospy.Rate(40)    
        self.button_status = False
        self.button_pressed = False            
        time.sleep(2)
        self.light_service("light_type_2")
        

    def button_callback(self, msg: ReadDigitalInputs):
        
        """
            [Callback function of Modbus read inputs topic]
            Arg:
                msg: Bool 
            if button pressed change self.button_pressed variable to True
        """    
        self.button_status = msg.all_inputs[4]
        if self.button_status == True:
            self.button_pressed = True
    
    # def task_finish_callback(self, msg: MoveBaseActionResult):
        
    #     """[Callback function of Move_base/result topic]
    #         Arg:
    #             msg: MoveBaseActionResult
    #         if goal reached  signal turns on untill button is pressed
    #     """
    #     self.sound_service("sound_stop")
    #     self.button_pressed = False
    #     self.sound_service("beep")
    #     while self.button_pressed == False:
    #             self.light_service("light_type_7")
    #             time.sleep(0.2)
            
    #     self.sound_service("sound_stop")
    #     self.sound_service("button")
    #     self.light_service("light_type_2")
    
    def status_recieve_callback(self, msg: GoalStatusArray):
        
        """[Callback function of Move_base/goal topic]
            Arg:
                msg: MoveBaseActionGoal
            if goal recieved , It starts turn on motion sound and light
        """
        if msg.status_list != []:
            data = msg.status_list[0].status
        
            if data == 1 and data != self.prev_data:
                self.sound_service("sound_stop")
                self.sound_service("motion")
                self.light_service("light_type_2")
                status = False
            elif data == 3 and data != self.prev_data:
                self.sound_service("sound_stop")
                self.button_pressed = False
                self.sound_service("beep")
                while self.button_pressed == False:
                        self.light_service("light_type_7")
                        time.sleep(0.2)
                status = True
                
                    
                self.sound_service("sound_stop")
                self.sound_service("button")
                self.light_service("light_type_2")
                time.sleep(1)
            
            else:
                status = False

            self.pub_task_comp.publish(status)
            self.prev_data = data

    def warning(self):
        
        """[Callback function of Move_base/goal topic]
            Arg:
                msg: MoveBaseActionGoal
            if goal recieved , It starts turn on motion sound and light
        """
        self.sound_service("warning")
    
    def stop_sound(self):
        
        """[Callback function of Move_base/goal topic]
            Arg:
                msg: MoveBaseActionGoal
            if goal recieved , It starts turn on motion sound and light
        """
        self.sound_service("sound_stop")
    
    

if __name__ == '__main__':
    rospy.init_node('Robot_sound_and_light')

    server = Task_compleation()
    rospy.spin() 
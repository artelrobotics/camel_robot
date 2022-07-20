#!/usr/bin/env python3
import rospy
from rospy.core import rospyerr, rospywarn
import math
import time
from sound_play.msg import SoundRequest
from camel_robot.srv import sound
from sound_play.libsoundplay import SoundClient
import time
class SoundService:

    def __init__(self):        
        self._as = rospy.Service('sound_server', sound, handler=self.sound_command)
        self.pub = rospy.Publisher('robotsound', SoundRequest, queue_size=1)
        self.rate = rospy.Rate(10)
        self.sound_dir = '/home/nvidia/sounds/'
        self.result = sound()
        self.sound = SoundRequest()
        self.soundhandle = SoundClient()

        
    def on_start(self):
        time.sleep(5)
        self.sound.sound = -2
        self.sound.command = 1
        self.sound.volume = 0.3
        self.sound.arg = self.sound_dir + 'startup.wav'
        self.sound.arg2 = ''
        self.pub.publish(self.sound)

    def sound_command(self, command : str) -> str:
        """ Service handler recives sound command in string format
            
        Anaylze command and send a particular massage to /robosound topic
        for sound playing
        """
        #Initialize command from service
        command = command.request_string
        
        #Stopping All playing sounds
        self.soundhandle.stopAll()
        
        
        #Playing a specific sound with command releted     
        
        if (command == "beep"):
            self.sound.sound = -2
            self.sound.command = 2
            self.sound.volume = 0.05 #0.5
            self.sound.arg = self.sound_dir + 'beep.wav'
            self.sound.arg2 = ''

            self.result = 'Sound type 3 is playing!'
            
        elif (command == "motion"):
            self.sound.sound = -2
            self.sound.command = 2
            self.sound.volume = 0.03
            self.sound.arg = self.sound_dir + 'Motion.mp3'
            self.sound.arg2 = ''

            self.result = 'Sound bep_signal is playing'

        elif (command == "button"):
            self.sound.sound = -2
            self.sound.command = 1
            self.sound.volume = 0.3
            self.sound.arg = self.sound_dir + 'correct.mp3'
            self.sound.arg2 = ''

            self.result = 'Sound bep_signal is playing'
        
        elif (command == "low_battery"):
            self.sound.sound = -2
            self.sound.command = 1
            self.sound.volume = 0.3
            self.sound.arg = self.sound_dir + 'low_battery.mp3'
            self.sound.arg2 = ''

            self.result = 'Sound low_battery is playing'
        
        elif (command == "warning"):
            self.sound.sound = -2
            self.sound.command = 2
            self.sound.volume = 0.3
            self.sound.arg = self.sound_dir + 'Warning.mp3'
            self.sound.arg2 = ''

            self.result = 'Sound warning_signal is playing'
        
        elif (command == "50"):
            self.sound.sound = -2
            self.sound.command = 1
            self.sound.volume = 0.3
            self.sound.arg = self.sound_dir + '11.mp3'
            self.sound.arg2 = ''

            self.result = '50 is playing'
        
        elif (command == "andijon"):
            self.sound.sound = -2
            self.sound.command = 1
            self.sound.volume = 1
            self.sound.arg = self.sound_dir + 'andijon.mp3'
            self.sound.arg2 = ''

            self.result = '50 is playing'

        elif (command == "goal"):
            self.sound.sound = -2
            self.sound.command = 1
            self.sound.volume = 0.3
            self.sound.arg = self.sound_dir + 'goal_recieve.wav'
            self.sound.arg2 = ''

            self.result = 'Sound bep_signal is playing'

        
        elif (command == "say_hi"):
            self.sound.sound = -3
            self.sound.command = 1
            self.sound.volume = 1.0
            self.sound.arg = 'Hi. My name is Camel AMR 001. I was made by Artel Group'
            self.sound.arg2 = ''

            self.result = 'Say hi is playing!'
        
        elif (command == "sound_stop"):
            self.sound.sound = -1
            self.sound.command = 0
            self.sound.volume = 1.0
            self.sound.arg = ''
            self.sound.arg2 = ''
            
            self.result = 'All sounds are stopped!'
        
        # Else case for invalid sound names
        else:
            rospy.logwarn('Invalid Sound name. Please check sound name and try again!')
            self.result = 'Invalid Sound name. Please check sound name and try again!'

        # publishing sound command massage to Sound Node(/robosound)
        self.pub.publish(self.sound)
        
        # return result of the Service
        return self.result
        
if __name__ == '__main__':
    rospy.init_node('Sound_Service')
    server = SoundService()
    server.rate.sleep()
    server.on_start()
    rospy.spin()
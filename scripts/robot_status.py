#!/usr/bin/env python3
from numpy import append
import rospy
from std_msgs.msg import String
from robotnik_msgs.msg import BatteryStatus
from camel_robot.msg import RobotStatus, DriverStatus, BmsStatus
from actionlib_msgs.msg import GoalStatusArray
from roboteq_motor_controller_driver.msg import channel_values

def check_states(status_names, state):
    """[Finding status name from binary array]
    Args:
        status_names(list):[Name of status]
        states(list):[Binary array]
    Returns:
        out_states(list):[list of name states]
    """
    states = [int(state) for state in format(state, "08b")]
    states.reverse()
    out_states = []
    for index, value in enumerate(states):
        if value:
            out_states.append(status_names[index])
    return out_states


def fault_flags_callback(value):
    """
        [Callback function of Fault flag topic]
        Arg:
            value(int64[]):fault_flag_number(2^(n-1))
        Publishes Name of fault to topic '/robot_status' 
    """
    status_naming = ["Roboteq_Driver/Overheat", "Roboteq_Driver/Overvoltage", "Roboteq_Driver/Undervoltage", "Roboteq_Driver/Short_Circuit", "Roboteq_Driver/Emergancy_Stop", "Roboteq_Driver/Motor/Sensor setup fault", "Roboteq_Driver/MosFet failure", "Roboteq_Driver/Default configuration loaded at startup"]
    if(value.value[0] == 0):
        status=["Roboteq_driver_fault_flag/Normal"]
    else:
        status = check_states(status_naming, state=value.value[0])
    #rospy.loginfo(status)
    robot_status.driver_status.fault_flag = status
    status_pub.publish(robot_status)
def status_flags_callback(value):
    """
        [Callback function of Status flags topic]
        Arg:
            value(int64[]):fault_flag_number(2^(n-1))
        Publishes Name of status each wheel to topic '/robot_status' 
    """
    status_naming_right = ["Motor_right/Amps Limit currently active", "Motor_right/Motor stalled", "Motor_right/Loop Error detected", "Motor_right/Safety Stop active", "Motor_right/Forward Limit triggered", "Motor_right/Reverse Limit triggered", "Motor_right/Amps Trigger activated"]
    status_naming_left = ["Motor_left/Amps Limit currently active", "Motor_left/Motor stalled", "Motor_left/Loop Error detected", "Motor_left/Safety Stop active", "Motor_left/Forward Limit triggered", "Motor_left/Reverse Limit triggered", "Motor_left/Amps Trigger activated"]
    if(value.value[0] == 0):
        status_left = ["Left_Motor_status_flag/Normal"]
    else:
        status_left = check_states(status_naming_left, state=value.value[0])
    if(value.value[1] == 0):
        status_right = ["Right_Motor_status_flag/Normal"]
    else:
        status_right = check_states(status_naming_right, state=value.value[1])
    
    #rospy.loginfo(status_left)
    robot_status.driver_status.status_flag_left = status_left
    robot_status.driver_status.status_flag_right = status_right
    status_pub.publish(robot_status)
    #rospy.loginfo(status_right)
    
def bms_status_flags_callback(data):
    """
        [Callback function of Fault flag topic]
        Arg:
            data(str):fault_flag_number(2^(n-1))
        Publishes Name of fault to topic 'robot_status' 
    """
    status_naming = ["roboteq_bms/Trigger Load", "roboteq_bms/Trigger Charge", "roboteq_bms/Cell Over Volt", 
                    "roboteq_bms/Cell Under Volt", "roboteq_bms/Unsafe Temperature", "roboteq_bms/Bad State of Health",
                     "roboteq_bms/Balancing", "roboteq_bms/RunScript"]
    #print(data.data)
    try :
        if data.data == '':
            status = ["BMS_Status_flag/Empty"]
            pass
            
        elif(int(data.data) == 0):
            status = ["BMS_Status_flag/Normal"]
        else:
            status = check_states(status_naming, state=int(data.data))    
    except ValueError:
        pass
    
    #rospy.loginfo(status)
    robot_status.bms_status.status_flag = status
    status_pub.publish(robot_status)

def bms_fault_flags_callback(data):
    """
        [Callback function of Fault flag topic]
        Arg:
            data(str):fault_flag_number(2^(n-1))
        Publishes Name of fault to topic 'robot_status' 
    """
    status_naming = ["roboteq_bms/OC Load", "roboteq_bms/Short Load", "roboteq_bms/OC Charger",
                     "roboteq_bms/Inverse Charger", "roboteq_bms/OC Pack In", "roboteq_bms/OC Pack Out", 
                     "roboteq_bms/Internal Fault", "roboteq_bms/Config Error"]

    try:
        if data.data == '':
            status = ["BMS_Fault_flag/Empty"]
            pass
        elif(int(data.data) == 0):
            status = ["BMS_Fault_flag/Normal"]
        else:
            status = check_states(status_naming, state=int(data.data))    
    except ValueError:
        pass
    
    #rospy.loginfo(status)
    robot_status.bms_status.fault_flag = status
    status_pub.publish(robot_status)

def bms_data_callback(data):
    if data.min_cell > 3.4:
        robot_status.bms_status.battery = "High level"
    elif data.min_cell < 3.4 and data.min_cell >= 3:
        robot_status.bms_status.battery = "Medium level" 
    elif data.min_cell < 3 and data.min_cell > 2.9:
        robot_status.bms_status.battery = "Low level"
    elif data.min_cell <= 2.9:
        robot_status.bms_status.battery = "Critical level"    
    robot_status.bms_status.charging = data.is_charging
    status_pub.publish(robot_status)

def move_base_status_callback(msg):
    """
        [Callback function of Move_Base_Status topic]
        Arg:
            msg(int): 
                uint8 PENDING=0
                uint8 ACTIVE=1
                uint8 PREEMPTED=2
                uint8 SUCCEEDED=3
                uint8 ABORTED=4
                uint8 REJECTED=5
                uint8 PREEMPTING=6
                uint8 RECALLING=7
                uint8 RECALLED=8
                uint8 LOST=9
        Publishes Name of status to topic 'robot_status' 
    """   
    status_msg = GoalStatusArray()
    status_msg = msg
    status = []
    status_naming_list = ["Move_base/PENDING", "Move_base/ACTIVE", "Move_base/PREEMPTED", "Move_base/SUCCEEDED",
                         "Move_base/ABORTED", "Move_base/REJECTED", "Move_base/PREEMPTING", "Move_base/RECALLING", 
                         "Move_base/RECALLED", "Move_base/LOST"]
    status_index = status_msg.status_list[0].status
    status.append(status_naming_list[status_index])
    #rospy.loginfo(status)
    status_pub.publish(status)

def listener():
    """
        Initialize node and Subscribes topics
    """
    global status_pub, robot_status 
    robot_status = RobotStatus()
    rospy.init_node('Robot_status', anonymous=True)
    status_pub = rospy.Publisher('robot_status', RobotStatus, queue_size=10)   
    rospy.Subscriber('driver/fault_flag', channel_values, fault_flags_callback)
    rospy.Subscriber('driver/status_flag', channel_values, status_flags_callback)
    rospy.Subscriber('bms/status_flags', String, bms_status_flags_callback)
    rospy.Subscriber('bms/fault_flags', String, bms_fault_flags_callback)
    rospy.Subscriber('bms/data', BatteryStatus, bms_data_callback)
    #rospy.Subscriber('move_base_status', GoalStatusArray, move_base_status_callback)
    rospy.spin()
   
if __name__ == '__main__':
    listener()
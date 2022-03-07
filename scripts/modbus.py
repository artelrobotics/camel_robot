#!/usr/bin/env python3
import re
import rospy
from ethernet_remote_io_module.msg import WriteCoil, ReadDigitalInputs, WriteCoils
from pymodbus.client.sync import ModbusTcpClient
from rospy import loginfo, logerr, logwarn, loginfo_once
import threading
import time


def validate_ip(ip: str) -> bool:
    regex = r"^((25[0-5]|2[0-4][0-9]|1[0-9][0-9]|[1-9]?[0-9])\.){3}(25[0-5]|2[0-4][0-9]|1[0-9][0-9]|[1-9]?[0-9])$"
    return bool(re.search(regex, ip))

def shutdown_hook() -> None:
    if client.connect():
        flag = client.write_coils(address=0, values=[False]*8, unit=0x01)
        if flag:
            client.close()
    else:
        pass

def write_coil(msg: WriteCoil) -> None:
    address = msg.address
    value = msg.value
    client.write_coil(address=address, value= value, unit=0x01)

def write_coils(msg: WriteCoils) -> None:
    start_address = msg.address
    end_address = msg.last
    value = msg.value
    client.write_coils(address=start_address, values=[value]*end_address, unit=0x01)
    
def read_inputs() -> None:
    while not rospy.is_shutdown():
        pub_msg = ReadDigitalInputs()
        res = client.read_discrete_inputs(address=0, count=8, unit=0x01)
        pub_msg.all_inputs = res.bits
        pub.publish(pub_msg)
        rate.sleep()


if __name__ == "__main__":
    rospy.init_node('write_coil', anonymous=True, disable_signals=True)
    rospy.on_shutdown(shutdown_hook)
    if not rospy.has_param('~ip_address'):
        logwarn('IP address of modbus server not specified')
        logwarn('Try to connect to default address')
    if not rospy.has_param('~port'):
        logwarn('Modbus server port not specified')
        logwarn('Using default port')
    ip_address = rospy.get_param('~ip_address', default='192.168.1.111')
    port = rospy.get_param('~port', default=502)
    if validate_ip(ip_address):
        loginfo(f'Modbus Server IP= {ip_address} port= {port}')
        loginfo('Connecting...')
        client = ModbusTcpClient(host=ip_address, port=port)
        if client.connect():
            loginfo('Connection Success')
            rate = rospy.Rate(10)
            rospy.Subscriber("write_coil", WriteCoil, callback=write_coil, queue_size=1)
            rospy.Subscriber("write_coils", WriteCoils, callback=write_coils, queue_size=1)
            pub = rospy.Publisher("read_inputs", ReadDigitalInputs, queue_size=1)
            thread = threading.Thread(target=read_inputs)
            thread.start()
            rospy.spin()
        else:
            logerr(f"Can't connect to {ip_address}")
            rospy.signal_shutdown('Wrong Modbus Server IP address or port')
    else:
        logerr(f'IP address validation error. Wrong IP {ip_address}')
        rospy.signal_shutdown('IP address validation error')
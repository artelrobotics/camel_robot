#!/usr/bin/env python3
import rospy
from ethernet_remote_io_module.msg import WriteCoil, ReadDigitalInputs, WriteCoils
import time

if __name__ == "__main__":
    rospy.init_node('pub_coil', anonymous=True)
    rate = rospy.Rate(10)
    pub = rospy.Publisher("/camel_amr_1000_001/common/write_coil", WriteCoil, queue_size=1)
    pub_msg = WriteCoil()
    while not rospy.is_shutdown():
        start_time = time.time()
        pub_msg.address = 1
        pub_msg.value = True
        pub.publish(pub_msg)
        rospy.loginfo(f"elapsed time {time.time() - start_time}")
        time.sleep(0.05)
        start_time = time.time()
        pub_msg.address = 1
        pub_msg.value = False
        pub.publish(pub_msg)
        rospy.loginfo(f"elapsed time {time.time() - start_time}")
        time.sleep(0.05)
        rate.sleep()

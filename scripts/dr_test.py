#!/usr/bin/env python
import rospy
import dynamic_reconfigure.client

def callback(config):
    rospy.loginfo(config)

if __name__ == "__main__":
    rospy.init_node("dynamic_client")
    client = dynamic_reconfigure.client.Client("/camel_amr_1000_001/move_base/local_costmap", timeout = 10, config_callback=callback)
    rate = rospy.Rate(0.1)

    while not rospy.is_shutdown:
        client.update_configuration({"groups": {'footprint': '[[-1.19,-0.70],[-1.19,0.65],[1.19,0.65],[1.19,-0.65]]'}})
        rate.sleep()


        # {'groups': 
        # {'origin_y': -3.0, 
        # 'parent': 0, 
        # 'footprint_padding': 0.01, 
        # 'height': 10, 
        # 'update_frequency': 10.0, 
        # 'groups': {},
        #  'origin_x': 0.0, 
        #  'footprint': '[[-1.19,-0.65],[-1.19,0.65],[1.19,0.65],[1.19,-0.65]]', 'id': 0, 'name': 'Default', 'parameters': {}, 'resolution': 0.1, 'robot_radius': 0.46, 'transform_tolerance': 0.4, 'width': 10, 'state': True, 'type': '', 'publish_frequency': 2.0}, 'footprint_padding': 0.01, 'transform_tolerance': 0.4, 'width': 10, 'update_frequency': 10.0, 'origin_y': -3.0, 'origin_x': 0.0, 'footprint': '[[-1.19,-0.65],[-1.19,0.65],[1.19,0.65],[1.19,-0.65]]', 'height': 10, 'resolution': 0.1, 'robot_radius': 0.46, 'publish_frequency': 2.0}
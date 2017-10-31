#!/usr/bin/env python
import rospy
import time
import numpy as np
import math
import sys
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from brics_actuator.msg import JointPositions
from brics_actuator.msg import JointValue

# This script is testing callback functions

def get_a(arr):
	global a
	a = arr
 

def callback(data):
    if (len(data.name) == 7):
        get_a(data.position[0:6])
        return data.position[0:6]

def main():
    arm_sub = rospy.Subscriber('/joint_states', JointState , callback)
    print("Initializing state listener")
    rospy.init_node('state_listener')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    
if __name__ == '__main__':
    main()



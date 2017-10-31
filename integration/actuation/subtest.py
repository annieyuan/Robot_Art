#!/usr/bin/env python

import rospy
import time
import numpy as np
import math
import sys
from std_msgs.msg import String
from sensor_msgs.msg import JointState

# This script is testing a subscriber node

def callback(data):
    print "hahah"
    if (len(data.name) == 7 or len(data.name) == 8):
        print data.name
        print data.position

def main(argv):        

    rospy.init_node('youbot_arm_control')
    print 1
    arm_sub = rospy.Subscriber('/joint_states', JointState , callback)
    print 2
    rospy.spin()

 

if __name__ == '__main__':
    main(sys.argv)
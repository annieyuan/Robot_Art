#!/usr/bin/env python

import rospy
import time
import numpy as np
import math
import sys
import ik_solver_service as ik
from ik_solver_service.srv import*


# This script is testing rosservice

def main():        
    # help(ikmsg)
    # Initialize waypoint list subscriber
    rospy.init_node('youbot_arm_control')

    print("Initializing robot state subscriber")
    
    rospy.wait_for_service('/solve_closest_ik')
    try:
        arm_ik_sol = rospy.ServiceProxy('/solve_closest_ik', SolveClosestIK)
        sol = arm_ik_sol([1,1,1,1,1],[1,1,1],[1,1,1])
        print sol.joint_angles
    except rospy.ServiceException, e:
        print "didnt get any shit"
    
    rate = rospy.Rate(20)


    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
 

if __name__ == '__main__':
    main()
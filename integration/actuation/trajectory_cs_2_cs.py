#!/usr/bin/env python
import rospy
import time
import numpy as np
import math
import sys
from std_msgs.msg import String
import ik_solver_service as ik
from ik_solver_service.srv import*
from sensor_msgs.msg import JointState
from brics_actuator.msg import JointPositions
from brics_actuator.msg import JointValue
from trajectory_generator.srv import*
from trajectory_generator.srv import CStoCS
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
import arm_fk_control as afc
'''
This script is supposed to return the velocity messages for going from an xyz position to another xyz position in Cartesian Space
However, it never returns a feasible solution. Please see js_2_js instead
'''

def main():
    a = CStoCS()
    value = afc.main()

    pos_arg = [Pose(),Pose()]
    pos_arg[0].position = Point()
    pos_arg[0].orientation = Quaternion()
    pos_arg[0].position.x = value[0]
    pos_arg[0].position.y = value[1]
    pos_arg[0].position.z = value[2]
    pos_arg[0].orientation.x = value[3]
    pos_arg[0].orientation.y = value[4]
    pos_arg[0].orientation.z = value[5]
    pos_arg[0].orientation.w = value[6]
    a.start_pos = pos_arg[0]
    pos_arg[1].position = Point()
    pos_arg[1].orientation = Quaternion()
    pos_arg[1].position.x = value[7]
    pos_arg[1].position.y = value[8]
    pos_arg[1].position.z = value[9]
    pos_arg[1].orientation.x = value[10]
    pos_arg[1].orientation.y = value[11]
    pos_arg[1].orientation.z = value[12]
    pos_arg[1].orientation.w = value[13]
    a.end_pos = pos_arg[1]
    a.start_vel = 0.0
    a.end_vel = 0.0
    a.max_vel = 0.05;
    a.max_acc = 0.5;
    #rospy.wait_for_service('/trajectory_generator')
    
    arm_ik_sol = rospy.ServiceProxy('/From_CS_to_CS', CStoCS)
    sol = arm_ik_sol(a.start_pos,a.end_pos,a.start_vel,a.end_vel,a.max_vel,a.max_acc)
    print sol


if __name__ == '__main__':
    main()



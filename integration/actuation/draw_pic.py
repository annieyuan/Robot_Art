#!/usr/bin/env python
import sys
import rospy
import tf
from brics_actuator.msg import JointPositions
from brics_actuator.msg import JointValue
from trajectory_js_2_js import draw_line_smooth
from trajectory_js_2_js import smudge
import arm_fk_control as afc
import arm_ik_control as aic
from draw_art import*
from math import sqrt
import json
import parser


canvas_height = 0.3
canvas_length = 0.3
img_height = 1500.0
img_length = 1500.0

def get_scale():
    return canvas_height/img_height

def draw_pic():
    arm_pub = rospy.Publisher('/arm_1/arm_controller/position_command', JointPositions, queue_size = 10)
    M = parser.main("/home/youbot/catkin_ws/src/robot_art/jsonfile/2str.json")
    scale = get_scale()

    for i in range(0,len(M)):
        de_xs = 0.32
        de_xe = 0.32
        de_ys = M[i][1][0]*scale
        de_zs = M[i][1][1]*scale+0.2
        de_ye = M[i][2][0]*scale
        de_ze = M[i][2][1]*scale+0.2
        print "drawing from (" + str(de_xs) + "," + str(de_ys)+ ","+ str(de_zs) + ") to (" + str(de_xe) + ","+ str(de_ye) + ","+ str(de_ze) + ")"
        smudge(de_xs, de_ys, de_zs, de_xe, de_ye, de_ze)
        aic.lift_pen(de_xe, de_ye, de_ze, arm_pub)
    #draw_line_smooth(0.3,0.3,0.3,0.0,0.0,0.6)

def main():

    rospy.init_node('draw_art', anonymous=True)
    
    draw_pic()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    


if __name__ == '__main__':
    main()
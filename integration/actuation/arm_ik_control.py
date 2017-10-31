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
import random
import oop_draw_pic


# Publish an array of message from joint 1 to joint 5
#
# Params:
# pub - the publisher for joint angles
# msg_arr - the array of messages specifying the angle of each joint
def pub_msg_arr(pub, msg_arr):
    #print msg_arr
    rate = rospy.Rate(20)
    for i in range(5):
        if (math.isnan(msg_arr[i].positions[0].value)):
            pass
        else:
            pub.publish(msg_arr[i])
            rate.sleep()
            time.sleep(0.5)
            pub.publish(msg_arr[i])
      

# Publish an array of message from joint 5 to joint 1
#
# Params:
# pub - the publisher for joint angles
# msg_arr - the array of messages specifying the angle of each joint
def pub_msg_arr_rev(pub, msg_arr):
    rate = rospy.Rate(20)
    for i in range(5):
        if (math.isnan(msg_arr[4-i].positions[0].value)):
            pass
        else:
            pub.publish(msg_arr[4-i])
            rate.sleep()
            pub.publish(msg_arr[4-i])


# Generating messages to be published based on joint angle
#
# Params:
# joint_angles - an array of joint angles, NaN for joints that do not move
#
# Returns:
# msg_arr - an array of JointPositions() messages
def gen_joint_msg(joint_angles):
    msg_arr = [0 for i in range(5)]
    for i in range(5):
        msg_arr[i] = JointPositions()
        msg_arr[i].positions = [JointValue()]
        msg_arr[i].positions[0].timeStamp = rospy.Time.now()
        msg_arr[i].positions[0].joint_uri = "arm_joint_"+str(i+1)
        msg_arr[i].positions[0].unit = "rad"
        # not sure about this, truncate the angles in the ik solution when they are out of range. Usually the difference is small
        if (i == 3 and joint_angles[i] > 3.429):
            msg_arr[i].positions[0].value = 3.429
        elif (i == 2 and joint_angles[i] > -0.0157):
            msg_arr[i].positions[0].value = -0.01571
        else:
            msg_arr[i].positions[0].value = joint_angles[i]
    return msg_arr


# IK Solver based on an initial guess of joint angles
# Have not gotten this to work well
#
# Returns:
# sol - an array of joint angles to achieve the desired end effector position
def solve_closest_ik():
    rospy.wait_for_service('/solve_closest_ik')
    try:
        arm_ik_sol = rospy.ServiceProxy('/solve_closest_ik', SolveClosestIK)
        flag = True
        de_norm1 = 0.0
        de_norm2 = 0.0
        de_norm3 = 0.0

        while (flag):
            sol = arm_ik_sol([1.967,0.97,-0.812,1.788,1.35],[-0.3,0.2,0.4],[de_norm1,de_norm2,de_norm3])
            if (sol.feasible == True):
                flag = False
                print sol.joint_angles
                return sol
            else:
                print sol.feasible
                de_norm1 = random.random()
                de_norm2 = random.random()
                de_norm3 = random.random()
    except rospy.ServiceException, e:
        print "Error in calling service"


# IK Solver based orientation and xyz position, searches for solution with a pitch closest to 0.0
# orientation de_norm1-3 are set to 0.0. This makes them slack variables
#
# Params:
# de_x - desired x position in Cartesian Coordinate
# de_y - desired y position in Cartesian Coordinate
# de_z - desired z position in Cartesian Coordinate
#
# Returns:
# sol - an array of joint angles to achieve the desired end effector position
def solve_fully_constrained_ik(de_x,de_y,de_z):
    rospy.wait_for_service('/solve_fully_constrained_ik')
    try:
        arm_ik_sol = rospy.ServiceProxy('/solve_fully_constrained_ik', SolveFullyConstrainedIK)
        flag = True
        call_id = np.uint8(1)
        # de_norm1-3 are specify the orientation of the end-effector. They are slack variables when passed in 0.0
        # DO NOT change these values. Write a new function if you need to
        de_norm1 = 0.0
        de_norm2 = 0.0
        de_norm3 = 0.0
        pitch = 0.0
        counter = 0
        while(flag):
            counter += 1
            
            # solving for inverse kinematics solution
            sol = arm_ik_sol(call_id,pitch,[de_x,de_y,de_z],[de_norm1,de_norm2,de_norm3])
            #Searching for a feasible pitch starting from 0, expand in both signs
            if (sol.feasible == True):
                flag = False
                #print sol.joint_angles
                return sol
            else:
                #print pitch
                if (counter%2 == 0):
                    pitch = pitch*(-1) 
                    #pitch -= 0.05
                    #big step size to increase search speed
                    pitch -= 0.05
                else:
                    pitch = pitch*(-1)
                if (pitch < -1.5 or pitch > 1.5):
                    flag = False

    except rospy.ServiceException, e:
        print "Error in calling service"







#does the same thing as the function above but solves with the end effector at 90 degrees
def solve_fully_constrained_ik_vert(de_x,de_y,de_z):
    rospy.wait_for_service('/solve_fully_constrained_ik')
    try:
        arm_ik_sol = rospy.ServiceProxy('/solve_fully_constrained_ik', SolveFullyConstrainedIK)
        flag = True
        call_id = np.uint8(1)
        # de_norm1-3 are specify the orientation of the end-effector. They are slack variables when passed in 0.0

        de_norm1 = 0.0
        de_norm2 = 0.0
        de_norm3 = 0.0

	degree = 90.0
        desired_pitch = degree

	pitch = 0.0
        counter = 0
        while(flag):
            counter += 1
            
            # solving for inverse kinematics solution
            sol = arm_ik_sol(call_id,desired_pitch,[de_x,de_y,de_z],[de_norm1,de_norm2,de_norm3])
            #Searching for a feasible pitch starting from 0, expand in both signs
            if (sol.feasible == True):
                flag = False
                #print sol.joint_angles
                return sol
            else:
                #print pitch
                if (counter%2 == 0):

                    pitch = pitch*(-1)
                    pitch -= 0.05
		    desired_pitch = degree + pitch
                else:
                    pitch = pitch*(-1)
		    desired_pitch = degree + pitch
                if (desired_pitch < (degree - 1.5) or desired_pitch > (degree + 1.5)):
                    flag = False

    except rospy.ServiceException, e:
        print "Error in calling service"

#go to Cartesian point with end effector pointed downward
def go_to_xyz_vert(x,y,z,arm_pub):
    sol = solve_fully_constrained_ik_vert(x,y,z)
    #print sol.feasible
    try:
        if (sol.feasible == True):
            msg_arr = gen_joint_msg(sol.joint_angles)
            pub_msg_arr(arm_pub, msg_arr)
        else:
            pass
	return True
    except AttributeError:
        print "Point Outside Reachable Space"
	return False







# Make the end effector go to a point based on xyz position in cartesian space and the publisher
#
# Params:
# x - desired x position in Cartesian Coordinate
# y - desired y position in Cartesian Coordinate
# z - desired z position in Cartesian Coordinate
# arm_pub - Publisher for position
def go_to_xyz(x,y,z,arm_pub):
    sol = solve_fully_constrained_ik(x,y,z)
    #print sol.feasible
    try:
        if (sol.feasible == True):
            msg_arr = gen_joint_msg(sol.joint_angles)
            pub_msg_arr(arm_pub, msg_arr)
        else:
            pass
	return True
    except AttributeError:
        print "Point Outside Reachable Space"
	return False


# Make the end effector go to a point based on xyz position in cartesian space and the publisher, publish from joint 5 to joint 1
#
# Params:
# x - desired x position in Cartesian Coordinate
# y - desired y position in Cartesian Coordinate
# z - desired z position in Cartesian Coordinate
# arm_pub - Publisher for position
def go_to_xyz_rev(x,y,z,arm_pub):
    sol = solve_fully_constrained_ik(x,y,z)
    #print sol.feasible
    try:
        if (sol.feasible == True):
            msg_arr = gen_joint_msg(sol.joint_angles)
            pub_msg_arr_rev(arm_pub, msg_arr)
        else:
            pass
    except AttributeError:
        print "Point Outside Reachable Space"



# Make the end effector go away from the canvas temporily, 
# You can manually change how far you want the arm to go back
# and how much you want the arm to sleep after lifting
#
# Params:
# x - x position in Cartesian Coordinate, i.e. the set plane for canvas
# curr_y - current y position in Cartesian Coordinate
# curr_z - current z position in Cartesian Coordinate
# arm_pub - Publisher for position
def lift_pen(x, curr_y, curr_z, arm_pub):
    go_to_xyz_rev(x-0.1, curr_y, curr_z, arm_pub)
    time.sleep(0.5)

# does the same thing as the previous function but assuming the end effector is pointed downward
def lift_pen_vert(x, curr_y, curr_z, arm_pub):
    go_to_xyz_vert(x, curr_y, curr_z+0.1, arm_pub)
    time.sleep(0.5)



# Draw line based on the start and end position
#
# Params:
# x - x position in Cartesian Coordinate, i.e. the set plane for canvas
# start_yz - the y and z coordinate of the starting point
# end_yz - the y and z coordinate of the ending point
# arm_pub - Publisher for position
# lift - boolean for whether or not to lift the pen after each stroke. Lift pen if True
def draw_line(x, start_yz, end_yz, arm_pub, lift):
    length = math.sqrt(math.pow((end_yz[0]-start_yz[0]),2)+math.pow((end_yz[1]-start_yz[1]),2))
    num_step = int(length*50)
    print str(num_step)+" of steps"
    step_y_size = (end_yz[0]-start_yz[0])/num_step
    step_z_size = (end_yz[1]-start_yz[1])/num_step
    go_to_xyz(x, start_yz[0], start_yz[1], arm_pub)
    rospy.sleep(0.2)
    curr_y = start_yz[0]
    curr_z = start_yz[1]

    for i in range(num_step):
        if (lift == True):
            lift_pen(x, curr_y, curr_z, arm_pub)
        curr_y = curr_y + step_y_size
        curr_z = curr_z + step_z_size
        go_to_xyz_rev(x, curr_y, curr_z, arm_pub)
        #time.sleep(0.5)
    go_to_xyz(x, end_yz[0], end_yz[1], arm_pub)




def draw_line_from_vert(x, start_yz, end_yz, arm_pub, lift):
    length = math.sqrt(math.pow((end_yz[0]-start_yz[0]),2)+math.pow((end_yz[1]-start_yz[1]),2))
    num_step = int(length*50)
    print str(num_step)+" of steps"
    step_y_size = (end_yz[0]-start_yz[0])/num_step
    step_z_size = (end_yz[1]-start_yz[1])/num_step
    go_to_xyz_vert(x, start_yz[0], start_yz[1], arm_pub)
    rospy.sleep(0.2)
    curr_y = start_yz[0]
    curr_z = start_yz[1]

    for i in range(num_step):
        if (lift == True):
            lift_pen_vert(x, curr_y, curr_z, arm_pub)
        curr_y = curr_y + step_y_size
        curr_z = curr_z + step_z_size
        go_to_xyz_vert(x, curr_y, curr_z, arm_pub)
        #time.sleep(0.5)
    go_to_xyz_vert(x, end_yz[0], end_yz[1], arm_pub)



def main():

    # initializing arm message
    print("Initializing arm control node")
    rospy.init_node('arm_ik_control')
    # Publisher for joint angles
    arm_pub = rospy.Publisher('/arm_1/arm_controller/position_command', JointPositions, queue_size = 10)
    # color calibrarion
    '''
    lift_pen_vert(0, 0.3, 0.188, arm_pub)
    go_to_xyz_vert(0, 0.3, 0.188, arm_pub)
    lift_pen_vert(0, 0.3, 0.189, arm_pub)
    lift_pen_vert(-0.07, 0.285, 0.188, arm_pub)
    go_to_xyz_vert(-0.07, 0.285, 0.188, arm_pub)
    lift_pen_vert(-0.07, 0.285, 0.188, arm_pub)
    '''
    
    #go_to_xyz(0, 0, 0.6, arm_pub)
    
    '''
    #water position
    lift_pen_vert(0.1, -0.25, 0.13, arm_pub)
    go_to_xyz_vert(0.1, -0.25, 0.13, arm_pub)
    
    #wiggle--for water and paint
    go_to_xyz_vert(0.1, -0.25+0.005, 0.13, arm_pub)
    go_to_xyz_vert(0.1, -0.25-0.005, 0.13, arm_pub)
    lift_pen_vert(0.1, -0.25, 0.13, arm_pub)
    '''
    go_to_xyz(0.3,0.3,0.3,arm_pub)

    #clean_init position
    #go_to_xyz_vert(0, -0.3, 0.22, arm_pub)

    #art = oop_draw_pic.paint()
    #art.clean_brush()
    #print "done"

    #go_to_xyz_vert(0, 0.3, 0.258, arm_pub)
    #go_to_xyz_vert(-0.07, 0.285, 0.238, arm_pub)
    #lift_pen_vert(-0.14, 0.312, 0.238, arm_pub)



    #ok blue
    '''
    go_to_xyz_vert(-0.07, 0.16, 0.18, arm_pub)
    lift_pen_vert(-0.07, 0.16, 0.18, arm_pub)
    #blue
    go_to_xyz_vert(-0.03, 0.31, 0.18, arm_pub)
    lift_pen_vert(-0.03, 0.31, 0.18, arm_pub)
    #light blue
    go_to_xyz_vert(-0.10, 0.312, 0.18, arm_pub)
    lift_pen_vert(-0.10, 0.312, 0.18, arm_pub)
    #yellow
    go_to_xyz_vert(0, 0.25, 0.19, arm_pub) 
    lift_pen_vert(0, 0.25, 0.19, arm_pub)
    #very dark blue
    go_to_xyz_vert(-0.07, 0.25, 0.19,arm_pub)
    lift_pen_vert(-0.07, 0.25, 0.19,arm_pub)
    #grey
    go_to_xyz_vert(-0.15, 0.26, 0.185, arm_pub)
    lift_pen_vert(-0.15, 0.26, 0.185, arm_pub)
    #go_to_xy(0.2, 0, 0.5, arm_pub)    
    '''

    #go_to_xyz_vert(0.1, 0.4, 0.1, arm_pub)
    '''
    lift_pen_vert(0.1, 0.2, 0.1, arm_pub)
    go_to_xyz_vert(0.1, 0.2, 0.1, arm_pub)
    lift_pen_vert(0.1, 0.2, 0.1, arm_pub)
    '''
    
    #go_to_xyz_rev(0.3,-0.3,0.7,arm_pub)

    #draw_line(0.3,(0.3,0.3),(-0.3,0.3), arm_pub, True)


    

  
 
    '''
    go_to_xyz_rev(0.1,0.0,0.6, arm_pub)
    go_to_xyz_rev(0.1,0.0,0.598, arm_pub)
    go_to_xyz_rev(0.1,0.0,0.596, arm_pub)
    go_to_xyz_rev(0.1,0.0,0.594, arm_pub)
    go_to_xyz_rev(0.1,0.0,0.592, arm_pub)
    go_to_xyz_rev(0.1,0.0,0.590, arm_pub)
    go_to_xyz_rev(0.1,0.0,0.588, arm_pub)
    go_to_xyz_rev(0.1,0.0,0.586, arm_pub)
    go_to_xyz_rev(0.1,0.0,0.584, arm_pub)
    go_to_xyz_rev(0.1,0.0,0.582, arm_pub)
    go_to_xyz_rev(0.1,0.0,0.580, arm_pub)
    go_to_xyz_rev(0.1,0.0,0.578, arm_pub)
    go_to_xyz_rev(0.1,0.0,0.576, arm_pub)
    go_to_xyz_rev(0.1,0.0,0.574, arm_pub)
    '''
    '''
    go_to_xyz(0.1,0.0,0.6, arm_pub)
    go_to_xyz(0.1,0.0,0.598, arm_pub)
    go_to_xyz(0.1,0.0,0.596, arm_pub)
    go_to_xyz(0.1,0.0,0.594, arm_pub)
    go_to_xyz(0.1,0.0,0.592, arm_pub)
    go_to_xyz(0.1,0.0,0.590, arm_pub)
    go_to_xyz(0.1,0.0,0.588, arm_pub)
    go_to_xyz(0.1,0.0,0.586, arm_pub)
    go_to_xyz(0.1,0.0,0.584, arm_pub)
    go_to_xyz(0.1,0.0,0.582, arm_pub)
    go_to_xyz(0.1,0.0,0.580, arm_pub)
    go_to_xyz(0.1,0.0,0.578, arm_pub)
    go_to_xyz(0.1,0.0,0.576, arm_pub)
    go_to_xyz(0.1,0.0,0.574, arm_pub)
    '''
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    

if __name__ == '__main__':
    main()

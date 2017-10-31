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
'''
This script is exactly the same as arm_ik_control.py except that this is object-oriented
There is an additional callback function that can be used for when subscribing to joint messages.
Call methods in __init__ to actuate the robot
'''

class arm_ik:
    def __init__(self):
        print("Arm IK initialized")
        # Initializing publishers, subscribers, and call services for ik
        self.arm_pub = rospy.Publisher('/arm_1/arm_controller/position_command', JointPositions, queue_size = 10)
        self.arm_ik_closest_sol = rospy.ServiceProxy('/solve_closest_ik', SolveClosestIK)
        self.arm_ik_fully_constr_sol = rospy.ServiceProxy('/solve_fully_constrained_ik', SolveFullyConstrainedIK)

        # Uncomment this if you need to use callback to read sensor data
        #self.arm_sub = rospy.Subscriber('/joint_states', JointState , self.callback)
        
        self.curr_joint_angles = [0 for i in range(5)]
        self.rate = rospy.Rate(20)
        self.msg_arr = [0 for i in range(5)]

        self.go_to_xyz(0.3, 0.3, 0.3)

        #self.draw_line(0.3, (0.3,0.3),(-0.3,0.3), False)
        #self.draw_line(0.35, (-0.2,0.4),(0.2,0.4), False)
        #self.draw_line(0.35, (0.2,0.4),(0.2,0.2), False)
        #self.draw_line(0.35, (0.2,0.2),(-0.2,0.2), False)


    # Callback function subscriber. Read sensor messages for the current joint angles

    def callback(self, data):
        if (len(data.name) == 7):
            print data.position
            [a,b,c,d,e,f,g] = data.position
            self.curr_joint_angles = [a,b,c,d,e]
            self.msg_arr = self.gen_joint_msg(self.curr_joint_angles)
             
            self.pub_msg_arr()


    # Publish an array of message from joint 1 to joint 5
    #
    # Params:
    # msg_arr - the array of messages specifying the angle of each joint

    def pub_msg_arr(self):
        for i in range(5):
            if (math.isnan(self.msg_arr[i].positions[0].value)):
                pass
            else:
                self.arm_pub.publish(self.msg_arr[i])
                self.rate.sleep()
                self.arm_pub.publish(self.msg_arr[i])
          

    # Publish an array of message from joint 5 to joint 1
    #
    # Params:
    # msg_arr - the array of messages specifying the angle of each joint

    def pub_msg_arr_rev(self):
        for i in range(5):
            if (math.isnan(self.msg_arr[i].positions[0].value)):
                pass
            else:
                self.arm_pub.publish(self.msg_arr[i])
                self.rate.sleep()
                self.arm_pub.publish(self.msg_arr[i])


    # Generating messages to be published based on joint angle
    #
    # Params:
    # joint_angles - an array of joint angles, NaN for joints that do not move
    #
    # Returns:
    # msg_arr - an array of JointPositions() messages

    def gen_joint_msg(self, joint_angles):
        for i in range(5):
            self.msg_arr[i] = JointPositions()
            self.msg_arr[i].positions = [JointValue()]
            self.msg_arr[i].positions[0].timeStamp = rospy.Time.now()
            self.msg_arr[i].positions[0].joint_uri = "arm_joint_"+str(i+1)
            self.msg_arr[i].positions[0].unit = "rad"
            #not sure about this, making the angle value right when out of range by truncating
            if (i == 3 and joint_angles[i] > 3.429):
                self.msg_arr[i].positions[0].value = 3.429
            elif (i == 2 and joint_angles[i] > -0.0157):
                self.msg_arr[i].positions[0].value = -0.01571
            else:
                self.msg_arr[i].positions[0].value = joint_angles[i]
        return self.msg_arr

    # IK Solver based on an initial guess of joint angles
    # Have not gotten this to work well
    #
    # Returns:
    # sol - an array of joint angles to achieve the desired end effector position

    def solve_closest_ik(self):
        rospy.wait_for_service('/solve_closest_ik')
        try:
            flag = True
            de_norm1 = 0.0
            de_norm2 = 0.0
            de_norm3 = 0.0

            while (flag):
                sol = self.arm_ik_closest_sol([1.967,0.97,-0.812,1.788,1.35],[-0.3,0.2,0.4],[de_norm1,de_norm2,de_norm3])
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

    def solve_fully_constrained_ik(self, de_x,de_y,de_z):
        rospy.wait_for_service('/solve_fully_constrained_ik')
        try:
            flag = True
            call_id = np.uint8(1)
            de_norm1 = 0.0
            de_norm2 = 0.0
            de_norm3 = 0.0
            pitch = 0.0
            counter = 0
            while(flag):
                counter += 1
                sol = self.arm_ik_fully_constr_sol(call_id,pitch,[de_x,de_y,de_z],[de_norm1,de_norm2,de_norm3])
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
                    else:
                        pitch = pitch*(-1)
                    if (pitch < - 1.5 or pitch > 1.5):
                        flag = False
        except rospy.ServiceException, e:
            print "Error in calling service"


    # Make the end effector go to a point based on xyz position in cartesian space and the publisher
    #
    # Params:
    # x - desired x position in Cartesian Coordinate
    # y - desired y position in Cartesian Coordinate
    # z - desired z position in Cartesian Coordinate

    def go_to_xyz(self, x,y,z):
        sol = self.solve_fully_constrained_ik(x,y,z)
        #print sol.feasible
        try:
            if (sol.feasible == True):
                self.msg_arr = self.gen_joint_msg(sol.joint_angles)
                self.pub_msg_arr()
            else:
                pass
        except AttributeError:
            print "Point Outside Reachable Space"

    # Make the end effector go to a point based on xyz position in cartesian space and the publisher, publish from joint 5 to joint 1
    #
    # Params:
    # x - desired x position in Cartesian Coordinate
    # y - desired y position in Cartesian Coordinate
    # z - desired z position in Cartesian Coordinate

    def go_to_xyz_rev(self, x,y,z):
        sol = self.solve_fully_constrained_ik(x,y,z)
        #print sol.feasible

        try:
            if (sol.feasible == True):
                self.msg_arr = self.gen_joint_msg(sol.joint_angles)
                self.pub_msg_arr_rev()
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

    def lift_pen(self, x, curr_y, curr_z):
        self.go_to_xyz_rev(x-0.1, curr_y, curr_z)


    # Draw line based on the start and end position
    #
    # Params:
    # x - x position in Cartesian Coordinate, i.e. the set plane for canvas
    # start_yz - the y and z coordinate of the starting point
    # end_yz - the y and z coordinate of the ending point
    # lift - boolean for whether or not to lift the pen after each stroke. Lift pen if True
    
    def draw_line(self, x, start_yz, end_yz, lift):
        length = math.sqrt(math.pow((end_yz[0]-start_yz[0]),2)+math.pow((end_yz[1]-start_yz[1]),2))
        num_step = int(length*50)
        print str(num_step)+" of steps"
        step_y_size = (end_yz[0]-start_yz[0])/num_step
        step_z_size = (end_yz[1]-start_yz[1])/num_step
        self.go_to_xyz(x, start_yz[0], start_yz[1])
        rospy.sleep(0.2)
        curr_y = start_yz[0]
        curr_z = start_yz[1]

        for i in range(num_step):
            if (lift == True):
                self.lift_pen(x, curr_y, curr_z)
            curr_y = curr_y + step_y_size
            curr_z = curr_z + step_z_size
            self.go_to_xyz_rev(x, curr_y, curr_z)
        self.go_to_xyz(x, end_yz[0], end_yz[1])


def main(args):
    # initializing arm message
    rospy.init_node('arm_ik_control')
    print("arm_ik_control node initialized")
    implemented_ik_control = arm_ik()
    
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    

if __name__ == '__main__':
    main(sys.argv)

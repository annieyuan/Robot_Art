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
from brics_actuator.msg import JointVelocities
from trajectory_generator.srv import*
from trajectory_generator.srv import CStoCS
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from trajectory_msgs.msg import JointTrajectoryPoint
import arm_fk_control as afc
import arm_ik_control as aic
from datetime import datetime

# Generating position messages to be published based on joint angle array
#
# Params:
# joint_angles - an array of joint angles, NaN for joints that do not move
#
# Returns:
# msg_arr - an array of JointPositions() messages

class velocityController:

    def __init__(self,x,y,z):
        rospy.init_node('arm_ik_control')
        self.curr_joint_angles = [0 for i in range(5)]
        self.p_gain = 0.08
	# change the 3-tuple below to go to a point
        self.sol = aic.solve_fully_constrained_ik(x,y,z)
        self.counter = 0
        self.arm_pos_pub = rospy.Publisher('/arm_1/arm_controller/position_command', JointPositions, queue_size = 10)
        self.arm_vel_pub = rospy.Publisher('/arm_1/arm_controller/velocity_command', JointVelocities, queue_size = 10)
        self.arm_js2js_sol = rospy.ServiceProxy('/From_JS_to_JS', JStoJS)
        self.joint_state_sub = rospy.Subscriber('/joint_states', JointState , self.callback, queue_size = 1)
        self.prev_time = datetime.now().time().microsecond
        self.curr_time = datetime.now().time().microsecond
    # Callback function subscriber. Read sensor messages for the current joint angles




    def callback(self, data):
        
        self.curr_time = datetime.now().time().microsecond
        error = [0 for i in range(5)]
        p_gain_arr = [0 for i in range(5)]
        def pub_msg_arr(msg_arr):
            for i in range(5):
                if (math.isnan(msg_arr[i].positions[0].value)):
                    pass
                else:
                    self.arm_pos_pub.publish(msg_arr[i])
                    time.sleep(0.01)
                    self.arm_pos_pub.publish(msg_arr[i])



        if (len(data.name) == 7):
            #print data.position
            [a,b,c,d,e,f,g] = data.position
            self.curr_joint_angles = [a,b,c,d,e]
        
        de_joint_angles = self.sol.joint_angles

        print "desired joint angles: "+str(de_joint_angles)+"\n"
        print "current joint angles: "+str(self.curr_joint_angles)+"\n"

        for i in range(5):
            error[i] = - de_joint_angles[4-i] + self.curr_joint_angles[4-i]
            if abs(error[i])<10.0**(-3):
                error[i] = 0
            p_gain_arr[i] = 1.0*error[i]

        joint_vel = p_gain_arr

        print "error is:" + str(list(reversed(error)))+"\n"



        vel_msg_arr = self.gen_vel_msg(joint_vel, p_gain_arr)
        self.pub_vel_msg_arr(vel_msg_arr)
        self.counter += 1
        self.prev_time = self.curr_time
        self.curr_time = datetime.now().time().microsecond
        time_diff = self.curr_time-self.prev_time
        print "time diff is " +str(time_diff) +"\n"


    def gen_pos_msg(self, joint_angles):
        msg_arr = [0 for i in range(5)]
        for i in range(5):
            msg_arr[i] = JointPositions()
            msg_arr[i].positions = [JointValue()]
            msg_arr[i].positions[0].timeStamp = rospy.Time.now()
            msg_arr[i].positions[0].joint_uri = "arm_joint_"+str(5-i)
            msg_arr[i].positions[0].unit = "rad"
            # not sure about this, truncate the angles in the ik solution when they are out of range. Usually the difference is small
            if (i == 3 and joint_angles[i] > 3.429):
                msg_arr[i].positions[0].value = 3.429
            elif (i == 2 and joint_angles[i] > -0.0157):
                msg_arr[i].positions[0].value = -0.01571
            else:
                msg_arr[i].positions[0].value = joint_angles[i]
        return msg_arr


    # Generating velocity messages to be published based on joint velocity array
    #
    # Params:
    # joint_vel - an array of joint vel, NaN for joints that do not move
    #
    # Returns:
    # msg_arr - an array of JointVelocities() messages

    def gen_vel_msg(self, joint_vel, p_gain_arr):
        msg_arr = [0 for i in range(5)]
        for i in range(5):
            msg_arr[i] = JointVelocities()
            msg_arr[i].velocities = [JointValue()]
            msg_arr[i].velocities[0].timeStamp = rospy.Time.now()
            msg_arr[i].velocities[0].joint_uri = "arm_joint_"+str(5-i)
            msg_arr[i].velocities[0].unit = "s^-1 rad"

            msg_arr[i].velocities[0].value = -joint_vel[i]*1
            
        return msg_arr



    # Publish an array of velocity messages from joint 1 to joint 5
    #
    # Params:
    # pub - the publisher for joint velocities
    # msg_arr - the array of messages specifying the velocity of each joint

    def pub_vel_msg_arr(self, msg_arr):
        #print msg_arr
        rate = rospy.Rate(20)
        for i in range(5):
            if (math.isnan(msg_arr[i].velocities[0].value)):
                pass
            else:
                self.arm_vel_pub.publish(msg_arr[i])
                rate.sleep()
                self.arm_vel_pub.publish(msg_arr[i])


    def pub_vel_msg_arr_neg(self, msg_arr):
        #print msg_arr
        rate = rospy.Rate(20)
        for i in range(5):
            if (math.isnan(msg_arr[i].velocities[0].value)):
                pass
            else:
                msg_arr[i].velocities[0].value = -msg_arr[i].velocities[0].value
                self.arm_vel_pub.publish(msg_arr[i])
                rate.sleep()
                self.arm_vel_pub.publish(msg_arr[i])


    # Publish an array of position messages from joint 1 to joint 5
    #
    # Params:
    # pub - the publisher for joint angles
    # msg_arr - the array of messages specifying the angle of each joint

    def pub_pos_msg_arr(self, msg_arr):
        #print msg_arr
        rate = rospy.Rate(20)
        for i in range(5):
            if (math.isnan(msg_arr[i].positions[0].value)):
                pass
            else:
                self.arm_pos_pub.publish(msg_arr[i])
                rate.sleep()
                time.sleep(0.2)
                self.arm_pos_pub.publish(msg_arr[i])

    # Publish an array of message from joint 5 to joint 1
    #
    # Params:
    # pub - the publisher for joint angles
    # msg_arr - the array of messages specifying the angle of each joint
    def pub_pos_msg_arr_rev(self, msg_arr):
        rate = rospy.Rate(20)
        for i in range(5):
            if (math.isnan(msg_arr[4-i].positions[0].value)):
                pass
            else:
                self.arm_pos_pub.publish(msg_arr[4-i])
                rate.sleep()
                time.sleep(0.2)
                self.arm_pos_pub.publish(msg_arr[4-i])


def main():
    

    #Drawing square here
    '''
    time.sleep(0.7)
    vel_con = velocityController(0.32,0.15,0.4)
    time.sleep(0.7)
    vel_con.sol = aic.solve_fully_constrained_ik(0.31,0.1,0.4)
    time.sleep(0.7)
    vel_con.sol = aic.solve_fully_constrained_ik(0.31,0.05,0.4)
    time.sleep(0.7)
    vel_con.sol = aic.solve_fully_constrained_ik(0.3,0.00,0.4)
    time.sleep(0.7)
    vel_con.sol = aic.solve_fully_constrained_ik(0.31,-0.05,0.4)
    time.sleep(0.7)
    vel_con.sol = aic.solve_fully_constrained_ik(0.32,-0.1,0.4)
    time.sleep(0.7)
    vel_con.sol = aic.solve_fully_constrained_ik(0.32,-0.15,0.4)

    
    time.sleep(0.7)
    vel_con.sol = aic.solve_fully_constrained_ik(0.3,-0.15,0.3)
    time.sleep(0.7)
    vel_con.sol = aic.solve_fully_constrained_ik(0.3,-0.15,0.2)
    time.sleep(0.7)
    vel_con.sol = aic.solve_fully_constrained_ik(0.31,-0.15,0.15)


    time.sleep(0.7)
    vel_con.sol = aic.solve_fully_constrained_ik(0.31,-0.15,0.15)
    time.sleep(0.7)
    vel_con.sol = aic.solve_fully_constrained_ik(0.31,-0.1,0.15)
    time.sleep(0.7)
    vel_con.sol = aic.solve_fully_constrained_ik(0.31,-0.05,0.15)
    time.sleep(0.7)
    vel_con.sol = aic.solve_fully_constrained_ik(0.32,0.00,0.15)
    time.sleep(0.7)
    vel_con.sol = aic.solve_fully_constrained_ik(0.32,0.05,0.15)
    time.sleep(0.7)
    vel_con.sol = aic.solve_fully_constrained_ik(0.31,0.1,0.15)
    time.sleep(0.7)
    vel_con.sol = aic.solve_fully_constrained_ik(0.31,0.15,0.15)



    time.sleep(0.7)
    vel_con.sol = aic.solve_fully_constrained_ik(0.31,0.15,0.15)
    time.sleep(0.7)
    vel_con.sol = aic.solve_fully_constrained_ik(0.31,0.15,0.2)
    time.sleep(0.7)
    vel_con.sol = aic.solve_fully_constrained_ik(0.31,0.15,0.3)
    time.sleep(0.7)
    vel_con.sol = aic.solve_fully_constrained_ik(0.31,0.15,0.4)
    time.sleep(3)
    vel_con.sol = aic.solve_fully_constrained_ik(0.1,0.15,0.4)
    '''

    #vel_con = velocityController(0.3,-0.3,0.3)
    #aic.go_to_xyz(0.32,0.02,0.3, vel_con.arm_pos_pub)
    #vel_con.get_curr_joint_angles()
    #vel_con.draw_line_smooth(0.32,0.3,0.3, 0.32,0.2,0.3)
    

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()



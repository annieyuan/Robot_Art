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


# Generating position messages to be published based on joint angle array
#
# Params:
# joint_angles - an array of joint angles, NaN for joints that do not move
#
# Returns:
# msg_arr - an array of JointPositions() messages

def gen_pos_msg(joint_angles):
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

def gen_vel_msg(joint_vel):
    msg_arr = [0 for i in range(5)]
    for i in range(5):
        msg_arr[i] = JointVelocities()
        msg_arr[i].velocities = [JointValue()]
        msg_arr[i].velocities[0].timeStamp = rospy.Time.now()
        msg_arr[i].velocities[0].joint_uri = "arm_joint_"+str(5-i)
        msg_arr[i].velocities[0].unit = "s^-1 rad"
        if (5-i == 2):
            # The calibration factors need to be tested. 0.075 for joint 2 and 0.08 for the rest seem like a good option here
            msg_arr[i].velocities[0].value = -joint_vel[i]*0.075
        else:
            msg_arr[i].velocities[0].value = -joint_vel[i]*0.08
    return msg_arr



# Publish an array of velocity messages from joint 1 to joint 5
#
# Params:
# pub - the publisher for joint velocities
# msg_arr - the array of messages specifying the velocity of each joint

def pub_vel_msg_arr(pub, msg_arr):
    #print msg_arr
    rate = rospy.Rate(20)
    for i in range(5):
        if (math.isnan(msg_arr[i].velocities[0].value)):
            pass
        else:
            pub.publish(msg_arr[i])
            rate.sleep()
            pub.publish(msg_arr[i])


def pub_vel_msg_arr_neg(pub, msg_arr):
    #print msg_arr
    rate = rospy.Rate(20)
    for i in range(5):
        if (math.isnan(msg_arr[i].velocities[0].value)):
            pass
        else:
            msg_arr[i].velocities[0].value = -msg_arr[i].velocities[0].value
            pub.publish(msg_arr[i])
            rate.sleep()
            pub.publish(msg_arr[i])


# Publish an array of position messages from joint 1 to joint 5
#
# Params:
# pub - the publisher for joint angles
# msg_arr - the array of messages specifying the angle of each joint

def pub_pos_msg_arr(pub, msg_arr):
    #print msg_arr
    rate = rospy.Rate(20)
    for i in range(5):
        if (math.isnan(msg_arr[i].positions[0].value)):
            pass
        else:
            pub.publish(msg_arr[i])
            rate.sleep()
            time.sleep(0.2)
            pub.publish(msg_arr[i])

# Publish an array of message from joint 5 to joint 1
#
# Params:
# pub - the publisher for joint angles
# msg_arr - the array of messages specifying the angle of each joint
def pub_pos_msg_arr_rev(pub, msg_arr):
    rate = rospy.Rate(20)
    for i in range(5):
        if (math.isnan(msg_arr[4-i].positions[0].value)):
            pass
        else:
            pub.publish(msg_arr[4-i])
            rate.sleep()
            time.sleep(0.2)
            pub.publish(msg_arr[4-i])

# Draw line based on the start and end position
#
# Params:
# x_s, y_s, z_s - start position coordinates in Cartesian Coordinate
# x_e, y_e, z_e - end position coordinates in Cartesian Coordinate
#
# Returns:
# the parameters for JS to JS solving

def xyz_2_xyz(x_s, y_s, z_s, x_e, y_e, z_e):
    start = JointPositions()
    end = JointPositions()
    start_v = JointVelocities()
    end_v = JointVelocities()
    point = JointTrajectoryPoint()
    start.positions = [JointValue() for i in range(5)]
    end.positions = [JointValue() for i in range(5)]
    start_v.velocities = [JointValue() for i in range(5)]
    end_v.velocities = [JointValue() for i in range(5)]
    sol_end = aic.solve_fully_constrained_ik(x_s, y_s, z_s)
    sol_start = aic.solve_fully_constrained_ik(x_e, y_e, z_e)
    start.positions[0].value = sol_start.joint_angles[0]
    start.positions[1].value = sol_start.joint_angles[1]
    start.positions[2].value = sol_start.joint_angles[2]
    start.positions[3].value = sol_start.joint_angles[3]
    start.positions[4].value = sol_start.joint_angles[4]
    end.positions[0].value = sol_end.joint_angles[0]
    end.positions[1].value = sol_end.joint_angles[1]
    end.positions[2].value = sol_end.joint_angles[2]
    end.positions[3].value = sol_end.joint_angles[3]
    end.positions[4].value = sol_end.joint_angles[4]

    joint_names = ["arm_joint_1","arm_joint_2","arm_joint_3","arm_joint_4","arm_joint_5"]

    for i in range(5):
        start.positions[i].joint_uri = joint_names[i]
        end.positions[i].joint_uri = joint_names[i]
        start_v.velocities[i].joint_uri = joint_names[i]
        end_v.velocities[i].joint_uri = joint_names[i]
        start.positions[i].unit = "rad"
        end.positions[i].unit = "rad"
        start_v.velocities[i].unit = "s^-1 rad"
        end_v.velocities[i].unit = "s^-1 rad"
        start_v.velocities[i].value = 0
        end_v.velocities[i].value = 0
    return (start, end, start_v, end_v)



# Draw line based on the start and end position
#
# Params:
# de_xs, de_ys, de_zs - start position coordinates in Cartesian Coordinate
# de_xe, de_ye, de_ze-de_zse - end position coordinates in Cartesian Coordinate

def draw_line_smooth(de_xs, de_ys, de_zs, de_xe, de_ye, de_ze):

    # This section (commented out) is attempting to shorten the line by scaling based on unit vector in 3D space
    '''
    (x, y, z) = (de_xe-de_xs, de_ye-de_ys, de_ze-de_zs)
    line_len = math.sqrt(math.pow(de_xe-de_xs, 2)+math.pow(de_ye-de_ys, 2)+math.pow(de_ze-de_zs, 2))
    unit_vec = (x/line_len, y/line_len, z/line_len)
    adjust_factor = line_len/20
    new_vec = [i * adjust_factor for i in unit_vec]
    de_xe = new_vec[0]+de_xs
    de_ye = new_vec[1]+de_ys
    de_ze = new_vec[2]+de_zs
    '''
    if((de_ys,de_zs)==(de_ye,de_ze)):
        de_ye = de_ys + 0.02

    (start, end, start_v, end_v) = xyz_2_xyz(de_xs, de_ys, de_zs, de_xe, de_ye, de_ze)
    js2js = JStoJS()
    js2js.start_pos = start
    js2js.end_pos = end
    js2js.start_vel = start_v
    js2js.end_vel = end_v
    js2js.max_vel = 0.05
    js2js.max_acc = 0.5
    arm_js2js_sol = rospy.ServiceProxy('/From_JS_to_JS', JStoJS)
    sol = arm_js2js_sol(js2js.start_pos,js2js.end_pos,js2js.start_vel,js2js.end_vel,js2js.max_vel,js2js.max_acc)
    #help(sol)
    msg_arr = sol.trajectory.points
    msg_len = len(msg_arr)
    print "there are " + str(msg_len) + " vel messages generated from (" + str(de_xs) + "," + str(de_ys)+ ","+ str(de_zs) + ") to (" + str(de_xe) + ","+ str(de_ye) + ","+ str(de_ze) + ")"


    #rospy.init_node('js_to_js trajectory')
    arm_pos_pub = rospy.Publisher('/arm_1/arm_controller/position_command', JointPositions, queue_size = 10)
    arm_vel_pub = rospy.Publisher('/arm_1/arm_controller/velocity_command', JointVelocities, queue_size = 10)
    #Move the end effector to the starting position
    msg = gen_pos_msg(msg_arr[0].positions)
    pub_pos_msg_arr_rev (arm_pos_pub, msg)
    #time.sleep(0.1)
    #pub_pos_msg_arr_rev (arm_pos_pub, msg)
    for i in range(msg_len):
        # This segment (commented out) is attempting to selevtively publish velocity messages based on JS to JS solution
        '''
        if (i%7 == 0):
            msg = gen_vel_msg(msg_arr[i].velocities)
            pub_vel_msg_arr (arm_vel_pub, msg)
        else:
            pass
        '''
        msg = gen_vel_msg(msg_arr[i].velocities)
        pub_vel_msg_arr (arm_vel_pub, msg)
    time.sleep(0.1)
    #Move the end effector to the end position
    #msg = gen_pos_msg(msg_arr[len(msg_arr)-1].positions)
    #pub_pos_msg_arr (arm_pos_pub, msg)



def smudge(de_xs, de_ys, de_zs, de_xe, de_ye, de_ze):
    if((de_ys,de_zs)==(de_ye,de_ze)):
        de_ye = de_ys - 0.02
    (start, end, start_v, end_v) = xyz_2_xyz(de_xs, de_ys, de_zs, de_xe, de_ye, de_ze)
    js2js = JStoJS()
    js2js.start_pos = start
    js2js.end_pos = end
    js2js.start_vel = start_v
    js2js.end_vel = end_v
    js2js.max_vel = 0.05
    js2js.max_acc = 0.5
    arm_js2js_sol = rospy.ServiceProxy('/From_JS_to_JS', JStoJS)
    sol = arm_js2js_sol(js2js.start_pos,js2js.end_pos,js2js.start_vel,js2js.end_vel,js2js.max_vel,js2js.max_acc)
    #help(sol)
    msg_arr = sol.trajectory.points
    msg_len = len(msg_arr)
    print "there are " + str(msg_len) + " vel messages generated from (" + str(de_xs) + "," + str(de_ys)+ ","+ str(de_zs) + ") to (" + str(de_xe) + ","+ str(de_ye) + ","+ str(de_ze) + ")"


    #rospy.init_node('js_to_js trajectory')
    arm_pos_pub = rospy.Publisher('/arm_1/arm_controller/position_command', JointPositions, queue_size = 10)
    arm_vel_pub = rospy.Publisher('/arm_1/arm_controller/velocity_command', JointVelocities, queue_size = 10)
    #Move the end effector to the starting position
    pos_msg = gen_pos_msg(msg_arr[0].positions)
    pub_pos_msg_arr (arm_pos_pub, pos_msg)
    time.sleep(0.01)
    pub_pos_msg_arr (arm_pos_pub, pos_msg)
    #vel_msg = [gen_vel_msg(msg_arr[i].velocities) for i in range(msg_len)]
    for i in range(msg_len):
        vel_msg = gen_vel_msg(msg_arr[i].velocities)
        pub_vel_msg_arr (arm_vel_pub, vel_msg)
    for i in range(msg_len):
        vel_msg = gen_vel_msg(msg_arr[i].velocities)
        pub_vel_msg_arr_neg (arm_vel_pub, vel_msg)
    for i in range(msg_len):
        vel_msg = gen_vel_msg(msg_arr[i].velocities)
        pub_vel_msg_arr (arm_vel_pub, vel_msg)
    time.sleep(0.01)



def main():
    
    #value = afc.main()
    rospy.init_node('arm_ik_control')
    draw_line_smooth(0.32,0.3,0.3, 0.32,0.2,0.3)
    #smudge(0.32,0.3,0.3,0.32,0.3,0.3)
    #draw_line_smooth(0.32,0.0,0.4,0.32,-0.15,0.4)
    #draw_line_smooth(0.3,-0.15,0.4,0.3,-0.15,0.25)
    #draw_line_smooth(0.3,-0.15,0.25,0.3,-0.15,0.1)
    #draw_line_smooth(0.3,-0.15,0.1,0.3,0.0,0.1)
    #draw_line_smooth(0.3,0.0,0.1,0.3,0.15,0.1)
    #draw_line_smooth(0.3,0.15,0.1,0.3,0.15,0.25)
    #draw_line_smooth(0.3,0.15,0.25,0.3,0.15,0.4)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()



#!/usr/bin/env python 
import rospy
import numpy as np
import math
import tf
import tf.transformations as trans

from tf.transformations import euler_from_quaternion
from tf.transformations import compose_matrix
from tf.transformations import is_same_transform
from geometry_msgs.msg import Twist,Vector3
from sensor_msgs.msg import JointState
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics 
from brics_actuator.msg import JointPositions
from brics_actuator.msg import JointValue
        




# Multiplies multiple matrices together.
#
# Params:
# A variable number of matrices to multiply.
#
# Returns:
# The product of the list of matrices.

def matmult(*x):
    return reduce(np.dot, x)

# Turns the parameter 3-vector into a 3x3 skew symmetric matrix.
#
# Params:
# w - a 3-vector.
#
# Returns:
# A 3x3 skew symmetric matrix.

def hat(w):
    return np.array([[0,-w[2],w[1]],
                     [w[2],0,-w[0]],
                     [-w[1],w[0],0]])

# Calculates a rotation matrix given an axis and rotation angle.
#
# Params:
# w - the axis of rotation
# th - the angle of rotation
#
# Returns:
# An SO(3) rotation matrix.

def expm(w,th):
    return np.identity(3) + hat(w)*np.sin(th) + np.dot(hat(w),hat(w))*(1-np.cos(th))

# Takes in a quaternion and returns the corresponding SO(3) matrix.
#
# Params:
# quat - the quaternion defining a rotation.
#
# Returns:
# An SO(3) matrix.

def quat_to_so3(quat):
    q = np.array(quat[0:3])
    th = quat[3]
    th = 2*np.arccos(th)
    if th != 0:
        q /= np.sin(th/2.0)
    else:
        q *= 0.0
    # return scipy.linalg.expm(hat(q)*th)
    return expm(q,th)


# Takes in a quatnernion and position and returns the corresponding SE(3) matrix.
#
# Params:
# quat - the quaternion defining a rotation.
# pos - a 3-vector defining a (x,y,z) location.
#
# Returns:
# An SE(3) matrix.

def quat_pos_to_se3(quat,pos):
    R = quat_to_so3(quat)
    g = np.hstack((R, np.array([pos.ravel()]).T))
    return np.vstack((g,[0,0,0,1]))

# Computes the inverse kinematics using damped least squares given a pose, a starting guess, a 
# damping parameter, and a maximum number of iterations.
#
# Params:
# kin - the kinematic model
# pose - the desired SE(3) pose of the end-effector.
# q0 - an intial guess for the inverse kinematic solution.
# lam - a tuning parameter for the damping.
# num - maximum number of iterations.
#
# Returns:
# The list of joint angles as a solution to the inverse kinematics.

def dls_ik(kin, pose, q0, lam=0.25, num=100):
    # get position and orientation:
    Rd = pose[0:3,0:3]
    Pd = pose[0:3,-1]
    # setup iterations:
    q = q0.copy()
    # run loop trying to get to target:
    for i in range(num):
        J = kin.jacobian(q)
        g = np.array(kin.forward(q))
        R = g[0:3,0:3]
        P = g[0:3,-1]
        Rdelt = matmult(Rd, R.T)
        rdelt_angles = np.array(trans.euler_from_matrix(Rdelt))
        e = np.hstack((Pd-P, rdelt_angles))
        dq = np.array(matmult(J.T,np.linalg.inv(matmult(J,J.T) + lam*np.eye(J.shape[0]))))
        q += matmult(dq,e)
        ##############################################################
        # should add a break condition and corresponding tolerance!! #
        ##############################################################
    return q

# Computes the inverse kinematics for the position only (no orientation) using damped 
# least squares given a pose, a starting guess, a damping parameter, and a maximum 
# number of iterations.
#
# Params:
# kin - the kinematic model
# pose - the desired SE(3) pose of the end-effector.
# q0 - an intial guess for the inverse kinematic solution.
# lam - a tuning parameter for the damping.
# num - maximum number of iterations.
#
# Returns:
# The list of joint angles as a solution to the inverse kinematics.

def dls_ik_position_only(kin, pose, q0, lam=0.25, num=100):
    # get position and orientation:
    Rd = pose[0:3,0:3]
    Pd = pose[0:3,-1].ravel()
    # setup iterations:
    q = q0.copy()
    # run loop trying to get to target:
    for i in range(num):
        J = kin.jacobian(q)
        J = J[0:3,:]
        g = np.array(kin.forward(q))
        P = g[0:3,-1]
        e = Pd-P.ravel()
        dq = np.array(matmult(J.T,np.linalg.inv(matmult(J,J.T) + lam*np.eye(J.shape[0]))))
        q += matmult(dq,e)
        ##############################################################
        # should add a break condition and corresponding tolerance!! #
        ##############################################################
    return q


##
# Performs an IK search while trying to balance the demands of reaching the goal,
# maintaining a posture, and prioritizing rotation or position.
def inverse_biased(kin, pose, q_init, q_bias, q_bias_weights, rot_weight=1.0, 
                   bias_vel=0.01, num_iter=100):
    # This code is potentially volatile
    q_out = q_init.copy()
    pos = pose[0:3,-1]
    rot = pose[0:3,0:3]
    for i in range(num_iter):
        # pos_fk, rot_fk = PoseConv.to_pos_rot(self.forward(q_out))
        g = np.array(kin.forward(q_out))
        pos_fk = g[0:3,-1]
        rot_fk = g[0:3,0:3]
        delta_twist = np.array(np.zeros(6))
        pos_delta = pos - pos_fk
        delta_twist[:3] = pos_delta
        rot_delta = np.eye(4)
        rot_delta[:3,:3] = rot * rot_fk.T
        rot_delta_angles = np.array(trans.euler_from_matrix(rot_delta))
        delta_twist[3:6] = rot_delta_angles
        J = np.array(kin.jacobian(q_out))
        J[3:6,:] *= np.sqrt(rot_weight)
        delta_twist[3:6] *= np.sqrt(rot_weight)
        J_tinv = matmult(np.linalg.inv(matmult(J.T,J) + np.diag(q_bias_weights)), J.T)
        q_bias_diff = q_bias - q_out
        q_bias_diff_normed = q_bias_diff * bias_vel / np.linalg.norm(q_bias_diff)
        delta_q = q_bias_diff_normed + matmult(J_tinv, (delta_twist - matmult(J, q_bias_diff_normed)))
        q_out += delta_q 
        q_out = np.array(kin.clip_joints_safe(q_out))
    return q_out

# Takes in a parameter values and clamps it between the parameter low and high limits.
#
# Params:
# value - the value to be limited.
# low_limit - the lower bound on the value.
# high_limit - the upper bound on the value.
#
# Returns:
# The limited value.

def low_high_limit(value, low_limit, high_limit):
    if low_limit >= high_limit:
        return value

    if value < low_limit:
        return low_limit

    if value > high_limit:
        return high_limit

    return value



# Takes in joint angles and a kinematics model. Print and returns the full position and orientation of the end effector
#
# Params:
# kin - kinematics model of the robot
# jointx_angles, the angle of joint x
#
# Returns:
# position and orientations

def solve_fk(kin, joint1_angle, joint2_angle, joint3_angle, joint4_angle, joint5_angle):
    fourbyfour = np.array(kin.forward([joint1_angle, joint2_angle, joint3_angle, joint4_angle, joint5_angle]))
    rot = fourbyfour[0:3,0:3]
    print "\n"
    print "Rotational Matrix:"
    print fourbyfour
    
    qw = np.sqrt((1+rot[0][0]+rot[1][1]+rot[2][2]))/2
    qx = (rot[2][1]-rot[1][2])/(4*qw)
    qy = (rot[0][2]-rot[2][0])/(4*qw)
    qz = (rot[1][0]-rot[0][1])/(4*qw)
    de_x = (fourbyfour)[0][3]
    de_y = (fourbyfour)[1][3]
    de_z = (fourbyfour)[2][3]

    print "Position:" + "\n" + "x: "+str(de_x)+" y: "+str(de_y)+" z: "+str(de_z)
    print "Quaternion: "+ "\n" + "qx: "+ str(qx) + " qy: "+ str(qx) +" qz: "+ str(qx) +" qw: "+ str(qw)
    return (de_x, de_y, de_z, qx, qy, qz, qw)

# A class representing the youBot arm that provides some control over it.

class YoubotArm:

    # Constructor.

    def __init__(self):

        youbot_urdf = URDF.from_parameter_server()
        self.kin_grasp = KDLKinematics(youbot_urdf, "arm_link_0", "arm_link_5")




def main():

    rospy.init_node('youbot_arm_control')

    try:
    	
        demo = YoubotArm()
        fk_sol = solve_fk(demo.kin_grasp, 3.73500459927, 1.3840749374, -1.24628291428,1.80825009285,2.92342649709)

    except rospy.ROSInterruptException:
        print "EXCEPTION"
        pass

    rospy.spin()


if __name__ == '__main__':
    main()

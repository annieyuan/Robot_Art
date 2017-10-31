#!/usr/bin/env python 


import rospy
import numpy as np
#import scipy.linalg
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
        

jointMax= [5.840139, 2.617989, -0.0157081, 3.42919, 5.641589]
jointMin= [0.01006921, 0.01006921, -5.0264, 0.0221391, 0.11062]

jointHome= [0.01007,0.01007,-0.15709,0.02214,0.1107]
jointCamera= [3.0,0.5,-0.9,0.1,3.0]
jointObject= [3.04171,0.63597,-1.017845,0.36284,2.876194]
jointGrasp = [3.04171,2.04427,-1.5189129,2.5434289757,2.8761944]
jointInitialize= [0.01007,.635971,-1.91989,1.04424,2.87619]

jointGuessForGrasp=[0.0, 0.0, 1.52, 1.84, -1.26, 2.4, 3.10]

armJointPosCandle = np.array([2.9496, 1.1344, -2.5482, 1.789, 2.9234])

gripperWidthAtGrasp = 0.00411
gripperWidthOpen = 0.0099

# Position and orientation above the grasping target
quat_above_grasp = np.array([0.601, 0.591, -0.372, 0.388])
pos_above_grasp = np.array([0.181, 0.778, 0.108])


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

# A class representing the youBot arm that provides some control over it.

class YoubotArm:

    # Constructor.

    def __init__(self):

        youbot_urdf = URDF.from_parameter_server()
        #self.kin_with_virtual = KDLKinematics(youbot_urdf, "virtual", "gripper_palm_link")
        self.kin_grasp = KDLKinematics(youbot_urdf, "arm_link_0", "arm_link_5")




def main():

    rospy.init_node('youbot_arm_control')

    try:
        demo = YoubotArm()
        fourbyfour = np.array(demo.kin_grasp.forward([2.16420827247, 1.3840749374, -1.24628291428,1.80825009285,2.92342649709]))
        rot = fourbyfour[0:3,0:3]
        print fourbyfour
        qw1 = np.sqrt((1+rot[0][0]+rot[1][1]+rot[2][2]))/2
        qx1 = (rot[2][1]-rot[1][2])/(4*qw1)
        qy1 = (rot[0][2]-rot[2][0])/(4*qw1)
        qz1 = (rot[1][0]-rot[0][1])/(4*qw1)


        de_x1 = (fourbyfour)[0][3]
        de_y1 = (fourbyfour)[1][3]
        de_z1 = (fourbyfour)[2][3]


        

        fourbyfour = np.array(demo.kin_grasp.forward([2.31685760087,1.04421656048,-0.87551280251,1.777338358,2.92342649709]))
        rot = fourbyfour[0:3,0:3]
        qw = np.sqrt((1+rot[0][0]+rot[1][1]+rot[2][2]))/2
        qx = (rot[2][1]-rot[1][2])/(4*qw)
        qy = (rot[0][2]-rot[2][0])/(4*qw)
        qz = (rot[1][0]-rot[0][1])/(4*qw)


        de_x = (fourbyfour)[0][3]
        de_y = (fourbyfour)[1][3]
        de_z = (fourbyfour)[2][3]
        #print (de_x1,de_y1, de_z1)
        #print (de_x, de_y, de_z)
        return (de_x1, de_y1, de_z1,  qx1, qy1, qz1, qw1, de_x, de_y, de_z,  qx, qy, qz, qw)


    except rospy.ROSInterruptException:
        print "EXCEPTION"
        pass

    rospy.spin()


if __name__ == '__main__':
    main()

#!/usr/bin/env python
import rospy
from brics_actuator.msg import JointPositions
from brics_actuator.msg import JointValue

import time

'''
This simple script show how to publish a message to a joint. The robot will wave hello to you when running this script
'''
def main(): 
    pub = rospy.Publisher('/arm_1/arm_controller/position_command', JointPositions)
    rospy.init_node('arm_test')

    msg = JointPositions()
    msg.positions = [JointValue()]
    msg.positions[0].timeStamp = rospy.Time.now()
    msg.positions[0].joint_uri = "arm_joint_1"
    msg.positions[0].unit = "rad"
    msg.positions[0].value = 2.0

    pub.publish(msg)

    time.sleep(1)
    pub.publish(msg)

    time.sleep(1)
    msg.positions[0].value = 0.1
    pub.publish(msg)

    msg.positions[0].joint_uri = "arm_joint_4"
    msg.positions[0].unit = "rad"
    def mk_msg(x):
        msg.positions[0].value = x

    for i in range(6):
        if (i%2 == 0):
            mk_msg(2.1)
        else:
            mk_msg(0.1)

        time.sleep(1.5)
        pub.publish(msg)

    msg.positions[0].joint_uri = "arm_joint_1"
    msg.positions[0].value = 2.0

    pub.publish(msg)

    rospy.spin()

if __name__ == "__main__":
    main()

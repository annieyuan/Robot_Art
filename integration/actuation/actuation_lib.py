#!/usr/bin/env python
import rospy
import velocity_controller as vc
import arm_ik_control as aic

vel_con = vc.velocityController(0.0,0.0,0.6)

def draw_line(x1,y1,z1,x2,y2,z2):
	vc.draw_line_3d(x1,y1,z1,x2,y2,z2,vel_con)

def go_to_xyz(x,y,z):
	vc.go_to_xyz(x,y,z,vel_con)

def go_to_xyz_vert(x,y,z):
	aic.go_to_xyz_vert(x,y,z,vel_con.arm_pos_pub)


def main():
    go_to_xyz(0.3,0.3,0.3)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
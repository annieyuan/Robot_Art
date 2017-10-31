#!/usr/bin/env python

#run this file with the following command:
#	rosrun robot_art test.py

import rospy
from brics_actuator.msg import JointPositions
from brics_actuator.msg import JointValue
import arm_fk_control as afc
import arm_ik_control as aic
import draw_art

#
#the purpose of this file is to have the robot test that it can reach all the points (i.e. paint,
#water, all parts of canvas) before attempting to draw art
#

def main():
    arm_pub = rospy.Publisher('/arm_1/arm_controller/position_command', JointPositions, queue_size = 10)

    art = draw_art.draw_art()

    colors = art.get_color_pos()
    brush_len = draw_art.PAINT_BRUSH_LENGTH

    print "Testing color positions..."

    aic.go_to_xyz_vert(0, 0.2, 0.1 + brush_len, arm_pub)
    on_left = True
    
    for color in colors:
	if (color == "none"):
	    on_left = False
	    #so the arm doesn't sweep through the canvas
	    print "Going to water at %s..." %colors[color]
	    aic.go_to_xyz_vert(0, 0.2, 0.1 + brush_len, arm_pub)
	    aic.go_to_xyz_vert(0, -0.2, 0.1 + brush_len, arm_pub)
	else:
	    on_left = True
	    print "Going to %s at %s..." %(color, colors[color])
	aic.lift_pen_vert(colors[color][0], colors[color][1], colors[color][2] + brush_len, arm_pub)
	aic.go_to_xyz_vert(colors[color][0], colors[color][1], colors[color][2] + brush_len, arm_pub)
	aic.lift_pen_vert(colors[color][0], colors[color][1], colors[color][2] + brush_len, arm_pub)
	if (color == "none"):
	    aic.go_to_xyz_vert(0, -0.2, 0.1 + brush_len, arm_pub)
	    aic.go_to_xyz_vert(0, 0.2, 0.1 + brush_len, arm_pub)
    

    print "Testing cleaning process..."
    if (on_left):
	aic.go_to_xyz_vert(0, 0.2, 0.1 + brush_len, arm_pub)
	aic.go_to_xyz_vert(0, -0.2, 0.1 + brush_len, arm_pub)
    art.clean_brush()

    aic.go_to_xyz_vert(0, -0.2, 0.1 + brush_len, arm_pub)
    aic.go_to_xyz_vert(0, 0.2, 0.1 + brush_len, arm_pub)

    print "Testing canvas positions..."

    top_left_pos = art.get_top_left_pos()
    top_right_pos = art.get_top_right_pos()

    bottom_left_pos = [top_left_pos[0], top_left_pos[1], top_left_pos[2] - draw_art.CANVAS_HEIGHT]
    bottom_right_pos = [top_right_pos[0], top_right_pos[1], top_right_pos[2] - draw_art.CANVAS_HEIGHT]

    print "Going to top left corner at %s..." %top_left_pos
    aic.lift_pen(top_left_pos[0] - brush_len, top_left_pos[1], top_left_pos[2], arm_pub)
    aic.go_to_xyz(top_left_pos[0] - brush_len, top_left_pos[1], top_left_pos[2], arm_pub)
    aic.lift_pen(top_left_pos[0] - brush_len, top_left_pos[1], top_left_pos[2], arm_pub)

    #aic.go_to_xyz_vert(0, 0.1, 0.1, arm_pub)
    '''
    print "Going to bottom left corner at %s..." %bottom_left_pos
    aic.lift_pen(bottom_left_pos[0] - brush_len, bottom_left_pos[1], bottom_left_pos[2], arm_pub)
    aic.go_to_xyz(bottom_left_pos[0] - brush_len, bottom_left_pos[1], bottom_left_pos[2], arm_pub)
    aic.lift_pen(bottom_left_pos[0] - brush_len, bottom_left_pos[1], bottom_left_pos[2], arm_pub)

    aic.go_to_xyz_vert(0, 0.1, 0.1, arm_pub)
    aic.go_to_xyz_vert(0, -0.1, 0.1, arm_pub)
    '''
    print "Going to top right corner at %s..." %top_right_pos
    aic.lift_pen(top_right_pos[0] - brush_len, top_right_pos[1],top_right_pos[2], arm_pub)
    aic.go_to_xyz(top_right_pos[0] - brush_len, top_right_pos[1],top_right_pos[2], arm_pub)
    aic.lift_pen(top_right_pos[0] - brush_len, top_right_pos[1],top_right_pos[2], arm_pub)
  
    #aic.go_to_xyz_vert(0, -0.1, 0.1, arm_pub)
    '''
    print "Going to bottom right corner at %s..." %bottom_right_pos
    aic.lift_pen(bottom_right_pos[0] - brush_len, bottom_right_pos[1], bottom_right_pos[2], arm_pub)
    aic.go_to_xyz(bottom_right_pos[0] - brush_len, bottom_right_pos[1], bottom_right_pos[2], arm_pub)
    aic.lift_pen(bottom_right_pos[0] - brush_len, bottom_right_pos[1], bottom_right_pos[2], arm_pub)
    '''
    aic.go_to_xyz_vert(0, -0.2, 0.1 + brush_len, arm_pub)

if __name__=='__main__':
    node = rospy.init_node("test", anonymous=True)
    print "Testing positions..."
    main()
    print "Done testing."

    try:
	rospy.spin()
    except KeyboardInterrupt:
	pass
	
    



#!/usr/bin/env python

#run this file with the following command:
#
#	rosrun robot_art draw_art.py "complete file_name_and_directory"
#
#Please run test.py to ensure the robot can reach all necessary points before running this program
#
import sys
import rospy
import tf
from brics_actuator.msg import JointPositions
from brics_actuator.msg import JointValue
from trajectory_js_2_js import draw_line_smooth
import arm_fk_control as afc
import arm_ik_control as aic
from math import sqrt
import json

#import apriltags_ros    #contains dictionary for apriltags topic type, will contain the type of tag and position for each tag detected

#GLOBAL VARIABLES:
#all values are in meters
MAX_PAINT_LENGTH = 0.1025
CANVAS_HEIGHT = 0.505
CANVAS_LENGTH = 0.4025

#used for calculating the position of the end of the paintbrush the end effector
#must be updated if using different paintbrushes
#further calculations also made assuming the end-effector is perpendicular to the canvas
PAINT_BRUSH_LENGTH = 0

class draw_art:

    def __init__(self, art_file=None):
	self.arm_pub = rospy.Publisher('/arm_1/arm_controller/position_command', JointPositions, queue_size = 10)
	self.current_color = "none"	

	#keep track of positions of things, in the ARM's coordinate space
	self.color_pos = {
		"red": [0.1, 0.3, 0.1], 
		"yellow": [0.1, 0.3, 0.1],
		"blue": [0.1, 0.3, 0.1],
		"black": [0.1, 0.3, 0.1],
		"white": [0.1, 0.3, 0.1],
		"none": [0.1, -0.3, 0.1]     #corresponds to the position of the water
	}   

	self.clean_pos_init = [0, -0.3, 0.1]  
	self.clean_pos = self.clean_pos_init
	self.arm_pos = [0.0, 0.0, 0.0]		#current position of the end-effector, updated with apriltags
	self.top_left_pos = [0.3, 0.3, 0.52]
	self.top_right_pos = [0.3, -0.3, 0.52]
	#will update many of these positions with callback function for apriltags later, see below for subscriber info

	#must be running freenect_launch freenect.launch and apriltags_ros example.launch with camera to initialize the following topic
	#self.apriltags = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, callback)	

	



	self.counter = 0    #keeps track of the length painted so the robot knows when it needs to dip in the paint again
	#open file and get dictionary containing layers and commands
	if (art_file == None):
	    self.layers = {}
	else:
	    self.layers = self.get_data(art_file)

	#number loops ensure the commands are executed in order even though
	#the dictionary may not be stored in order
	for i in range(1, len(self.layers)+1):
	    layer = "layer" + str(i)
	    for j in range(1, len(self.layers[layer])+1):
		cmd = "cmd" + str(j)
		color = self.layers[layer][cmd]["color"]


		start_2d = self.layers[layer][cmd]["path"]["startPos"].split(",")
		#convert strings to integers
		start_2d[0] = int(start_2d[0][1:])
		start_2d[1] = int(start_2d[1][:len(start_2d[1])-1])
		end_2d = self.layers[layer][cmd]["path"]["endPos"].split(",")
		#convert strings to integers
		end_2d[0] = int(end_2d[0][1:])
		end_2d[1] = int(end_2d[1][:len(end_2d[1])-1])


		start = self.get_3d_point(start_2d[0], start_2d[1])
		end = self.get_3d_point(end_2d[0], end_2d[1])

		self.counter += self.get_length(start, end)
		

		if (color != self.current_color):
		    if (self.current_color != "none"):
			self.clean_brush()
		    self.dip_in_color(color)
		    self.current_color = color
		elif (self.counter >= MAX_PAINT_LENGTH):
		    self.dip_in_color(self.current_color)


		#go to the starting position
		aic.lift_pen(start[0], start[1], start[2], arm_pub)
		#make stroke
		draw_line_smooth(start[0], start[1], start[2], end[0], end[1], end[2])
		aic.lift_pen(end[0], end[1], end[2], self.arm_pub)
		
	    

    #the following are get functions are for test.py
    def get_top_left_pos(self):
	return self.top_left_pos

    def get_top_right_pos(self):
	return self.top_right_pos

    def get_color_pos(self):
	return self.color_pos



    #following two functions are for an apriltags implementation
    def callback(self, data):
	pass

    #get position of the tag relative the arms position
    def get_tag_position(self):
	pass



    #parses the json file into usable information
    def get_data(self, art_file):
	raw_data = open(art_file).read()
	data = json.loads(raw_data)

	#makes cmd a dictionary
	for layer in data:
	    for cmd in data[layer]:
		data[layer][cmd] = data[layer][cmd][0]
	return data


    #given three dimensional points, returns the length
    def get_length(self, start, end):
	return sqrt((start[0] - end[0])**2 + (start[1] - end[1])**2 + (start[2] - end[2])**2)

    #given 2d point on canvas, returns 3d point in arm's space
    def get_3d_point(self, x_2d, y_2d):
	(x_2d, y_2d) = self.conv_pixels_to_m((x_2d, y_2d))

	x = self.top_left_pos[0] - PAINT_BRUSH_LENGTH    #so the robot does not punch the paintbrush through the canvas
	z = self.top_left_pos[2] - y_2d
	y = self.top_left_pos[1] - x_2d

	return (x, y, z)

    #assumes a resolution 72 pixels/inch (28.35 pixels/cm = 2834.65 pixels/m)
    #given a 2d value x in pixels with the assumed dpi, it determines the values in meters
    def conv_pixels_to_m(self, point):
	return (point[0] / 2834.65, point[1] / 2834.65)





    def clean_brush(self):
	#assumes the cleaning area is to the right of the robot
	self.clean_lines()

	#dip in water
	self.dip_in_color("none")

	self.clean_lines()



    def clean_lines(self):
	for i in range(3):
	    aic.lift_pen_vert(self.clean_pos[0], self.clean_pos[1], self.clean_pos[2] + PAINT_BRUSH_LENGTH, self.arm_pub)
	    is_valid = aic.go_to_xyz_vert(self.clean_pos[0], self.clean_pos[1], self.clean_pos[2], self.arm_pub)
	    if (not is_valid):
		self.clean_pos = self.clean_pos_init
	    else:
		self.clean_pos[0] -= 0.05
	    aic.draw_line_from_vert(self.clean_pos[0], (self.clean_pos[1], self.clean_pos[2] + PAINT_BRUSH_LENGTH), (self.clean_pos[1] + 0.05, self.clean_pos[2] + PAINT_BRUSH_LENGTH), self.arm_pub, False)
	    aic.lift_pen_vert(self.clean_pos[0], self.clean_pos[1], self.clean_pos[2] + PAINT_BRUSH_LENGTH, self.arm_pub)



    #dips the end of the paint brush in the desired paint color once
    def dip_in_color(self, color):	
	#go to point just above paint and dip in 
	aic.lift_pen_vert(self.color_pos[color][0], self.color_pos[color][1], self.color_pos[color][2] + PAINT_BRUSH_LENGTH, self.arm_pub)
	aic.go_to_xyz_vert(self.color_pos[color][0], self.color_pos[color][1], self.color_pos[color][2] + PAINT_BRUSH_LENGTH, self.arm_pub)
	aic.lift_pen_vert(self.color_pos[color][0], self.color_pos[color][1], self.color_pos[color][2] + PAINT_BRUSH_LENGTH, self.arm_pub)



    #TODO: def change_brush(self, brush_type):
	#self.clean_brush()
	#return brush to stand..
	#pick up new brush...





def main(args):
    rospy.init_node('draw_art', anonymous=True)
    if (len(args) >= 2):
        art_file = args[1]
	print "Starting painting..."
        art = draw_art(art_file)
	print "Done painting."
    else:
	print "You must provide a file"
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    

if __name__ == '__main__':
    main(sys.argv)



#!/usr/bin/env python

#run this file with the following command:
#
#    rosrun robot_art paint.py "complete file_name_and_directory"
#
#Please run test.py to ensure the robot can reach all necessary points before running this program
#
import sys
import rospy
import tf
from brics_actuator.msg import JointPositions
from brics_actuator.msg import JointValue
# from trajectory_js_2_js import draw_line_smooth
from trajectory_js_2_js import *
import arm_fk_control as afc
import arm_ik_control as aic
from math import sqrt
import json
import parser

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

class paint:

    def __init__(self):
	    self.arm_pub = rospy.Publisher('/arm_1/arm_controller/position_command', JointPositions, queue_size = 10)
	    
	    #keep track of positions of things, in the ARM's coordinate space
	    self.palette_height = 0.12
	    self.current_color = "none"   
	    self.color_pos = {
	        "red": [0.1, 0.3, self.palette_height], 
	        (58.0, 82.0, 136.0): [-0.03, 0.31, 0.18],
	        (97.0,124.0,152.0): [-0.10, 0.312, 0.18],
	        (28.0,31.0,34.0): [-0.07, 0.25, 0.19],
	        (154.0,169.0,156.0):[-0.15, 0.26, 0.185],
	        (155.0, 151.0, 75.0):[0.0, 0.25, 0.19],
	        (49.0,60.0,81.0):[-0.07, 0.16, 0.18],
	        "blue": [0.1, 0.3, self.palette_height],
	        "black": [-0.07, 0.285, 0.188],
	        "white": [0.1, 0.3,self.palette_height],
	        "none": [0.1, -0.25, 0.13]     #corresponds to the position of the water
	    }   

	    self.clean_pos_init = [0, -0.3, 0.226]  
	    self.clean_pos = self.clean_pos_init
	    self.top_left_pos = [0.3, 0.3, 0.52]
	    self.top_right_pos = [0.3, -0.3, 0.52]
	    #will update many of these positions with callback function for apriltags later, see below for subscriber info

	    #must be running freenect_launch freenect.launch and apriltags_ros example.launch with camera to initialize the following topic
	    #self.apriltags = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, callback)    

	    self.canvas_height = 0.3
	    self.canvas_length = 0.3
	    self.img_height = 200.0
	    self.img_length = 200.0

	    self.counter = 0    #keeps track of the length painted so the robot knows when it needs to dip in the paint again
	    #open file and get dictionary containing layers and commands


    def get_scale(self):
        return self.canvas_height/self.img_height

    def art_to_3d(self, pos_arr):
        scale = self.get_scale()
        de_xs = 0.33
        de_ys = (pos_arr[1][0]-self.img_length/2)*scale
        de_zs = (pos_arr[1][1])*scale+0.2
        de_xe = 0.33
        de_ye = (pos_arr[2][0]-self.img_length/2)*scale
        de_ze = (pos_arr[2][1])*scale+0.2
        return (de_xs, de_ys, de_zs, de_xe, de_ye, de_ze)


    def draw_pic(self,art_file):
    	def lift_pen_smooth(x, curr_y, curr_z):
    		draw_line_smooth(x, curr_y, curr_z, x-0.03, curr_y, curr_z-0.03)
    	def prepare_next(x, curr_y, curr_z, next_y, next_z):
    		draw_line_smooth(x-0.03, curr_y, curr_z, x-0.03, next_y, next_z)

        M = parser.main("/home/youbot/catkin_ws/src/robot_art/jsonfile/"+art_file)
        colr = M[0][0]
        self.dip_in_color(M[0][0])
        scale = self.get_scale()


        #preparing brush position after dipping the first color
        (de_xs, de_ys, de_zs, de_xe, de_ye, de_ze) = self.art_to_3d(M[0])
        aic.go_to_xyz(de_xs-0.05, de_ys, de_zs-0.03,self.arm_pub)
        counter = 1
        #yellow between 165 to 181
        #looping through all strokes
        for i in range(168,len(M)):
            (de_xs, de_ys, de_zs, de_xe, de_ye, de_ze) = self.art_to_3d(M[i])
            print "drawing " + str(i) + "/" + str(len(M)) + " lines"
            print "drawing from (" + str(de_xs) + "," + str(de_ys)+ ","+ str(de_zs) + ") to (" + str(de_xe) + ","+ str(de_ye) + ","+ str(de_ze) + ") in 3D space"
            print "drawing from (" + str(M[i][1][0]) + "," + str(M[i][1][1]) + ") to (" + str(M[i][2][0]) + ","+ str(M[i][2][1]) + ") in art space \n"
            #draw_line_smooth(de_xs, de_ys, de_zs, de_xe, de_ye, de_ze)

            if counter%2 == 0:
            	smudge(de_xs, de_ys, de_zs, de_xe, de_ye, de_ze)
            	lift_pen_smooth(de_xe, de_ye, de_ze)
            	if (counter%4 == 0):
	            	self.dip_in_color(M[i][0])
	            	aic.go_to_xyz(de_xs_next-0.02, de_ys_next, de_zs_next-0.035,self.arm_pub)

            try:
                (de_xs_next, de_ys_next, de_zs_next, de_xe_next, de_ye_next, de_ze_next) = self.art_to_3d(M[i+1])
            except IndexError:
                pass
            #prepare_next(de_xe, de_ye, de_ze, de_ye_next, de_ze_next-0.02)
            #switching color when color changes
            if (colr!= M[i+1][0]):
                print "switching to "+str(colr)+" color" 
                #self.clean_brush()
                self.dip_in_color(M[i+1][0])
                colr = M[i+1][0]

                aic.go_to_xyz(de_xs_next-0.02, de_ys_next, de_zs_next-0.035,self.arm_pub)
            else:
                print "same color"
                prepare_next(de_xe, de_ye, de_ze, de_ye_next, de_ze_next-0.035)
            counter = counter+1


            
            #aic.lift_pen(de_xe, de_ye, de_ze, self.arm_pub)
        #draw_line_smooth(0.3,0.3,0.3,0.0,0.0,0.6)
    
    #the following are get functions are for test.py
    def get_top_left_pos(self):
    	return self.top_left_pos

    def get_top_right_pos(self):
    	return self.top_right_pos

    def get_color_pos(self):
    	return self.color_pos

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
	    #self.dip_in_color("none")= '''none
	    '''
	    aic.go_to_xyz_vert(self.color_pos["none"][0], self.color_pos["none"][1], self.color_pos["none"][2] + PAINT_BRUSH_LENGTH +0.2, self.arm_pub)
	    aic.go_to_xyz_vert(self.color_pos["none"][0], self.color_pos["none"][1], self.color_pos["none"][2] + PAINT_BRUSH_LENGTH, self.arm_pub)
	    aic.go_to_xyz_vert(self.color_pos["none"][0], self.color_pos["none"][1], self.color_pos["none"][2] + PAINT_BRUSH_LENGTH + 0.2, self.arm_pub)
	    '''
	    self.clean_lines()



    def clean_lines(self):
	    for i in range(3):
	        aic.lift_pen(self.clean_pos[0], self.clean_pos[1], self.clean_pos[2] + PAINT_BRUSH_LENGTH, self.arm_pub)
	        aic.go_to_xyz(self.clean_pos[0], self.clean_pos[1], self.clean_pos[2], self.arm_pub)
	        if (self.clean_pos[2] <= 0.1):
	        	self.clean_pos = self.clean_pos_init
	        else:
		        self.clean_pos[2] -= 0.05
		        aic.draw_line(self.clean_pos[0], (self.clean_pos[1], self.clean_pos[2] + PAINT_BRUSH_LENGTH), (self.clean_pos[1], self.clean_pos[2] + 0.05 + PAINT_BRUSH_LENGTH), self.arm_pub, False)
		        #aic.lift_pen(self.clean_pos[0], self.clean_pos[1], self.clean_pos[2] + 0.05 + PAINT_BRUSH_LENGTH, self.arm_pub)



    #dips the end of the paint brush in the desired paint color once
    def dip_in_color(self, color):    
	    #go to point just above paint and dip in 
	    print "Dipping in color " + str(color)
	    aic.go_to_xyz(0.1,0.3,0.5, self.arm_pub)
	    aic.lift_pen_vert(self.color_pos[color][0], self.color_pos[color][1], self.color_pos[color][2] + PAINT_BRUSH_LENGTH, self.arm_pub)
	    aic.go_to_xyz_vert(self.color_pos[color][0], self.color_pos[color][1], self.color_pos[color][2] + PAINT_BRUSH_LENGTH, self.arm_pub)
	    aic.lift_pen_vert(self.color_pos[color][0], self.color_pos[color][1], self.color_pos[color][2] + PAINT_BRUSH_LENGTH, self.arm_pub)




def main(args):
    rospy.init_node('paint', anonymous=True)
    if (len(args) >= 2):
        art_file = args[1]
    	print "Starting painting..."
        art = paint()
        #art.dip_in_color("red")
        art.draw_pic(art_file)
    	print "Done painting."
    else:
    	print "You must provide a file"
    
	

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    

if __name__ == '__main__':
    main(sys.argv)


